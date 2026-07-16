// Fill out your copyright notice in the Description page of Project Settings.

#include "Robot/PickPlaceDispatcher.h"

#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Robot/RobotSimLog.h"
#include "Robot/Serial6DoFRobotActor.h"

namespace
{
	/**
	 * 한 프레임에 소진할 배급 스텝의 상한. 태스크 액터와 같은 이유다 — 프레임 hitch로 누적기가
	 * 커졌을 때 따라잡기가 다시 hitch를 유발하는 것을 막는다. 배급은 한 스텝에 최대 로봇 수만큼만
	 * 일어나므로 상한이 낮아도 실질적 손해가 없다.
	 */
	constexpr int32 MaxDispatchStepsPerFrame = 8;

	/** 거리 동률 판정 허용치 (cm²). 부동소수 오차로 tie-break가 흔들리지 않게 한다. */
	constexpr double DistanceTieEpsilonSq = 1.0;
}

APickPlaceDispatcher::APickPlaceDispatcher()
{
	PrimaryActorTick.bCanEverTick = true;

	SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot")));
}

void APickPlaceDispatcher::BeginPlay()
{
	Super::BeginPlay();

	// 초기화를 첫 Tick으로 미룬다. 여기서 하면 태스크 액터/로봇의 BeginPlay가 아직 안 돌았을 수 있고,
	// 그러면 등록이 비어 있거나 로봇의 VisualGraspPoint가 본에 붙기 전이라 도달성 캐시가 전부
	// 수학 EE 폴백으로 계산된다 — 조용히 틀린 캐시가 만들어진다.
	bPendingInit = true;
}

void APickPlaceDispatcher::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	WriteDispatchCsvToDisk();

	Super::EndPlay(EndPlayReason);
}

void APickPlaceDispatcher::RegisterTaskActor(APickPlaceTaskActor* TaskActor)
{
	if (!TaskActor)
	{
		return;
	}

	RegisteredTasks.AddUnique(TaskActor);

	UE_LOG(LogRobotSim, Log,
		TEXT("[PickPlaceDispatcher] 태스크 등록 — '%s' (우선순위 %d, 로봇 '%s')"),
		*TaskActor->GetName(), TaskActor->GetRobotPriority(),
		TaskActor->GetRobot() ? *TaskActor->GetRobot()->GetName() : TEXT("None"));
}

void APickPlaceDispatcher::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	if (bPendingInit)
	{
		bPendingInit = false;
		InitializeWorkPool();
	}

	if (!bInitialized)
	{
		return;
	}

	DrawDebugLayout();

	// 배급도 고정 스텝으로 전진시킨다. 태스크 액터와 같은 시간 기준을 써야 Dispatch.csv의
	// sim_time_sec과 PickPlace CSV의 time_s를 나란히 놓고 볼 수 있다.
	TimeAccumulatorSec += DeltaSeconds;

	int32 Steps = 0;
	while (TimeAccumulatorSec >= FixedTimeStepSec && Steps < MaxDispatchStepsPerFrame)
	{
		SimTimeSec += FixedTimeStepSec;
		TimeAccumulatorSec -= FixedTimeStepSec;
		++Steps;

		AssignTasksToIdleRobots();
	}

	if (Steps >= MaxDispatchStepsPerFrame)
	{
		TimeAccumulatorSec = 0.0;
	}
}

#pragma region Initialization

void APickPlaceDispatcher::InitializeWorkPool()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	if (RegisteredTasks.Num() == 0)
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[PickPlaceDispatcher] 등록된 태스크 액터가 없습니다 — 각 PickPlaceTask 액터의 Dispatcher 프로퍼티에 ")
			TEXT("이 dispatcher를 할당했는지 확인하세요. 배급할 대상이 없어 아무 일도 일어나지 않습니다."));
		return;
	}

	SortTaskActorsDeterministically();

	// 출발지 상판 위에 박스를 스폰한다. 슬롯 Z가 곧 박스 **바닥**이 앉을 높이다(HeightAboveSurface=0).
	TArray<FVector> SourceSlots;
	if (!FPickPlaceLayout::BuildSlotsOnSurface(SourceSurfaceActor, NumBoxesToSpawn, SourceSlotStrideCm,
			SourceSlotOffsetCm, /*HeightAboveSurfaceCm=*/0.0, SourceSlots))
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[PickPlaceDispatcher] SourceSurfaceActor가 없거나 바운드가 없어 박스를 스폰하지 못했습니다 — ")
			TEXT("팔레트 액터를 레벨에 배치하고 할당하세요."));
		return;
	}

	TArray<APickPlaceBoxActor*> SpawnedBoxes;
	FPickPlaceLayout::SpawnBoxesOnSlots(World, BoxClass ? BoxClass.Get() : APickPlaceBoxActor::StaticClass(),
		SourceSlots, SourceSurfaceActor->GetActorRotation(), this, SpawnedBoxes);

	Boxes.Reset();
	for (APickPlaceBoxActor* Box : SpawnedBoxes)
	{
		Boxes.Add(Box);
	}

	if (Boxes.Num() == 0)
	{
		UE_LOG(LogRobotSim, Warning, TEXT("[PickPlaceDispatcher] 박스가 하나도 스폰되지 않았습니다 (BoxClass 확인)."));
		return;
	}

	// 도착지 슬롯. 툴 목표 높이 = 상판 + 박스 **전체** 높이 — 파지점이 박스 윗면이라 박스가 툴에서
	// 아래로 전체 높이만큼 매달려 있기 때문이다 (태스크 액터의 BuildPalletSlots와 같은 규약).
	const double BoxHeightCm = Boxes[0] ? Boxes[0]->GetHeightCm() : 0.0;
	if (!FPickPlaceLayout::BuildSlotsOnSurface(DestinationSurfaceActor, Boxes.Num(), DestinationSlotStrideCm,
			DestinationSlotOffsetCm, BoxHeightCm, DestinationSlotsWorld))
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[PickPlaceDispatcher] DestinationSurfaceActor가 없거나 바운드가 없어 도착지 슬롯을 만들지 못했습니다 — ")
			TEXT("레일/컨베이어 액터를 레벨에 배치하고 할당하세요."));
		return;
	}

	BoxTaken.Init(false, Boxes.Num());
	SlotTaken.Init(false, DestinationSlotsWorld.Num());
	SlotActiveTask.Init(INDEX_NONE, DestinationSlotsWorld.Num());
	Blacklist.Init(false, RegisteredTasks.Num() * Boxes.Num());

	if (bEnableDispatchCsv)
	{
		DispatchCsvRows.Reset();
		DispatchCsvRows.Add(TEXT("sim_time_sec,event,robot,box,slot,reason"));
	}

	BuildReachCaches();

	bInitialized = true;

	UE_LOG(LogRobotSim, Log,
		TEXT("[PickPlaceDispatcher] 작업 풀 준비 완료 — 로봇 %d대, 박스 %d개, 도착지 슬롯 %d개, ")
		TEXT("고정 스텝 %.4fs, 슬롯 상호배제 %.0fcm"),
		RegisteredTasks.Num(), Boxes.Num(), DestinationSlotsWorld.Num(), FixedTimeStepSec, MinSlotSeparationCm);
}

void APickPlaceDispatcher::SortTaskActorsDeterministically()
{
	// RobotPriority → FName 순. 이 정렬이 멀티로봇 결정론의 근거다: 배급 순회 순서가 실행마다
	// 달라지면 겹친 작업 영역의 박스를 누가 가져갈지가 매번 바뀐다. UE의 액터 틱 순서도
	// TActorIterator 순서도 보장이 없으므로 명시적 키로 고정한다.
	// 술어가 TObjectPtr가 아니라 **참조**를 받는 것이 맞다: TArray<TObjectPtr<T>>::Sort는
	// TDereferenceWrapper가 `Predicate(*A, *B)`로 역참조해 넘긴다. TObjectPtr로 받으면 그 T&가
	// TObjectPtr로 암묵 변환되어 deprecated 경고가 난다.
	// (RegisterTaskActor가 null을 걸러내므로 역참조는 안전하다.)
	RegisteredTasks.Sort([](const APickPlaceTaskActor& A, const APickPlaceTaskActor& B)
	{
		if (A.GetRobotPriority() != B.GetRobotPriority())
		{
			return A.GetRobotPriority() < B.GetRobotPriority();
		}
		// 이름 문자열 비교 — FName의 FastLess는 내부 인덱스 순이라 실행마다 달라질 수 있어 쓰면 안 된다.
		return A.GetName() < B.GetName();
	});

	FString Order;
	for (int32 i = 0; i < RegisteredTasks.Num(); ++i)
	{
		Order += FString::Printf(TEXT("%s%s(%d)"), i > 0 ? TEXT(" → ") : TEXT(""),
			*RegisteredTasks[i]->GetName(), RegisteredTasks[i]->GetRobotPriority());
	}

	UE_LOG(LogRobotSim, Log, TEXT("[PickPlaceDispatcher] 배급 순서 확정 (RobotPriority → 이름): %s"), *Order);
}

void APickPlaceDispatcher::BuildReachCaches()
{
	const int32 NumTasks = RegisteredTasks.Num();
	const int32 NumBoxes = Boxes.Num();
	const int32 NumSlots = DestinationSlotsWorld.Num();

	BoxReachCache.Init(false, NumTasks * NumBoxes);
	SlotReachCache.Init(false, NumTasks * NumSlots);

	// 여기가 이 클래스에서 가장 비싼 지점이다 — (로봇 × (박스 + 슬롯))만큼 고정점 반복 IK가 돈다.
	// 사이클 시작 시점에 한 번만 치르고 이후 배급은 O(1) 조회가 되도록 캐시한다. 박스는 집히기
	// 전까지 움직이지 않으므로 캐시가 유효하다.
	for (int32 t = 0; t < NumTasks; ++t)
	{
		APickPlaceTaskActor* Task = RegisteredTasks[t];
		if (!Task)
		{
			continue;
		}

		int32 ReachableBoxes = 0;
		for (int32 b = 0; b < NumBoxes; ++b)
		{
			if (!Boxes[b])
			{
				continue;
			}
			const bool bReach = Task->CanReachGraspPointWorld(Boxes[b]->GetGraspPointWorld());
			BoxReachCache[t * NumBoxes + b] = bReach;
			ReachableBoxes += bReach ? 1 : 0;
		}

		int32 ReachableSlots = 0;
		for (int32 s = 0; s < NumSlots; ++s)
		{
			const bool bReach = Task->CanReachGraspPointWorld(DestinationSlotsWorld[s]);
			SlotReachCache[t * NumSlots + s] = bReach;
			ReachableSlots += bReach ? 1 : 0;
		}

		UE_LOG(LogRobotSim, Log,
			TEXT("[PickPlaceDispatcher] 도달성 캐시 '%s' — 박스 %d/%d, 슬롯 %d/%d 도달 가능"),
			*Task->GetName(), ReachableBoxes, NumBoxes, ReachableSlots, NumSlots);

		if (ReachableBoxes == 0 || ReachableSlots == 0)
		{
			UE_LOG(LogRobotSim, Warning,
				TEXT("[PickPlaceDispatcher] '%s'는 박스나 슬롯에 하나도 닿지 못합니다 — 이 로봇은 아무 일도 받지 못합니다. ")
				TEXT("로봇을 출발지/도착지 쪽으로 옮기거나, 태스크 액터의 도달 반경 탐색 로그를 확인하세요."),
				*Task->GetName());
		}
	}

	// 슬롯/박스마다 **누가 닿는지**를 찍는다. "슬롯 2/6"만으로는 어느 2개인지 알 수 없어 얼마나
	// 어느 쪽으로 옮겨야 할지 판단할 수 없다 — 슬롯은 직접 세팅하는 값이 아니라 도착지 액터
	// 위치/간격에서 계산되는 결과이므로, 어느 인덱스가 막혔는지가 곧 처방이 된다.
	UE_LOG(LogRobotSim, Log, TEXT("[PickPlaceDispatcher] ── 도착지 슬롯별 도달 로봇 ──"));
	for (int32 s = 0; s < NumSlots; ++s)
	{
		FString Who;
		for (int32 t = 0; t < NumTasks; ++t)
		{
			if (RegisteredTasks[t] && SlotReachCache[t * NumSlots + s])
			{
				Who += (Who.IsEmpty() ? TEXT("") : TEXT(", ")) + RegisteredTasks[t]->GetName();
			}
		}

		UE_LOG(LogRobotSim, Log,
			TEXT("[PickPlaceDispatcher]   슬롯 %d — 월드 (%.0f, %.0f, %.0f) : %s"),
			s, DestinationSlotsWorld[s].X, DestinationSlotsWorld[s].Y, DestinationSlotsWorld[s].Z,
			Who.IsEmpty() ? TEXT("**아무도 도달 불가**") : *Who);
	}

	UE_LOG(LogRobotSim, Log, TEXT("[PickPlaceDispatcher] ── 출발지 박스별 도달 로봇 ──"));
	for (int32 b = 0; b < NumBoxes; ++b)
	{
		if (!Boxes[b])
		{
			continue;
		}

		FString Who;
		for (int32 t = 0; t < NumTasks; ++t)
		{
			if (RegisteredTasks[t] && BoxReachCache[t * NumBoxes + b])
			{
				Who += (Who.IsEmpty() ? TEXT("") : TEXT(", ")) + RegisteredTasks[t]->GetName();
			}
		}

		const FVector Grasp = Boxes[b]->GetGraspPointWorld();
		UE_LOG(LogRobotSim, Log,
			TEXT("[PickPlaceDispatcher]   박스 %d — 파지점 월드 (%.0f, %.0f, %.0f) : %s"),
			b, Grasp.X, Grasp.Y, Grasp.Z, Who.IsEmpty() ? TEXT("**아무도 도달 불가**") : *Who);
	}
}

#pragma endregion

#pragma region Assignment

void APickPlaceDispatcher::AssignTasksToIdleRobots()
{
	const int32 NumBoxes = Boxes.Num();

	// 정렬된 순서로 순회한다 — 이 순서가 결정론의 절반이고, 나머지 절반은 아래 후보 선택의 tie-break다.
	for (int32 t = 0; t < RegisteredTasks.Num(); ++t)
	{
		APickPlaceTaskActor* Task = RegisteredTasks[t];
		if (!Task || !Task->IsIdle())
		{
			continue;
		}

		const ASerial6DoFRobotActor* Robot = Task->GetRobot();
		if (!Robot)
		{
			continue;
		}

		// 놓을 곳이 없으면 집어봐야 소용없다. 슬롯 선택은 박스와 무관하므로(도달성 + 상호배제만 본다)
		// 박스 루프 밖에서 한 번만 구한다.
		const int32 Slot = FindFreeSlotFor(t);
		if (Slot == INDEX_NONE)
		{
			continue; // 지금은 이 로봇이 쓸 슬롯이 없다 — 다른 로봇이 비켜주면 다음 스텝에 다시 본다
		}

		const FVector RobotBase = Robot->GetActorLocation();

		int32 BestBox = INDEX_NONE;
		double BestDistSq = TNumericLimits<double>::Max();

		for (int32 b = 0; b < NumBoxes; ++b)
		{
			if (BoxTaken[b] || !Boxes[b])
			{
				continue;
			}
			if (Blacklist[t * NumBoxes + b])
			{
				continue; // 이 로봇이 전에 실패한 박스 — 다른 로봇은 여전히 시도할 수 있다
			}
			if (!BoxReachCache[t * NumBoxes + b])
			{
				continue;
			}

			const double DistSq = FVector::DistSquared(Boxes[b]->GetGraspPointWorld(), RobotBase);

			// 가장 가까운 박스를 고르되, 거리가 사실상 같으면 **박스 인덱스가 작은 쪽**을 택한다.
			// 부동소수 오차로 tie-break가 흔들리면 재실행 시 할당이 달라져 결정론이 깨진다.
			const bool bCloser = DistSq < BestDistSq - DistanceTieEpsilonSq;
			const bool bTieButLowerIndex =
				FMath::Abs(DistSq - BestDistSq) <= DistanceTieEpsilonSq && b < BestBox;

			if (BestBox == INDEX_NONE || bCloser || bTieButLowerIndex)
			{
				BestDistSq = DistSq;
				BestBox = b;
			}
		}

		if (BestBox == INDEX_NONE)
		{
			continue;
		}

		BoxTaken[BestBox] = true;
		SlotTaken[Slot] = true;
		SlotActiveTask[Slot] = t;

		FPickPlaceTask NewTask;
		NewTask.Box = Boxes[BestBox];
		NewTask.DestinationSlotIndex = Slot;

		UE_LOG(LogRobotSim, Log,
			TEXT("[PickPlaceDispatcher] 배급 — '%s' ← 박스 %d (베이스에서 %.1fcm) → 슬롯 %d [t=%.2fs]"),
			*Task->GetName(), BestBox, FMath::Sqrt(BestDistSq), Slot, SimTimeSec);

		RecordDispatchEvent(TEXT("Assign"), Task, BestBox, Slot,
			FString::Printf(TEXT("nearest_reachable dist=%.1fcm"), FMath::Sqrt(BestDistSq)));

		Task->AssignTask(NewTask);

		// 배급이 일어났으면 상황이 바뀐 것이므로 정지 보고를 다시 열어둔다.
		bStallReported = false;
	}

	ReportStallOrCompletion();
}

void APickPlaceDispatcher::ReportStallOrCompletion()
{
	if (bStallReported)
	{
		return;
	}

	// 한 대라도 일하는 중이면 아직 판단할 때가 아니다.
	for (const TObjectPtr<APickPlaceTaskActor>& Task : RegisteredTasks)
	{
		if (Task && !Task->IsIdle())
		{
			return;
		}
	}

	const int32 NumBoxes = Boxes.Num();

	int32 Remaining = 0;
	for (int32 b = 0; b < NumBoxes; ++b)
	{
		Remaining += (Boxes[b] && !BoxTaken[b]) ? 1 : 0;
	}

	bStallReported = true;

	if (Remaining == 0)
	{
		UE_LOG(LogRobotSim, Log,
			TEXT("[PickPlaceDispatcher] **작업 완료** — 박스 %d개 전량 처리, 로봇 전부 유휴 [t=%.2fs]"),
			NumBoxes, SimTimeSec);
		// 이벤트 이름을 Complete와 구분한다: 이건 박스 하나의 완료가 아니라 **전체 요약**이다.
		// Complete를 재사용하면 CSV에서 Complete 행을 세었을 때 박스 개수보다 하나 많아져,
		// 집계하는 사람이 "완료 5건 / 박스 4개"로 오해한다.
		RecordDispatchEvent(TEXT("AllDone"), nullptr, INDEX_NONE, INDEX_NONE,
			FString::Printf(TEXT("all_boxes_done n=%d"), NumBoxes));
		WriteDispatchCsvToDisk();
		return;
	}

	// 로봇이 전부 유휴인데 박스가 남았다 = **작업이 미완인 채 정지했다.** 실패이므로 이유를 분해해 남긴다.
	int32 FreeSlots = 0;
	for (int32 s = 0; s < SlotTaken.Num(); ++s)
	{
		FreeSlots += SlotTaken[s] ? 0 : 1;
	}

	UE_LOG(LogRobotSim, Warning,
		TEXT("[PickPlaceDispatcher] **배급 정지 — 박스 %d/%d개가 미처리로 남았습니다** (로봇 전부 유휴) [t=%.2fs]. ")
		TEXT("빈 슬롯 %d/%d개."),
		Remaining, NumBoxes, SimTimeSec, FreeSlots, SlotTaken.Num());

	// 남은 박스마다 "왜 아무도 못 가져가는가"를 짚는다. 원인이 도달성인지 슬롯 고갈인지
	// 블랙리스트인지에 따라 처방이 완전히 다르다 (배치 이동 vs stride 축소 vs 무시).
	for (int32 b = 0; b < NumBoxes; ++b)
	{
		if (!Boxes[b] || BoxTaken[b])
		{
			continue;
		}

		int32 CanReachCount = 0;
		int32 NotBlacklistedCount = 0;
		int32 HasSlotCount = 0;

		for (int32 t = 0; t < RegisteredTasks.Num(); ++t)
		{
			if (!BoxReachCache[t * NumBoxes + b])
			{
				continue;
			}
			++CanReachCount;

			if (Blacklist[t * NumBoxes + b])
			{
				continue;
			}
			++NotBlacklistedCount;

			if (FindFreeSlotFor(t) != INDEX_NONE)
			{
				++HasSlotCount;
			}
		}

		const TCHAR* Cause =
			CanReachCount == 0        ? TEXT("**어느 로봇도 도달 불가** → 출발지를 로봇 쪽으로 옮기거나 SourceSlotStrideCm을 줄이세요")
			: NotBlacklistedCount == 0 ? TEXT("도달 가능한 로봇이 전부 이 박스에 실패해 블랙리스트에 올랐습니다")
			: HasSlotCount == 0        ? TEXT("**쓸 수 있는 빈 슬롯이 없음** → 도착지가 멀거나 DestinationSlotStrideCm이 넓어 슬롯이 도달 반경 밖입니다")
			:                            TEXT("원인 불명 (배급 로직 확인 필요)");

		UE_LOG(LogRobotSim, Warning, TEXT("[PickPlaceDispatcher]   박스 %d — %s"), b, *FString(Cause));
	}

	WriteDispatchCsvToDisk();
}

int32 APickPlaceDispatcher::FindFreeSlotFor(int32 TaskIndex) const
{
	const int32 NumSlots = DestinationSlotsWorld.Num();

	// 가장 낮은 인덱스부터 — 순서를 고정해야 재실행 시 같은 슬롯이 나온다.
	for (int32 s = 0; s < NumSlots; ++s)
	{
		if (SlotTaken[s])
		{
			continue;
		}
		if (!SlotReachCache[TaskIndex * NumSlots + s])
		{
			continue;
		}
		if (IsSlotTooCloseToActive(s, TaskIndex))
		{
			continue;
		}
		return s;
	}

	return INDEX_NONE;
}

bool APickPlaceDispatcher::IsSlotTooCloseToActive(int32 SlotIndex, int32 RequestingTaskIndex) const
{
	if (MinSlotSeparationCm <= 0.0)
	{
		return false; // 상호배제 비활성
	}

	// 다른 로봇이 지금 작업 중인 슬롯 근처에는 배급하지 않는다. 두 팔이 실제로 부딪칠 위험이 가장 큰
	// 지점이 "둘 다 같은 레일 근처에 놓으러 가는 순간"이기 때문이다. 이미 놓기가 끝난 슬롯
	// (SlotActiveTask == INDEX_NONE)은 팔이 떠났으므로 제약하지 않는다.
	for (int32 s = 0; s < DestinationSlotsWorld.Num(); ++s)
	{
		if (s == SlotIndex || SlotActiveTask[s] == INDEX_NONE || SlotActiveTask[s] == RequestingTaskIndex)
		{
			continue;
		}

		if (FVector::Dist(DestinationSlotsWorld[s], DestinationSlotsWorld[SlotIndex]) < MinSlotSeparationCm)
		{
			return true;
		}
	}

	return false;
}

#pragma endregion

#pragma region TaskLifecycle

void APickPlaceDispatcher::CompleteTask(APickPlaceTaskActor* TaskActor, const FPickPlaceTask& Task)
{
	const int32 BoxIndex = IndexOfBox(Task.Box);

	// 슬롯은 점유 상태로 남긴다 (박스가 거기 놓여 있으므로). 다만 활성 표시는 해제해야
	// 그 근처 슬롯이 다시 배급 가능해진다 — 팔이 이미 떠났기 때문이다.
	if (DestinationSlotsWorld.IsValidIndex(Task.DestinationSlotIndex))
	{
		SlotActiveTask[Task.DestinationSlotIndex] = INDEX_NONE;
	}

	UE_LOG(LogRobotSim, Log,
		TEXT("[PickPlaceDispatcher] 완료 — '%s' 박스 %d → 슬롯 %d [t=%.2fs]"),
		TaskActor ? *TaskActor->GetName() : TEXT("?"), BoxIndex, Task.DestinationSlotIndex, SimTimeSec);

	RecordDispatchEvent(TEXT("Complete"), TaskActor, BoxIndex, Task.DestinationSlotIndex, TEXT(""));

	// 완료/정지 판정은 ReportStallOrCompletion이 한다 — 여기서 "미배급 0개"를 찍으면 **배급됨**과
	// **완료됨**을 혼동시킨다(마지막 박스를 배급한 순간에 뜨므로). 판정 기준은 "로봇이 전부 유휴인가"다.
	bStallReported = false;
}

void APickPlaceDispatcher::ReturnTask(
	APickPlaceTaskActor* TaskActor, const FPickPlaceTask& Task, bool bBlacklist, const FString& Reason)
{
	const int32 BoxIndex = IndexOfBox(Task.Box);

	// **박스와 슬롯을 함께 반납한다.** 박스만 되돌리면 슬롯이 영구 점유로 새어 뒤쪽 박스들이
	// 놓을 곳을 잃는다 — FPickPlaceTask가 둘을 한 몸으로 들고 다니는 이유가 이것이다.
	if (Boxes.IsValidIndex(BoxIndex))
	{
		BoxTaken[BoxIndex] = false;
	}
	if (DestinationSlotsWorld.IsValidIndex(Task.DestinationSlotIndex))
	{
		SlotTaken[Task.DestinationSlotIndex] = false;
		SlotActiveTask[Task.DestinationSlotIndex] = INDEX_NONE;
	}

	UE_LOG(LogRobotSim, Warning,
		TEXT("[PickPlaceDispatcher] 반납 — '%s' 박스 %d / 슬롯 %d 되돌림 (%s) [t=%.2fs]"),
		TaskActor ? *TaskActor->GetName() : TEXT("?"), BoxIndex, Task.DestinationSlotIndex, *Reason, SimTimeSec);

	RecordDispatchEvent(TEXT("Return"), TaskActor, BoxIndex, Task.DestinationSlotIndex, Reason);

	if (!bBlacklist || !TaskActor || !Boxes.IsValidIndex(BoxIndex))
	{
		return;
	}

	// (로봇, 박스) 실패를 기록한다. 안 하면 같은 로봇이 즉시 그 박스를 다시 가져가 또 실패하고,
	// 배급→중단→배급이 무한 반복되어 화면에서 팔이 같은 자리를 영원히 버벅인다.
	const int32 TaskIndex = RegisteredTasks.IndexOfByKey(TaskActor);
	if (TaskIndex != INDEX_NONE)
	{
		Blacklist[TaskIndex * Boxes.Num() + BoxIndex] = true;

		UE_LOG(LogRobotSim, Warning,
			TEXT("[PickPlaceDispatcher] 블랙리스트 — '%s'는 박스 %d를 다시 시도하지 않습니다 (다른 로봇은 가능)"),
			*TaskActor->GetName(), BoxIndex);

		RecordDispatchEvent(TEXT("Blacklist"), TaskActor, BoxIndex, Task.DestinationSlotIndex, Reason);
	}
}

bool APickPlaceDispatcher::GetDestinationSlotWorld(int32 SlotIndex, FVector& OutWorld) const
{
	if (!DestinationSlotsWorld.IsValidIndex(SlotIndex))
	{
		return false;
	}

	OutWorld = DestinationSlotsWorld[SlotIndex];
	return true;
}

FColor APickPlaceDispatcher::ReachColor(int32 ReachableRobotCount)
{
	// 0 = 아무도 못 감 (배치를 고쳐야 함) / 1 = 한 대만 / 2+ = 겹침 (배급 경쟁·슬롯 상호배제 구간)
	if (ReachableRobotCount <= 0)
	{
		return FColor::Red;
	}
	return ReachableRobotCount == 1 ? FColor::Yellow : FColor::Green;
}

void APickPlaceDispatcher::DrawDebugLayout() const
{
	const UWorld* World = GetWorld();
	if (!bDrawDebugLayout || !World)
	{
		return;
	}

	const int32 NumTasks = RegisteredTasks.Num();
	const int32 NumBoxes = Boxes.Num();
	const int32 NumSlots = DestinationSlotsWorld.Num();

	// 표면 바운드 — "행이 상판을 벗어났나"가 한눈에 보인다. 로그의 '상판 밖으로 나간 슬롯 N개'와
	// 같은 사실을 눈으로 확인하는 수단이다.
	for (const AActor* Surface : { SourceSurfaceActor.Get(), DestinationSurfaceActor.Get() })
	{
		if (!Surface)
		{
			continue;
		}
		const FBox B = Surface->GetComponentsBoundingBox(/*bNonColliding=*/true);
		if (B.IsValid)
		{
			DrawDebugBox(World, B.GetCenter(), B.GetExtent(), FColor(80, 80, 160), false, -1.0f, 0, 1.0f);
		}
	}

	// 도착지 슬롯 — 행을 선으로 잇고 각 슬롯을 도달 로봇 수로 색칠한다.
	for (int32 s = 0; s < NumSlots; ++s)
	{
		int32 ReachCount = 0;
		for (int32 t = 0; t < NumTasks; ++t)
		{
			ReachCount += (RegisteredTasks[t] && SlotReachCache[t * NumSlots + s]) ? 1 : 0;
		}

		const FColor Color = ReachColor(ReachCount);
		const FVector P = DestinationSlotsWorld[s];

		// 점유된 슬롯은 속을 채워 구분한다 (이미 박스가 놓였거나 배급 중).
		DrawDebugSphere(World, P, 12.0f, 8, Color, false, -1.0f, 0, SlotTaken[s] ? 3.0f : 1.0f);
		DrawDebugString(World, P + FVector(0, 0, 20), FString::Printf(TEXT("S%d"), s), nullptr, Color, 0.0f);

		if (s > 0)
		{
			DrawDebugLine(World, DestinationSlotsWorld[s - 1], P, FColor(120, 120, 120), false, -1.0f, 0, 1.0f);
		}
	}

	// 출발지 박스 — 아직 안 가져간 것만 그린다 (들려서 이동 중인 박스에 마커가 따라다니면 어지럽다).
	for (int32 b = 0; b < NumBoxes; ++b)
	{
		if (!Boxes[b] || BoxTaken[b])
		{
			continue;
		}

		int32 ReachCount = 0;
		for (int32 t = 0; t < NumTasks; ++t)
		{
			ReachCount += (RegisteredTasks[t] && BoxReachCache[t * NumBoxes + b]) ? 1 : 0;
		}

		const FVector P = Boxes[b]->GetGraspPointWorld();
		const FColor Color = ReachColor(ReachCount);

		DrawDebugSphere(World, P, 12.0f, 8, Color, false, -1.0f, 0, 1.0f);
		DrawDebugString(World, P + FVector(0, 0, 20), FString::Printf(TEXT("B%d"), b), nullptr, Color, 0.0f);
	}

	// 로봇 베이스 — 어느 로봇이 어느 쪽인지 알아야 "행을 어느 방향으로 옮길지"가 나온다.
	for (int32 t = 0; t < NumTasks; ++t)
	{
		const APickPlaceTaskActor* Task = RegisteredTasks[t];
		if (!Task || !Task->GetRobot())
		{
			continue;
		}

		const FVector Base = Task->GetRobot()->GetActorLocation();
		DrawDebugSphere(World, Base, 25.0f, 12, FColor::Cyan, false, -1.0f, 0, 2.0f);
		DrawDebugString(World, Base + FVector(0, 0, 40),
			FString::Printf(TEXT("%s (P%d)"), *Task->GetName(), Task->GetRobotPriority()), nullptr, FColor::Cyan, 0.0f);
	}
}

int32 APickPlaceDispatcher::IndexOfBox(const APickPlaceBoxActor* Box) const
{
	for (int32 i = 0; i < Boxes.Num(); ++i)
	{
		if (Boxes[i] == Box)
		{
			return i;
		}
	}
	return INDEX_NONE;
}

#pragma endregion

#pragma region Logging

void APickPlaceDispatcher::RecordDispatchEvent(
	const TCHAR* Event, const APickPlaceTaskActor* TaskActor, int32 BoxIndex, int32 SlotIndex, const FString& Reason)
{
	if (!bEnableDispatchCsv)
	{
		return;
	}

	// reason에 쉼표/개행이 섞이면 컬럼이나 행이 밀린다.
	FString SafeReason = Reason;
	SafeReason.ReplaceInline(TEXT(","), TEXT(";"));
	SafeReason.ReplaceInline(TEXT("\n"), TEXT(" "));
	SafeReason.ReplaceInline(TEXT("\r"), TEXT(" "));

	// 길면 자른다. abort 사유 전문(수백 자)이 그대로 들어오면 한 행이 CSV를 도배해 표로 읽을 수 없다.
	// 전문은 이미 로그에 있으므로 여기는 어느 단계에서 왜 실패했는지만 알아볼 정도면 된다.
	constexpr int32 MaxReasonLen = 60;
	if (SafeReason.Len() > MaxReasonLen)
	{
		SafeReason = SafeReason.Left(MaxReasonLen) + TEXT("...");
	}

	DispatchCsvRows.Add(FString::Printf(TEXT("%.5f,%s,%s,%d,%d,%s"),
		SimTimeSec, Event, TaskActor ? *TaskActor->GetName() : TEXT("?"), BoxIndex, SlotIndex, *SafeReason));
}

void APickPlaceDispatcher::WriteDispatchCsvToDisk()
{
	// 헤더 한 줄뿐이면 배급 이벤트가 없었다는 뜻이므로 빈 파일을 만들지 않는다.
	if (!bEnableDispatchCsv || DispatchCsvRows.Num() <= 1)
	{
		return;
	}

	const FString Directory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("RobotSim"));
	const FString FilePath = FPaths::Combine(Directory, DispatchCsvFileName);

	IFileManager::Get().MakeDirectory(*Directory, /*Tree=*/true);

	// UTF-8(BOM 포함) 강제 — reason에 한글이 들어가므로 기본값(AutoDetect)은 UTF-16을 쓴다. 그러면
	// pandas/awk가 못 읽어 "할당의 증거"라는 이 파일의 용도 자체가 무너진다.
	if (FFileHelper::SaveStringArrayToFile(DispatchCsvRows, *FilePath, FFileHelper::EEncodingOptions::ForceUTF8))
	{
		UE_LOG(LogRobotSim, Log,
			TEXT("[PickPlaceDispatcher] 배급 CSV %d행 기록: %s"), DispatchCsvRows.Num() - 1, *FilePath);
	}
	else
	{
		UE_LOG(LogRobotSim, Warning, TEXT("[PickPlaceDispatcher] 배급 CSV 기록 실패: %s"), *FilePath);
	}

	// 헤더만 남겨 재기록 시 중복 append를 막는다.
	DispatchCsvRows.SetNum(1);
}

#pragma endregion
