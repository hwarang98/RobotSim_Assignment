// Fill out your copyright notice in the Description page of Project Settings.

#include "Robot/PickPlaceTaskActor.h"

#include "Components/PoseableMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/Engine.h"
#include "EngineUtils.h"
#include "Engine/SkinnedAsset.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Robot/PickPlaceDispatcher.h"
#include "Robot/RobotDlsIK.h"
#include "Robot/RobotDynamicsRNEA.h"
#include "Robot/RobotSimLog.h"
#include "Robot/Serial6DoFModel.h"
#include "Robot/Serial6DoFRobotActor.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
	/**
	 * 한 프레임에 소진할 수 있는 고정 스텝의 상한.
	 * 프레임 hitch로 누적기가 커졌을 때 따라잡기 스텝이 다시 hitch를 유발하는 "죽음의 나선"을 막는다.
	 * 상한에 걸리면 남은 누적 시간은 버린다 — 시뮬레이션 시간이 실시간보다 느려질 뿐,
	 * 궤적/CSV의 스텝 간격은 여전히 정확히 FixedTimeStepSec다 (결정론 유지).
	 */
	constexpr int32 MaxFixedStepsPerFrame = 8;

	/**
	 * quintic smoothstep S(u) = 6u⁵ − 15u⁴ + 10u³.
	 * S(0)=0, S(1)=1, S'(0)=S'(1)=0, S''(0)=S''(1)=0 이므로 궤적의 시작/끝에서 속도와 가속도가 모두 0이다.
	 * cubic smoothstep(3u²−2u³)과 달리 가속도까지 연속이라 이후 토크 제어 단계에서 지령 토크에
	 * 계단 불연속이 생기지 않는다.
	 */
	FORCEINLINE double QuinticSmoothStep(double U)
	{
		return U * U * U * (U * (U * 6.0 - 15.0) + 10.0);
	}

	/** quintic smoothstep의 최대 속도 / 평균 속도 비율 = S'(0.5) = 15/8. 소요시간 역산의 안전계수다. */
	constexpr double QuinticPeakVelocityRatio = 15.0 / 8.0;

	/** 적재 시 박스 바닥을 지지면에서 살짝 띄우는 여유 (cm). 놓는 순간 지지면과 겹쳐 튀는 것을 막는다. */
	constexpr double PlaceClearanceCm = 0.5;

	/**
	 * 프레임타임 HUD 표시의 EMA 계수. 원시 DeltaSeconds는 프레임마다 크게 튀어 숫자를 읽을 수 없다.
	 * 평활의 대가로 스파이크가 과소평가되므로, 이 값은 **표시용이지 프로파일링용이 아니다** —
	 * 정확한 프레임 분석은 `stat unit`을 쓸 것.
	 */
	constexpr double FrameTimeSmoothingAlpha = 0.1;

	/**
	 * UI 텔레메트리용 RNEA 옵션.
	 *
	 * 마찰을 켜는 이유: qd가 유한차분으로 실제 값이 들어오므로 마찰 항(점성 + 쿨롱)이 물리적 의미를
	 * 갖는다. 로터 관성을 끄는 이유: 그 항은 qdd에만 곱해지는데 여기선 qdd=0이라 아무 효과가 없다 —
	 * 켜면 "고려했다"는 거짓 정밀도만 생긴다.
	 */
	FRobotRNEAOptions MakeTelemetryRNEAOptions()
	{
		FRobotRNEAOptions Options;
		Options.bEnableGravity = true;
		Options.bIncludeFriction = true;
		Options.bIncludeRotorInertia = false;
		return Options;
	}
}

#pragma region APickPlaceBoxActor

APickPlaceBoxActor::APickPlaceBoxActor()
{
	PrimaryActorTick.bCanEverTick = false;

	// 박스 본체이자 물리 바디. 엔진 기본 Cube(한 변 100cm)를 15cm로 줄여 로봇 작업 반경에 맞춘다.
	// BP로 서브클래싱해 메시/스케일을 바꿔도 된다 — 파지 높이는 바운드에서 읽는다.
	BoxMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BoxMesh"));
	SetRootComponent(BoxMesh);
	BoxMesh->SetRelativeScale3D(FVector(0.15));

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeFinder(TEXT("/Engine/BasicShapes/Cube.Cube"));
	if (CubeFinder.Succeeded())
	{
		BoxMesh->SetStaticMesh(CubeFinder.Object);
	}

	// 놓인 뒤 바닥/다른 박스와 부딪혀야 하므로 완전한 물리 바디로 설정한다.
	// (로봇 링크 메시는 NoCollision이라 팔과 박스는 서로 통과한다 — 충돌 회피는 범위 밖이고,
	//  대신 ApproachOffsetCm 수직 진입으로 시각적 간섭을 줄인다.)
	BoxMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	BoxMesh->SetCollisionObjectType(ECC_PhysicsBody);
	BoxMesh->SetCollisionResponseToAllChannels(ECR_Block);
	BoxMesh->SetSimulatePhysics(true);
}

void APickPlaceBoxActor::BeginPlay()
{
	Super::BeginPlay();

	if (BoxMesh)
	{
		// Chaos의 밀도 기반 자동 질량 대신 MassKg를 그대로 쓴다.
		// 동역학 파라미터의 유일한 정의처를 에디터 프로퍼티 한 곳으로 모으기 위함이다.
		BoxMesh->SetMassOverrideInKg(NAME_None, static_cast<float>(MassKg), true);
	}
}

FVector APickPlaceBoxActor::GetGraspPointWorld() const
{
	if (!BoxMesh)
	{
		return GetActorLocation();
	}

	// 바운드는 월드 스케일/피벗이 모두 반영된 값이다. 윗면 중심 = 바운드 중심에서 +Z로 절반 높이.
	const FBoxSphereBounds B = BoxMesh->Bounds;
	return FVector(B.Origin.X, B.Origin.Y, B.Origin.Z + B.BoxExtent.Z);
}

double APickPlaceBoxActor::GetHeightCm() const
{
	return BoxMesh ? BoxMesh->Bounds.BoxExtent.Z * 2.0 : 0.0;
}

FVector APickPlaceBoxActor::GetBoundsExtentCm() const
{
	return BoxMesh ? BoxMesh->Bounds.BoxExtent : FVector::ZeroVector;
}

double APickPlaceBoxActor::GetBoundsBottomZWorld() const
{
	return BoxMesh ? BoxMesh->Bounds.Origin.Z - BoxMesh->Bounds.BoxExtent.Z : GetActorLocation().Z;
}

void APickPlaceBoxActor::BeginGrasp()
{
	if (!BoxMesh || bGrasped)
	{
		return;
	}

	// 물리를 끄면 이후 트랜스폼 설정이 Chaos와 충돌하지 않고 그대로 반영된다.
	// 실제 파지 추종(매 스텝 트랜스폼 갱신)은 APickPlaceTaskActor가 담당한다.
	BoxMesh->SetSimulatePhysics(false);

	// 파지 중에는 콜리전도 끈다. 물리만 끄면 박스가 "콜리전 있는 kinematic 바디"가 되어, 팔이 휘두를 때
	// 아직 바닥에 놓인 다른 박스를 그대로 후려쳐 날려버린다 (실제로 박스 1이 90cm 밀려났다).
	// 그리퍼가 박스를 소유하는 동안 그 박스는 씬과 상호작용하지 않는다고 보는 편이 이 슬라이스에 맞다 —
	// 어차피 로봇 링크 메시도 NoCollision이라 팔과 박스는 서로 통과한다.
	BoxMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	bGrasped = true;

	UE_LOG(LogRobotSim, Log, TEXT("[APickPlaceBoxActor] %s 파지 — 물리/콜리전 비활성 (질량 %.2fkg)"), *GetName(), MassKg);
}

void APickPlaceBoxActor::EndGrasp()
{
	if (!BoxMesh || !bGrasped)
	{
		return;
	}

	BoxMesh->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
	BoxMesh->SetSimulatePhysics(true);

	// 놓는 순간의 속도를 명시적으로 0으로 만든다. 물리를 켜면 Chaos가 직전 kinematic 이동으로부터
	// 속도를 추정할 수 있는데, 그러면 정지 상태에서 놓았는데도 박스가 튀어나갈 수 있다.
	BoxMesh->SetPhysicsLinearVelocity(FVector::ZeroVector);
	BoxMesh->SetPhysicsAngularVelocityInRadians(FVector::ZeroVector);
	bGrasped = false;

	UE_LOG(LogRobotSim, Log, TEXT("[APickPlaceBoxActor] %s 파지 해제 — 물리/콜리전 활성"), *GetName());
}

#pragma endregion

#pragma region APickPlaceTaskActor_Lifecycle

APickPlaceTaskActor::APickPlaceTaskActor()
{
	PrimaryActorTick.bCanEverTick = true;

	SetRootComponent(CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot")));
}

void APickPlaceTaskActor::BeginPlay()
{
	Super::BeginPlay();

	if (!Robot)
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[APickPlaceTaskActor] Robot이 지정되지 않아 사이클을 시작할 수 없습니다. 디테일 패널에서 Robot을 할당하세요."));
		return;
	}

	// 틱 순서 강제: ASerial6DoFRobotActor::Tick은 매 틱 JointAnglesDeg(에디터 프로퍼티)를 관절 상태에
	// 되쓴다. 이 액터가 로봇보다 먼저 틱하면 그 되쓰기에 궤적이 매 프레임 지워진다. prerequisite을 걸어
	// 로봇 → 태스크 순서를 보장하면, 로봇 클래스를 한 줄도 고치지 않고 되쓰기 위에 결과를 얹을 수 있다.
	AddTickPrerequisiteActor(Robot);

	if (Dispatcher)
	{
		// dispatcher 모드: 박스/슬롯 소유권이 dispatcher로 넘어간다. 스폰하지 않는다.
		// dispatcher가 먼저 틱해야 배급이 같은 프레임에 반영되므로 prerequisite을 하나 더 건다.
		Dispatcher->RegisterTaskActor(this);
		AddTickPrerequisiteActor(Dispatcher);
	}
	else if (bSpawnBoxesOnBeginPlay)
	{
		// standalone 폴백 — dispatcher 없이도 지금까지와 완전히 동일하게 동작한다.
		SpawnBoxes();
	}

	// StartCycle을 여기서 부르면 안 된다: 로봇의 BeginPlay가 아직 안 돌았을 수 있고, 그러면
	// VisualGraspPoint가 본에 붙기 전이라 GetVisualGraspPointWorld()가 수학 EE로 폴백해
	// 반복 보정이 통째로 무의미해진다(실제로 그렇게 실패했다). AddTickPrerequisiteActor는 Tick
	// 순서만 보장하지 BeginPlay 순서는 보장하지 않는다.
	//
	// 첫 Tick으로 미루면 모든 액터의 BeginPlay가 끝난 뒤라 안전하다.
	bPendingAutoStart = bAutoStartOnBeginPlay;
}

void APickPlaceTaskActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// 진행 중이던 배급을 반납한다 (박스 + 슬롯). PIE 종료 시엔 실질적 영향이 없지만,
	// 반납 이벤트가 Dispatch.csv에 남아야 "끝날 때 무엇이 진행 중이었나"를 읽을 수 있다.
	if (Dispatcher && AssignedTask.IsValid())
	{
		Dispatcher->ReturnTask(this, AssignedTask, /*bBlacklist=*/false, TEXT("EndPlay"));
		AssignedTask = FPickPlaceTask();
	}

	// 사이클이 Done에 도달하기 전에 PIE를 멈춰도 그때까지의 샘플은 남긴다.
	WriteCsvToDisk();

	Super::EndPlay(EndPlayReason);
}

void APickPlaceTaskActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// HUD 프레임타임. 원시값은 진동이 심해 읽을 수 없으므로 EMA로 평활한다 (표시용, 프로파일링용 아님).
	LastFrameTimeMs = FMath::Lerp(LastFrameTimeMs, static_cast<double>(DeltaSeconds) * 1000.0, FrameTimeSmoothingAlpha);

	// 자동 시작은 첫 Tick으로 미뤄져 있다 (BeginPlay 순서 문제 — BeginPlay의 주석 참조).
	// 이 시점엔 로봇 BeginPlay가 끝나 VisualGraspPoint가 본에 붙어 있다.
	if (bPendingAutoStart)
	{
		bPendingAutoStart = false;
		StartCycle();
	}

	if (bDrawDebugStatus && GEngine)
	{
		GEngine->AddOnScreenDebugMessage(
			static_cast<uint64>(GetUniqueID()), 0.0f,
			Phase == EPickPlacePhase::Aborted ? FColor::Red : FColor::Cyan,
			FString::Printf(TEXT("PickPlace: %s | 박스 %d/%d | t=%.2fs"),
				PhaseToString(Phase), CurrentBoxIndex + 1, Boxes.Num(), SimTimeSec));
	}

	// 모션 재생 (D-02) — FSM 위에 얹는 별도 모드다. 재생 중에는 FSM을 전진시키지 않고 저장된 관절
	// 궤적만 고정 스텝으로 되재생한 뒤, 아래 공통 경로(SetJointAngles)로 로봇에 민다. 여기서 return하지
	// 않고 SetJointAngles까지 흘러가야 로봇 Tick의 홈 자세 되쓰기를 이긴다.
	if (bReplayActive)
	{
		ReplayTimeAccumulatorSec += DeltaSeconds;

		int32 StepsThisFrame = 0;
		while (bReplayActive && ReplayTimeAccumulatorSec >= FixedTimeStepSec && StepsThisFrame < MaxFixedStepsPerFrame)
		{
			StepReplay(FixedTimeStepSec);
			ReplayTimeAccumulatorSec -= FixedTimeStepSec;
			++StepsThisFrame;
		}

		if (StepsThisFrame >= MaxFixedStepsPerFrame)
		{
			ReplayTimeAccumulatorSec = 0.0;
		}

		// 재생 중에도 관절은 로봇에 밀어야 한다. 박스는 재생 대상이 아니므로 UpdateHeldBoxTransform은
		// 부르지 않는다 (재생 중 HeldBox는 항상 nullptr다).
		if (Robot)
		{
			Robot->SetJointAngles(ActiveState);
		}
		return;
	}

	// standalone의 Idle은 "이 액터가 관절을 소유하지 않는다"는 뜻이므로 로봇에게 돌려준다
	// (STEP A 동작 그대로). 반면 **dispatcher 모드의 Idle은 "다음 배급 대기"** 라 자세를 유지해야 한다 —
	// 놓으면 작업 사이마다 팔이 홈 자세로 튕겨 데모가 경련한다.
	if (Phase == EPickPlacePhase::Idle && !Dispatcher)
	{
		return;
	}

	// Idle(dispatcher 대기)/Done/Aborted에서는 FSM을 전진시키지 않지만 SetJointAngles는 계속해야 한다.
	// 멈추면 로봇 Tick의 JointAnglesDeg 되쓰기가 다시 이겨서 팔이 홈 자세로 튕겨 돌아간다.
	//
	// bPaused도 같은 부류다 — 전진만 멈추고 관절 소유권은 놓지 않는다 (SetPaused 주석 참조).
	const bool bAdvancing =
		!bPaused &&
		Phase != EPickPlacePhase::Idle && Phase != EPickPlacePhase::Done && Phase != EPickPlacePhase::Aborted;

	if (bAdvancing)
	{
		// 고정 타임스텝 적분: 프레임 시간을 누적해 FixedTimeStepSec 단위로만 FSM을 전진시킨다.
		// 프레임레이트가 흔들려도 궤적 형상과 CSV 샘플 간격이 동일하게 나온다.
		TimeAccumulatorSec += DeltaSeconds;

		int32 StepsThisFrame = 0;
		while (TimeAccumulatorSec >= FixedTimeStepSec && StepsThisFrame < MaxFixedStepsPerFrame)
		{
			StepFixed(FixedTimeStepSec);
			TimeAccumulatorSec -= FixedTimeStepSec;
			++StepsThisFrame;

			if (Phase == EPickPlacePhase::Done || Phase == EPickPlacePhase::Aborted)
			{
				break;
			}
		}

		if (StepsThisFrame >= MaxFixedStepsPerFrame)
		{
			// 상한에 걸렸다 = 프레임이 심하게 밀렸다. 남은 빚을 버려 나선을 끊는다.
			TimeAccumulatorSec = 0.0;
		}
	}
	else
	{
		// 정지 중이어도 팔은 중력을 버티고 있다. 게이지를 0으로 두면 화면이 거짓말을 한다.
		UpdateHoldingTorqueCache();

		// **누적기를 전진시키지 않는다.** Pause 중 DeltaSeconds를 계속 더하면 재개 순간 밀린 빚이
		// 한꺼번에 터져 MaxFixedStepsPerFrame 클램프가 걸리고, 그건 곧 결정론 경고다.
		TimeAccumulatorSec = 0.0;
	}

	// 관절 상태를 로봇에 반영한다. 로봇의 되쓰기는 이 시점에 이미 끝났으므로(tick prerequisite)
	// 이 호출이 이번 프레임의 최종 자세가 된다.
	if (Robot)
	{
		Robot->SetJointAngles(ActiveState);
	}

	// 박스 추종은 최종 관절 자세가 확정된 뒤 한 번만 하면 된다 (중간 고정 스텝의 자세는 그려지지 않는다).
	UpdateHeldBoxTransform();
}

#pragma endregion

#pragma region APickPlaceTaskActor_Spawning

void APickPlaceTaskActor::SpawnBoxes()
{
	UWorld* World = GetWorld();
	if (!World || !Robot)
	{
		return;
	}

	// 스폰이 켜져 있으면 이 액터가 Boxes의 소유자다. 레벨에 수동 배치한 잔여물이 섞이지 않게 비운다.
	Boxes.Reset();

	// 출발지 상판 위에 슬롯을 잡는다. HeightAboveSurface=0 — 슬롯 Z가 곧 박스 **바닥**이 앉을 높이다.
	// (도착지는 툴 목표라 박스 높이만큼 띄운다. 같은 함수, 다른 인자 — 규약이 하나라 헷갈릴 일이 없다.)
	//
	// dispatcher도 이 함수들을 그대로 쓴다. 배치 규약을 한 곳에 두지 않으면 두 경로가 조용히
	// 갈라져 박스가 상판을 뚫거나 뜬다.
	TArray<FVector> SlotWorld;
	if (!FPickPlaceLayout::BuildSlotsOnSurface(SourceSurfaceActor, NumBoxesToSpawn, SourceSlotStrideCm,
			SourceSlotOffsetCm, /*HeightAboveSurfaceCm=*/0.0, SlotWorld))
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[APickPlaceTaskActor] SourceSurfaceActor가 없거나 바운드가 없어 박스를 스폰하지 못했습니다 — ")
			TEXT("팔레트 액터를 레벨에 배치하고 할당하세요."));
		return;
	}

	TArray<APickPlaceBoxActor*> SpawnedBoxes;
	FPickPlaceLayout::SpawnBoxesOnSlots(World, BoxClass ? BoxClass.Get() : APickPlaceBoxActor::StaticClass(),
		SlotWorld, SourceSurfaceActor->GetActorRotation(), this, SpawnedBoxes);

	for (APickPlaceBoxActor* Box : SpawnedBoxes)
	{
		Boxes.Add(Box);
	}

	// 간격/표면 크기 검사는 FPickPlaceLayout이 한다 (dispatcher 경로도 같은 검사를 받아야 하므로).
	UE_LOG(LogRobotSim, Log,
		TEXT("[APickPlaceTaskActor] 출발지 '%s' 상판에 박스 %d개 스폰 — 간격 (%.0f, %.0f, %.0f)cm"),
		*SourceSurfaceActor->GetName(), Boxes.Num(),
		SourceSlotStrideCm.X, SourceSlotStrideCm.Y, SourceSlotStrideCm.Z);
}

void APickPlaceTaskActor::LogReachableRadiusBand(const FVector& DirectionXY, double ZLocal, const TCHAR* Label)
{
	if (!Robot)
	{
		return;
	}

	const FVector Dir = FVector(DirectionXY.X, DirectionXY.Y, 0.0).GetSafeNormal();
	if (Dir.IsNearlyZero())
	{
		return;
	}

	// 탐색 상한은 홈 자세 EE 반경의 3배로 잡는다. 시각 그리퍼는 수학 EE보다 훨씬 멀리 가므로
	// (메시가 수학 팔의 약 2배) 수학 반경 기준으로는 여유를 크게 둬야 최대 도달점을 놓치지 않는다.
	const FVector HomeEELocal = Robot->GetModel().ComputeEndEffectorTransform(FRobot6DJointState()).GetLocation();
	const double MaxProbeCm = FMath::Max(HomeEELocal.Size() * 3.0, 100.0);

	// 20cm 간격: 반경 하나당 SolveForVisualGraspPoint가 IK를 최대 12번 + 메시 동기화를 그만큼 돌리므로
	// 10cm 간격이면 탐색 3회에 1400회가 넘어 PIE 시작이 눈에 띄게 멈춘다. 배치 튜닝용 진단이라
	// 20cm 해상도면 충분하다.
	constexpr double ProbeStepCm = 20.0;

	// **시각 그리퍼** 기준으로 훑는다 — 박스가 붙는 곳이 거기이고, 사이클의 도달 판정도 그 기준이기
	// 때문이다. 수학 EE의 도달 구간을 찍으면 실제 동작과 다른 숫자를 보고하게 된다: 메시가 수학 팔의
	// 약 2배라 시각 그리퍼는 수학 EE가 못 가는 먼 곳까지 간다.
	//
	// 반경은 **XY 평면 반경**이다 (3D 거리가 아니다). 중단 메시지도 같은 양을 찍어야 비교가 성립한다.
	double FirstReachable = -1.0;
	double LastReachable = -1.0;
	double WorstErrorInBandCm = 0.0;

	for (double R = ProbeStepCm; R <= MaxProbeCm; R += ProbeStepCm)
	{
		const FTransform Target(GraspRotation.Quaternion(), Dir * R + FVector(0.0, 0.0, ZLocal));

		FRobot6DJointState ProbeSolution;
		double ProbeErrorCm = 0.0;
		if (SolveForVisualGraspPoint(Target, ProbeSolution, ProbeErrorCm))
		{
			if (FirstReachable < 0.0)
			{
				FirstReachable = R;
			}
			LastReachable = R;
			WorstErrorInBandCm = FMath::Max(WorstErrorInBandCm, ProbeErrorCm);
		}
	}

	UE_LOG(LogRobotSim, Log,
		TEXT("[APickPlaceTaskActor] 시각 그리퍼 도달 XY반경 탐색(%s) — 방향 (%.2f, %.2f), 높이 z=%.1fcm[로봇 공간], %.0fcm 간격, 감쇠 α=%.2f:"),
		Label, Dir.X, Dir.Y, ZLocal, ProbeStepCm, VisualSolveDamping);

	if (FirstReachable < 0.0)
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[APickPlaceTaskActor]   **어떤 XY반경에서도 시각 그리퍼를 보낼 수 없습니다** (탐색 상한 %.0fcm). ")
			TEXT("높이 z가 작업 영역 밖이거나(지지면 트레이스가 엉뚱한 것을 짚었는지 확인), ")
			TEXT("반복 보정이 발산하는 중입니다(VisualSolveDamping을 낮춰 보세요 — 현재 %.2f)."),
			MaxProbeCm, VisualSolveDamping);
		return;
	}

	UE_LOG(LogRobotSim, Log,
		TEXT("[APickPlaceTaskActor]   **XY반경 %.0f ~ %.0fcm** 에서 시각 그리퍼 도달 가능 (구간 내 최대 잔차 %.2fcm). ")
		TEXT("배치 목표의 XY반경이 이 구간 안에 들어와야 합니다."),
		FirstReachable, LastReachable, WorstErrorInBandCm);
}

#pragma region FPickPlaceLayout

bool FPickPlaceLayout::BuildSlotsOnSurface(
	const AActor* SurfaceActor, int32 SlotCount, const FVector& StrideCm, const FVector& OffsetCm,
	double HeightAboveSurfaceCm, TArray<FVector>& OutSlotWorld)
{
	OutSlotWorld.Reset();

	if (!SurfaceActor || SlotCount <= 0)
	{
		return false;
	}

	// 상판 높이는 **바운드**에서 읽는다 — 메시 피벗이 바닥/중심/구석 어디에 있든 무관해진다
	// (박스 파지점에서 쓴 것과 같은 이유이자 같은 방법이다).
	const FBox SurfaceBounds = SurfaceActor->GetComponentsBoundingBox(/*bNonColliding=*/true);
	if (!SurfaceBounds.IsValid)
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[APickPlaceTaskActor] '%s'의 바운드가 유효하지 않습니다 (메시가 없나요?) — 슬롯을 만들 수 없습니다."),
			*SurfaceActor->GetName());
		return false;
	}

	const double SurfaceTopWorldZ = SurfaceBounds.Max.Z;
	const FTransform SurfaceToWorld = SurfaceActor->GetActorTransform();

	// 행의 기준은 액터 피벗이 아니라 **바운드 중심**이다. 프롭 메시는 피벗이 한쪽 끝이나 구석에 있는
	// 경우가 흔해서(실제로 BP_Pallet은 피벗이 바운드 끝에 있어 행의 절반이 상판 밖으로 나갔다)
	// 피벗을 기준으로 잡으면 행이 표면에서 벗어난다. 박스 파지점·상판 높이를 바운드로 계산해
	// 피벗 독립으로 만들어 둔 것과 같은 이유이자 같은 방법이다.
	const FVector RowCenterWorld = SurfaceBounds.GetCenter();

	// 간격/오프셋은 액터 로컬 공간의 **방향**이므로 위치가 아니라 벡터로 변환한다 —
	// 액터를 회전시키면 행도 같이 돌되, 피벗 위치에는 영향받지 않는다.
	const FVector StrideWorld = SurfaceToWorld.TransformVector(StrideCm);
	const FVector OffsetWorld = SurfaceToWorld.TransformVector(OffsetCm);

	for (int32 i = 0; i < SlotCount; ++i)
	{
		// 행을 중심에 맞춰 정렬한다: i - (N-1)/2 → 짝수 개면 중심 양옆, 홀수 개면 가운데가 중심.
		// 이렇게 하면 박스 개수를 바꿔도 액터를 다시 옮길 필요가 없다.
		const double Centered = static_cast<double>(i) - (static_cast<double>(SlotCount) - 1.0) * 0.5;

		FVector SlotWorld = RowCenterWorld + OffsetWorld + StrideWorld * Centered;

		// XY는 위에서 정해지고 Z만 상판 기준으로 덮어쓴다 — 액터가 조금 기울어져 있어도
		// 박스는 중력 기준 수평으로 놓여야 하기 때문이다.
		SlotWorld.Z = SurfaceTopWorldZ + HeightAboveSurfaceCm;

		OutSlotWorld.Add(SlotWorld);
	}

	// 슬롯이 상판 밖으로 나가면 그 자리 박스는 **허공에 상판 높이로** 스폰돼 떨어진다 (Z는 상판 기준으로
	// 일괄 계산하므로 XY가 표면을 벗어나도 높이만 맞는다). 떨어지면서 서로 부딪혀 흩어지고, 낙하 중에
	// 파지 지점이 스냅샷되면 엉뚱한 곳을 집는다 — 겉보기엔 "박스가 이상하게 흩어진다"로만 보인다.
	int32 OutsideCount = 0;
	for (const FVector& Slot : OutSlotWorld)
	{
		if (!SurfaceBounds.IsInsideXY(Slot))
		{
			++OutsideCount;
		}
	}

	if (OutsideCount > 0)
	{
		const FVector SurfaceSize = SurfaceBounds.GetSize();
		const double RowLengthCm = StrideCm.Size() * static_cast<double>(SlotCount - 1);

		UE_LOG(LogRobotSim, Warning,
			TEXT("[FPickPlaceLayout] '%s' 상판 밖으로 나간 슬롯 %d/%d개 — 그 자리 박스는 허공에서 떨어져 흩어집니다. ")
			TEXT("슬롯 행 길이 %.0fcm (간격 %.0f × %d칸) vs 상판 크기 (%.0f × %.0f)cm. ")
			TEXT("→ 개수(SlotCount %d)를 줄이거나, 간격을 줄이거나, 더 큰 표면을 쓰세요."),
			*SurfaceActor->GetName(), OutsideCount, SlotCount, RowLengthCm,
			StrideCm.Size(), SlotCount - 1, SurfaceSize.X, SurfaceSize.Y, SlotCount);
	}

	return true;
}

void FPickPlaceLayout::SpawnBoxesOnSlots(
	UWorld* World, UClass* BoxClass, const TArray<FVector>& SlotWorld, const FRotator& SpawnRotation,
	AActor* Owner, TArray<APickPlaceBoxActor*>& OutBoxes)
{
	OutBoxes.Reset();

	if (!World || !BoxClass)
	{
		return;
	}

	for (int32 i = 0; i < SlotWorld.Num(); ++i)
	{
		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		SpawnParams.Owner = Owner;

		// 먼저 슬롯 위치에 스폰한다. 실제 바운드는 스폰 후에만 알 수 있고, 그 값이 있어야
		// "상판 위에 정확히" 앉힐 수 있다. 회전은 출발지 액터를 따라간다(상판 위에 정렬돼 보이도록).
		APickPlaceBoxActor* Box =
			World->SpawnActor<APickPlaceBoxActor>(BoxClass, FTransform(SpawnRotation, SlotWorld[i]), SpawnParams);

		if (!Box)
		{
			UE_LOG(LogRobotSim, Warning, TEXT("[FPickPlaceLayout] 박스 %d 스폰 실패 (BoxClass 확인)"), i);
			continue;
		}

		// 박스 바닥이 상판에 정확히 닿게 앉힌다. 뜬 채로 스폰되면 낙하 중에 파지 지점이 스냅샷돼
		// 엉뚱한 곳을 집게 되고, 옆 박스를 밀어내기도 한다.
		//
		// 액터 위치가 아니라 **바운드 바닥면**을 기준으로 보정한다. 프롭 메시는 피벗이 바닥이나
		// 구석에 있는 경우가 흔해서 "액터 위치 = 기하학적 중심"이 성립하지 않는다.
		const double DropZ = SlotWorld[i].Z - Box->GetBoundsBottomZWorld();
		Box->AddActorWorldOffset(FVector(0.0, 0.0, DropZ),
			/*bSweep=*/false, /*OutSweepHitResult=*/nullptr, ETeleportType::TeleportPhysics);

		OutBoxes.Add(Box);

		UE_LOG(LogRobotSim, Log,
			TEXT("[FPickPlaceLayout] 박스 %d 스폰 — 월드 (%.1f, %.1f, %.1f)cm, 높이 %.1fcm, 파지점(윗면 중심) (%.1f, %.1f, %.1f)cm"),
			i, Box->GetActorLocation().X, Box->GetActorLocation().Y, Box->GetActorLocation().Z, Box->GetHeightCm(),
			Box->GetGraspPointWorld().X, Box->GetGraspPointWorld().Y, Box->GetGraspPointWorld().Z);
	}

	// 슬롯 간격이 박스 크기보다 좁으면 스폰 순간 서로 겹쳐 물리가 밀어낸다. 그러면 박스가 파지 지점
	// 스냅샷에서 벗어나 흡착판이 빈 곳을 집는데, 겉보기엔 "박스가 흩어진다 / IK가 이상하다"로 보여
	// 원인을 찾기 어렵다. 조용히 틀리는 종류라 실측값으로 미리 말한다.
	//
	// 간격은 stride 파라미터가 아니라 **실제 슬롯 사이 거리**로 잰다 — 이 함수는 슬롯을 어떻게 만들었는지
	// 모르고, 알 필요도 없다.
	if (OutBoxes.Num() >= 2 && OutBoxes[0])
	{
		const double GapCm = FVector::Dist(SlotWorld[0], SlotWorld[1]);
		const FVector Extent = OutBoxes[0]->GetBoundsExtentCm();
		const FVector GapDir = (SlotWorld[1] - SlotWorld[0]).GetSafeNormal();

		// 바운드는 월드 AABB이므로, 간격 방향으로 박스가 차지하는 폭은 성분 절대값 합으로 근사한다.
		const double BoxWidthAlongGap = 2.0 * (FMath::Abs(Extent.X * GapDir.X)
			+ FMath::Abs(Extent.Y * GapDir.Y) + FMath::Abs(Extent.Z * GapDir.Z));

		if (GapCm < BoxWidthAlongGap)
		{
			UE_LOG(LogRobotSim, Warning,
				TEXT("[FPickPlaceLayout] **슬롯 간격이 박스보다 좁습니다** — 간격 %.1fcm < 박스 폭 %.1fcm. ")
				TEXT("스폰 순간 박스들이 겹쳐 물리가 서로 밀어내고, 그러면 파지 지점 스냅샷이 어긋나 ")
				TEXT("흡착판이 빈 곳을 집습니다. 간격을 **%.0fcm 이상**으로 키우세요."),
				GapCm, BoxWidthAlongGap, FMath::CeilToDouble(BoxWidthAlongGap * 1.1));
		}
	}
}

#pragma endregion

void APickPlaceTaskActor::BuildPalletSlots(double BoxHeightCm)
{
	PalletSlotLocations.Reset();

	if (!Robot)
	{
		return;
	}

	// 툴 목표 높이 = 상판 + 박스 **전체** 높이 + 여유. 파지점이 박스 윗면이라 박스는 툴에서 아래로
	// 전체 높이만큼 매달려 있으므로, 툴이 이 높이면 박스 바닥이 상판에 닿는다.
	// (파지점을 중심으로 잡던 시절엔 절반 높이였다 — 파지 규약과 반드시 함께 바뀌어야 하는 값이다.)
	TArray<FVector> SlotWorld;
	if (!FPickPlaceLayout::BuildSlotsOnSurface(DestinationSurfaceActor, Boxes.Num(), DestinationSlotStrideCm,
			DestinationSlotOffsetCm, BoxHeightCm + PlaceClearanceCm, SlotWorld))
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[APickPlaceTaskActor] DestinationSurfaceActor가 없거나 바운드가 없어 도착지 슬롯을 만들지 못했습니다 — ")
			TEXT("레일/컨베이어 액터를 레벨에 배치하고 할당하세요."));
		return;
	}

	const FTransform RobotToWorld = Robot->GetActorTransform();
	for (int32 i = 0; i < SlotWorld.Num(); ++i)
	{
		// IK는 로봇 공간에서 푼다.
		const FVector SlotLocal = RobotToWorld.InverseTransformPosition(SlotWorld[i]);
		PalletSlotLocations.Add(SlotLocal);

		UE_LOG(LogRobotSim, Log,
			TEXT("[APickPlaceTaskActor] 도착지 슬롯 %d — 월드 (%.1f, %.1f, %.1f)cm / 로봇 공간 (%.1f, %.1f, %.1f)cm [XY반경 %.1fcm]"),
			i, SlotWorld[i].X, SlotWorld[i].Y, SlotWorld[i].Z, SlotLocal.X, SlotLocal.Y, SlotLocal.Z,
			FVector2D(SlotLocal.X, SlotLocal.Y).Size());
	}

	UE_LOG(LogRobotSim, Log,
		TEXT("[APickPlaceTaskActor] 도착지 '%s' 기준 슬롯 %d개 — 박스 높이 %.1fcm, 간격 (%.0f, %.0f, %.0f)cm"),
		*DestinationSurfaceActor->GetName(), SlotWorld.Num(), BoxHeightCm,
		DestinationSlotStrideCm.X, DestinationSlotStrideCm.Y, DestinationSlotStrideCm.Z);
}

#pragma endregion

#pragma region APickPlaceTaskActor_PublicAPI

void APickPlaceTaskActor::StartCycle()
{
	if (!Robot)
	{
		UE_LOG(LogRobotSim, Warning, TEXT("[APickPlaceTaskActor] Robot이 없어 StartCycle을 무시합니다."));
		return;
	}

	// 진행 중이던 파지가 있으면 정리하고 처음부터 다시 시작한다 (데모 중 재실행 편의).
	if (HeldBox)
	{
		HeldBox->EndGrasp();
		HeldBox = nullptr;
	}

	// 로봇의 현재 자세를 시작점으로 삼는다. 이후로는 ActiveState가 관절 상태의 source of truth다.
	ActiveState = Robot->GetJointState();
	PreviousState = ActiveState;

	SimTimeSec = 0.0;
	TimeAccumulatorSec = 0.0;

	// 텔레메트리 캐시도 처음부터다. 남겨두면 이전 사이클의 토크/오차가 UI에 잠깐 유령처럼 남는다.
	// bPaused는 건드리지 않는다 — 일시정지는 사용자의 UI 의도이지 사이클 상태가 아니다.
	LastJointVelocityRadPerSec = FRobot6DJointVelocity();
	LastJointTorqueNm = FRobot6DJointTorque();
	LastReachErrorCm = 0.0;
	CompletedBoxCount = 0;

	// 다른 로봇의 태스크 액터와 파일명이 겹치는지 먼저 확정한다 (겹치면 조용히 덮어쓴다).
	ResolveCsvFileName();

	CsvRows.Reset();
	if (bEnableCsvLogging)
	{
		// 여기 있는 값은 전부 ActiveState(이 액터가 소유한 관절 상태)에서 직접 계산한다 — 로봇/메시
		// 컴포넌트를 읽지 않는다. 그래야 프레임 중 어느 시점에 기록하든 값이 일관된다.
		//
		// (한때 흡착판/박스 월드 좌표도 기록했는데, 그건 로봇 컴포넌트를 읽는 값이라 로봇 Tick이
		//  JointAnglesDeg로 되쓴 홈 자세를 찍었다. 화면은 멀쩡한데 CSV만 틀리는 함정이라 걷어냈다.)
		//
		// tau 컬럼은 **끝에 붙인다** — 중간에 끼우면 기존 컬럼 인덱스가 밀려서 이미 만든 분석
		// 스크립트/차트가 조용히 다른 열을 읽는다.
		CsvRows.Add(TEXT("time_s,phase,box_index,")
			TEXT("q0_deg,q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,")
			TEXT("qd0_degps,qd1_degps,qd2_degps,qd3_degps,qd4_degps,qd5_degps,")
			TEXT("ee_x_cm,ee_y_cm,ee_z_cm,ee_roll_deg,ee_pitch_deg,ee_yaw_deg,")
			TEXT("target_x_cm,target_y_cm,target_z_cm,box_held,")
			TEXT("tau0_nm,tau1_nm,tau2_nm,tau3_nm,tau4_nm,tau5_nm"));
	}

	// 누적 모드면 기존 파일의 과거 행을 읽어둔다 (헤더 확정 뒤 — 헤더 일치 검사에 CsvRows[0]이 필요하다).
	// 끄면 AccumulatedPriorRows는 비어 있어 기존 동작(덮어쓰기)과 동일하다.
	AccumulatedPriorRows.Reset();
	if (bEnableCsvLogging && bAccumulateMotionAcrossRuns)
	{
		LoadPriorRowsForAccumulation();
	}

	// dispatcher 모드: 박스도 슬롯도 dispatcher가 소유한다. 여기서는 배급을 기다리기만 한다.
	// (bCycleStarted가 서야 dispatcher가 IsIdle()을 true로 보고 배급을 시작한다.)
	if (Dispatcher)
	{
		bCycleStarted = true;
		EnterPhase(EPickPlacePhase::Idle);

		UE_LOG(LogRobotSim, Log,
			TEXT("[APickPlaceTaskActor] dispatcher 모드 — '%s'의 배급을 기다립니다 (우선순위 %d). ")
			TEXT("박스/슬롯은 dispatcher가 소유합니다."),
			*Dispatcher->GetName(), RobotPriority);
		return;
	}

	const int32 FirstBox = FindNextValidBoxIndex(0);
	if (FirstBox == INDEX_NONE)
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[APickPlaceTaskActor] 처리할 박스가 없습니다 (bSpawnBoxesOnBeginPlay를 켜거나 Boxes 배열을 채우세요)."));
		EnterPhase(EPickPlacePhase::Done);
		return;
	}

	// 팔레트 슬롯 높이는 박스 크기에 의존하므로 박스가 확정된 뒤에 계산한다.
	BuildPalletSlots(Boxes[FirstBox]->GetHeightCm());

	bCycleStarted = true;
	CurrentBoxIndex = FirstBox;

	// 첫 IK를 돌리기 전에 실제 도달 가능 구간을 찍는다. 실패하더라도 Aborted 로그의 XY반경을
	// 이 구간과 대조하면 얼마나 어느 쪽으로 옮겨야 하는지 바로 나온다 — 추측할 필요가 없다.
	if (bLogWorkspaceProbe)
	{
		// 실제 목표 높이에서 훑어야 의미가 있다 — 박스 액터 위치가 아니라 **파지점(윗면)** 높이가
		// 툴이 가는 곳이고, 도달 가능 반경은 높이에 크게 의존한다(팔이 위로 뻗을수록 반경이 준다).
		const FVector FirstGraspLocal =
			Robot->GetActorTransform().InverseTransformPosition(Boxes[FirstBox]->GetGraspPointWorld());
		LogReachableRadiusBand(FirstGraspLocal, FirstGraspLocal.Z, TEXT("박스 파지 높이"));

		// 접근 자세는 파지점보다 ApproachOffsetCm만큼 위라 더 빡빡하다 — 실패는 보통 여기서 먼저 난다.
		const FVector ApproachLocal = FirstGraspLocal + ApproachOffsetCm;
		LogReachableRadiusBand(ApproachLocal, ApproachLocal.Z, TEXT("박스 접근 높이"));

		if (PalletSlotLocations.IsValidIndex(0))
		{
			const FVector FirstSlotLocal = PalletSlotLocations[0];
			LogReachableRadiusBand(FirstSlotLocal, FirstSlotLocal.Z, TEXT("도착지 높이"));
		}
	}

	UE_LOG(LogRobotSim, Log,
		TEXT("[APickPlaceTaskActor] 사이클 시작 — 박스 %d개, 고정 타임스텝 %.4fs (%.0fHz), 속도 스케일 %.2f"),
		Boxes.Num(), FixedTimeStepSec, 1.0 / FixedTimeStepSec, VelocityScale);

	// 파지 기준을 명시적으로 찍는다. 이게 조용히 폴백되면 반복 보정이 무의미해지는데도 겉으로는
	// 그냥 "도달 실패"로만 보여서 원인 파악이 불가능하다 — 실제로 그 침묵 때문에 헛짚었다.
	if (Robot->IsVisualGraspPointAttached())
	{
		UE_LOG(LogRobotSim, Log,
			TEXT("[APickPlaceTaskActor] 파지 기준 = **시각 파지점** (그리퍼 본에 부착됨). 박스는 보이는 흡착판에 붙습니다."));
	}
	else
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[APickPlaceTaskActor] 파지 기준 = **수학 EE로 폴백** — VisualGraspPoint가 그리퍼 본에 붙지 않았습니다. ")
			TEXT("SkeletalMesh를 안 쓰는 구성이면 이게 정상입니다. KUKA 메시를 쓰는데 이 경고가 보인다면 ")
			TEXT("RobotConfig의 VisualGraspBoneName을 확인하세요 — 이 상태에서는 반복 보정이 무의미하고 ")
			TEXT("박스가 보이는 그리퍼가 아니라 수학 EE에 붙습니다."));
	}

	// 이 로봇의 실제 기구학을 통째로 찍는다. RobotConfig(DataAsset, 바이너리)가 기구학을 소유하므로
	// 소스만 읽어서는 규모도 툴 축 방향도 알 수 없다 — 그래서 CreateDefault(105cm급)를 기준으로
	// 배치값을 추측했다가 두 번 틀렸다. 추측 대신 모델에게 직접 묻는다.
	{
		const FSerial6DoFModel& Model = Robot->GetModel();
		const FTransform HomeEE = Model.ComputeEndEffectorTransform(FRobot6DJointState());
		const FRotator HomeRot = HomeEE.GetRotation().Rotator();
		const FVector HomeLoc = HomeEE.GetLocation();

		UE_LOG(LogRobotSim, Log,
			TEXT("[APickPlaceTaskActor] [기구학] 홈 자세(Q=0) EE = 위치 (%.1f, %.1f, %.1f)cm [베이스에서 %.1fcm], ")
			TEXT("회전 (Pitch=%.1f, Yaw=%.1f, Roll=%.1f)도"),
			HomeLoc.X, HomeLoc.Y, HomeLoc.Z, HomeLoc.Size(), HomeRot.Pitch, HomeRot.Yaw, HomeRot.Roll);

		// 툴 프레임의 각 축이 홈 자세에서 어디를 향하는지. "그리퍼가 어느 축으로 뻗어 있는가"를
		// 알아야 GraspRotation을 계산할 수 있다 (기본 모델은 +X가 접근 방향이지만 config마다 다르다).
		const FQuat HomeQ = HomeEE.GetRotation();
		UE_LOG(LogRobotSim, Log,
			TEXT("[APickPlaceTaskActor] [기구학] 홈 자세 툴 축 방향 — +X=(%.2f, %.2f, %.2f), +Y=(%.2f, %.2f, %.2f), +Z=(%.2f, %.2f, %.2f). ")
			TEXT("이 중 아래(0,0,-1)를 향하는 축이 접근 축이면 GraspRotation은 홈 회전 그대로면 된다."),
			HomeQ.GetAxisX().X, HomeQ.GetAxisX().Y, HomeQ.GetAxisX().Z,
			HomeQ.GetAxisY().X, HomeQ.GetAxisY().Y, HomeQ.GetAxisY().Z,
			HomeQ.GetAxisZ().X, HomeQ.GetAxisZ().Y, HomeQ.GetAxisZ().Z);

		const FRotator ToolRot = Model.ToolOffset.GetRotation().Rotator();
		UE_LOG(LogRobotSim, Log,
			TEXT("[APickPlaceTaskActor] [기구학] ToolOffset = 위치 (%.1f, %.1f, %.1f)cm, 회전 (Pitch=%.1f, Yaw=%.1f, Roll=%.1f)도"),
			Model.ToolOffset.GetLocation().X, Model.ToolOffset.GetLocation().Y, Model.ToolOffset.GetLocation().Z,
			ToolRot.Pitch, ToolRot.Yaw, ToolRot.Roll);

		for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
		{
			UE_LOG(LogRobotSim, Log,
				TEXT("[APickPlaceTaskActor] [기구학] J%d — 축 (%.2f, %.2f, %.2f), 오프셋 (%.1f, %.1f, %.1f)cm, 한계 %.0f~%.0f도"),
				i, Model.JointAxes[i].X, Model.JointAxes[i].Y, Model.JointAxes[i].Z,
				Model.LinkOffsets[i].X, Model.LinkOffsets[i].Y, Model.LinkOffsets[i].Z,
				FMath::RadiansToDegrees(Model.JointLimits[i].MinRad),
				FMath::RadiansToDegrees(Model.JointLimits[i].MaxRad));
		}
	}

	EnterPhase(EPickPlacePhase::ToPickApproach);
}

void APickPlaceTaskActor::ResetCycle()
{
	if (HeldBox)
	{
		HeldBox->EndGrasp();
		HeldBox = nullptr;
	}

	// 배급받은 작업을 반납하지 않으면 그 박스와 슬롯이 영구 점유로 새어, 남은 작업이 배급되지 않는다.
	// 블랙리스트는 걸지 않는다 — 리셋은 이 로봇이 그 박스에 실패했다는 뜻이 아니다.
	if (Dispatcher && AssignedTask.IsValid())
	{
		Dispatcher->ReturnTask(this, AssignedTask, /*bBlacklist=*/false, TEXT("ResetCycle"));
	}
	AssignedTask = FPickPlaceTask();
	bCycleStarted = false;

	// 재생 중 리셋하면 재생도 멈춘다 (파싱한 프레임은 남겨 다시 PlayReplay로 재생할 수 있게 한다).
	bReplayActive = false;
	ReplayTimeAccumulatorSec = 0.0;

	Phase = EPickPlacePhase::Idle;
	CurrentBoxIndex = INDEX_NONE;
	PhaseElapsedSec = 0.0;
	PhaseDurationSec = 0.0;
	SimTimeSec = 0.0;
	TimeAccumulatorSec = 0.0;

	LastJointVelocityRadPerSec = FRobot6DJointVelocity();
	LastJointTorqueNm = FRobot6DJointTorque();
	LastReachErrorCm = 0.0;
	CompletedBoxCount = 0;

	UE_LOG(LogRobotSim, Log,
		TEXT("[APickPlaceTaskActor] 사이클을 Idle로 리셋했습니다 — 관절 소유권을 로봇에 반환하므로 팔이 JointAnglesDeg 자세로 돌아갑니다."));
}

void APickPlaceTaskActor::FlushCsvNow()
{
	WriteCsvToDisk();
}

#pragma region Replay

void APickPlaceTaskActor::LoadMotionCsv()
{
	ReplayFrames.Reset();

	// 재생 파일명: 지정이 없으면 이 액터가 기록하는 CsvFileName을 쓴다 (방금 자기 궤적을 재생하는 흔한 경우).
	const FString FileName = ReplayMotionFileName.IsEmpty() ? CsvFileName : ReplayMotionFileName;
	if (FileName.IsEmpty())
	{
		UE_LOG(LogRobotSim, Warning, TEXT("[PickPlaceTask] 재생 파일명이 비어 있습니다 (ReplayMotionFileName/CsvFileName 확인)."));
		return;
	}

	const FString FilePath = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("RobotSim"), FileName);

	TArray<FString> Lines;
	if (!FFileHelper::LoadFileToStringArray(Lines, *FilePath) || Lines.Num() < 2)
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[PickPlaceTask] 모션 CSV를 읽지 못했습니다: %s (파일 없음 또는 데이터 행 0개). 먼저 사이클을 한 번 돌려 CSV를 생성하세요."),
			*FilePath);
		return;
	}

	// 컬럼은 **이름으로 찾는다** — 고정 인덱스는 컬럼 순서가 한 번만 바뀌어도 조용히 틀린 열을 읽는다.
	// (기록부는 tau를 끝에 붙인다는 규약이 있어 중간 삽입 여지가 있다.) 기록↔재생 대칭의 계약은 이름이다.
	TArray<FString> Header;
	Lines[0].ParseIntoArray(Header, TEXT(","), /*InCullEmpty=*/false);

	int32 QColumn[FSerial6DoFModel::NumJoints];
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		// 헤더에 BOM/공백이 섞일 수 있으므로 trim 후 정확히 일치하는 이름을 찾는다.
		const FString Want = FString::Printf(TEXT("q%d_deg"), i);
		QColumn[i] = INDEX_NONE;
		for (int32 c = 0; c < Header.Num(); ++c)
		{
			if (Header[c].TrimStartAndEnd() == Want)
			{
				QColumn[i] = c;
				break;
			}
		}

		if (QColumn[i] == INDEX_NONE)
		{
			UE_LOG(LogRobotSim, Warning,
				TEXT("[PickPlaceTask] 헤더에서 컬럼 '%s'을(를) 찾지 못했습니다 — 이 파일은 재생할 수 없습니다 (헤더 불일치). 파일: %s"),
				*Want, *FilePath);
			ReplayFrames.Reset();
			return;
		}
	}

	// 데이터 행 파싱. 숫자 파싱 실패 행은 스킵하되 경고는 한 번만 낸다 (스팸 금지).
	const int32 MaxColumnNeeded = FMath::Max<int32>({ QColumn[0], QColumn[1], QColumn[2], QColumn[3], QColumn[4], QColumn[5] });
	bool bWarnedBadRow = false;

	for (int32 r = 1; r < Lines.Num(); ++r)
	{
		TArray<FString> Cols;
		Lines[r].ParseIntoArray(Cols, TEXT(","), /*InCullEmpty=*/false);
		if (Cols.Num() <= MaxColumnNeeded)
		{
			if (!bWarnedBadRow)
			{
				UE_LOG(LogRobotSim, Warning, TEXT("[PickPlaceTask] 컬럼 수가 부족한 행을 건너뜁니다 (행 %d 등). 이후 유사 행은 조용히 스킵합니다."), r);
				bWarnedBadRow = true;
			}
			continue;
		}

		FRobot6DJointState Frame;
		bool bRowOk = true;
		for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
		{
			const FString Cell = Cols[QColumn[i]].TrimStartAndEnd();
			if (!Cell.IsNumeric())
			{
				bRowOk = false;
				break;
			}
			// **degree → radian 역변환.** 기록은 RadiansToDegrees로 저장하므로 정확히 역으로 되돌린다.
			// 빠뜨리면 팔이 57배 꺾인다.
			Frame.Q[i] = FMath::DegreesToRadians(FCString::Atod(*Cell));
		}

		if (!bRowOk)
		{
			if (!bWarnedBadRow)
			{
				UE_LOG(LogRobotSim, Warning, TEXT("[PickPlaceTask] 숫자로 파싱되지 않는 행을 건너뜁니다 (행 %d 등). 이후 유사 행은 조용히 스킵합니다."), r);
				bWarnedBadRow = true;
			}
			continue;
		}

		ReplayFrames.Add(Frame);
	}

	if (ReplayFrames.Num() == 0)
	{
		UE_LOG(LogRobotSim, Warning, TEXT("[PickPlaceTask] 파싱된 재생 프레임이 0개입니다: %s"), *FilePath);
		return;
	}

	UE_LOG(LogRobotSim, Log,
		TEXT("[PickPlaceTask] 모션 CSV 로드 완료 — %d 프레임 (%s). 고정 스텝 %.4fs로 재생하면 약 %.2fs 길이."),
		ReplayFrames.Num(), *FileName, FixedTimeStepSec, ReplayFrames.Num() * FixedTimeStepSec);
}

TArray<FString> APickPlaceTaskActor::GetAvailableMotionCsvFiles() const
{
	TArray<FString> Files;

	const FString Directory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("RobotSim"));
	IFileManager::Get().FindFiles(Files, *FPaths::Combine(Directory, TEXT("*.csv")), /*Files=*/true, /*Directories=*/false);

	// FindFiles는 파일명만 반환한다(경로 없음). 그대로 드롭다운에 쓰고, LoadMotionCsv가 폴더를 붙인다.
	Files.Sort();
	return Files;
}

void APickPlaceTaskActor::SetReplayMotionFileName(const FString& InFileName)
{
	if (ReplayMotionFileName == InFileName)
	{
		return;
	}

	ReplayMotionFileName = InFileName;

	// 파일이 바뀌었으면 로드해 둔 프레임을 버린다 — 안 그러면 다음 PlayReplay가 (비어 있지 않으므로)
	// 자동 로드를 건너뛰고 **이전 파일**을 재생한다. 조용히 틀린 파일을 트는 함정이다.
	ReplayFrames.Reset();

	UE_LOG(LogRobotSim, Log, TEXT("[PickPlaceTask] 재생 파일 지정 — '%s' (다음 PlayReplay에서 로드)"), *InFileName);
}

void APickPlaceTaskActor::PlayReplay()
{
	if (!Robot)
	{
		UE_LOG(LogRobotSim, Warning, TEXT("[PickPlaceTask] Robot이 없어 재생할 수 없습니다."));
		return;
	}

	// 편의: 아직 로드 안 했으면 한 번 자동 로드한다 (로드/재생 분리는 유지하되 버튼 한 번으로 되게).
	if (ReplayFrames.Num() == 0)
	{
		LoadMotionCsv();
	}
	if (ReplayFrames.Num() == 0)
	{
		return; // LoadMotionCsv가 이미 이유를 로그로 남겼다
	}

	// 진행 중이던 사이클/파지를 정리한다 — 재생과 FSM이 같은 관절을 두고 다투면 안 된다.
	if (HeldBox)
	{
		HeldBox->EndGrasp();
		HeldBox = nullptr;
	}
	if (Dispatcher && AssignedTask.IsValid())
	{
		Dispatcher->ReturnTask(this, AssignedTask, /*bBlacklist=*/false, TEXT("PlayReplay"));
	}
	AssignedTask = FPickPlaceTask();

	Phase = EPickPlacePhase::Idle;   // 재생은 FSM 단계가 아니라 그 위에 얹는 모드다
	bReplayActive = true;
	ReplayFrameIndex = 0;
	ReplayTimeAccumulatorSec = 0.0;

	// 첫 프레임을 즉시 반영해 재생 시작 순간의 튐을 없앤다.
	ActiveState = ReplayFrames[0];

	UE_LOG(LogRobotSim, Log, TEXT("[PickPlaceTask] 재생 시작 — %d 프레임."), ReplayFrames.Num());
}

void APickPlaceTaskActor::StopReplay()
{
	if (!bReplayActive)
	{
		return;
	}

	bReplayActive = false;
	Phase = EPickPlacePhase::Idle;

	UE_LOG(LogRobotSim, Log, TEXT("[PickPlaceTask] 재생 중단 (%d/%d 프레임)."), ReplayFrameIndex, ReplayFrames.Num());
}

void APickPlaceTaskActor::ClearMotionCsv()
{
	const FString FilePath = GetMotionCsvPath();

	const bool bDeleted = IFileManager::Get().Delete(*FilePath, /*RequireExists=*/false, /*EvenReadOnly=*/false, /*Quiet=*/true);

	AccumulatedPriorRows.Reset();
	ReplayFrames.Reset();
	ReplayFrameIndex = 0;

	UE_LOG(LogRobotSim, Log,
		TEXT("[PickPlaceTask] 모션 CSV %s: %s"), bDeleted ? TEXT("삭제") : TEXT("삭제 대상 없음"), *FilePath);
}

void APickPlaceTaskActor::StepReplay(double /*DeltaSec*/)
{
	if (ReplayFrames.Num() == 0)
	{
		bReplayActive = false;
		return;
	}

	// CSV 한 행 = 고정 스텝 하나. 프레임을 되넣기만 한다 — 각속도/토크는 재생 대상이 아니다
	// (그 값들은 저장된 궤적의 파생일 뿐이고, 재생의 계약은 "관절각을 그대로 되돌린다"이다).
	ActiveState = ReplayFrames[ReplayFrameIndex];
	++ReplayFrameIndex;

	if (ReplayFrameIndex >= ReplayFrames.Num())
	{
		bReplayActive = false;
		Phase = EPickPlacePhase::Idle;
		UE_LOG(LogRobotSim, Log, TEXT("[PickPlaceTask] 재생 완료 — %d 프레임 재현."), ReplayFrames.Num());
	}
}

bool APickPlaceTaskActor::IsRecording() const
{
	// **실제로 샘플이 쌓이는 순간**만 true다 — IsReplaying이 재생 프레임 전진 중에만 true인 것과 대칭.
	// RecordCsvRow는 StepFixed에서만 불리고, StepFixed는 FSM이 전진할 때만 돈다. 그 조건을 그대로 반영한다:
	// 로깅 켜짐 + 로봇 있음 + 재생 아님 + 일시정지 아님 + 궤적을 전진시키는 단계(Idle/Done/Aborted 아님).
	// 그래서 Done에 도달하거나 배급 대기(Idle)로 들어가면 REC 표시등이 꺼진다.
	return bEnableCsvLogging && Robot && !bReplayActive && !bPaused
		&& Phase != EPickPlacePhase::Idle && Phase != EPickPlacePhase::Done && Phase != EPickPlacePhase::Aborted;
}

bool APickPlaceTaskActor::IsReplaying() const
{
	return bReplayActive;
}

float APickPlaceTaskActor::GetReplayProgress() const
{
	if (!bReplayActive || ReplayFrames.Num() == 0)
	{
		return 0.0f;
	}
	return static_cast<float>(ReplayFrameIndex) / static_cast<float>(ReplayFrames.Num());
}

#pragma endregion

#pragma endregion

#pragma region APickPlaceTaskActor_UIBinding

//~ 여기 있는 getter는 전부 **캐시/기존 상태만 읽는다**. RNEA도 IK도 FK 재계산도 없다.
//~ BlueprintPure는 바인딩 수 × 프레임만큼 호출되므로 하나라도 계산을 하면 프레임이 무너진다.
//~ 모두 Robot이 null이거나 PIE 시작 전이어도 안전한 기본값을 반환한다.

FText APickPlaceTaskActor::GetRobotDisplayText() const
{
	// 로봇이 없으면 이 액터 이름 — 빈 문자열을 주면 패널이 익명이 되어 어느 로봇인지 알 수 없다.
	return FText::FromString(Robot ? Robot->GetActorNameOrLabel() : GetActorNameOrLabel());
}

FText APickPlaceTaskActor::GetPhaseDisplayText() const
{
	// 로그/CSV와 **같은 문자열**을 쓴다. UMETA DisplayName을 따로 두면 화면과 로그가 갈라져
	// "화면엔 Grasp인데 CSV엔 뭐라고 찍혔나"를 매번 번역해야 한다.
	return FText::FromString(PhaseToString(Phase));
}

float APickPlaceTaskActor::GetPhaseProgress() const
{
	if (PhaseDurationSec <= 0.0)
	{
		return 0.0f;
	}

	return static_cast<float>(FMath::Clamp(PhaseElapsedSec / PhaseDurationSec, 0.0, 1.0));
}

TArray<float> APickPlaceTaskActor::GetJointAnglesDeg() const
{
	TArray<float> Result;
	Result.Reserve(FSerial6DoFModel::NumJoints);

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		Result.Add(static_cast<float>(FMath::RadiansToDegrees(ActiveState.Q[i])));
	}

	return Result;
}

TArray<float> APickPlaceTaskActor::GetJointVelocityDegPerSec() const
{
	TArray<float> Result;
	Result.Reserve(FSerial6DoFModel::NumJoints);

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		Result.Add(static_cast<float>(FMath::RadiansToDegrees(LastJointVelocityRadPerSec.Qd[i])));
	}

	return Result;
}

TArray<float> APickPlaceTaskActor::GetJointTorqueNm() const
{
	TArray<float> Result;
	Result.Reserve(FSerial6DoFModel::NumJoints);

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		Result.Add(static_cast<float>(LastJointTorqueNm.TauNm[i]));
	}

	return Result;
}

TArray<float> APickPlaceTaskActor::GetJointTorqueRatio() const
{
	TArray<float> Result;
	Result.Reserve(FSerial6DoFModel::NumJoints);

	if (!Robot)
	{
		Result.AddZeroed(FSerial6DoFModel::NumJoints);
		return Result;
	}

	const FSerial6DoFModel& Model = Robot->GetModel();

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		const double MaxTorqueNm = Model.JointLimits[i].MaxTorqueNm;

		// 한계가 0 이하면 0을 반환한다 — 나누면 inf/NaN이 게이지로 새어 위젯이 깨진다.
		// (B-02가 경고했듯 기본값 100 N·m는 J1의 실측 −303 N·m에 한참 못 미친다. 그 경우 이 비율은
		//  1.0에 붙박이는데, 그건 게이지 버그가 아니라 **MaxTorqueNm가 실제로 부족하다는 신호**다.)
		Result.Add(MaxTorqueNm > 0.0
			? static_cast<float>(FMath::Clamp(FMath::Abs(LastJointTorqueNm.TauNm[i]) / MaxTorqueNm, 0.0, 1.0))
			: 0.0f);
	}

	return Result;
}

FVector APickPlaceTaskActor::GetToolLocationWorld() const
{
	if (!Robot)
	{
		return FVector::ZeroVector;
	}

	return LocalToWorld(ComputeToolLocalTransform()).GetLocation();
}

float APickPlaceTaskActor::GetLastReachErrorCm() const
{
	return static_cast<float>(LastReachErrorCm);
}

int32 APickPlaceTaskActor::GetCompletedBoxCount() const
{
	return CompletedBoxCount;
}

float APickPlaceTaskActor::GetFrameTimeMs() const
{
	return static_cast<float>(LastFrameTimeMs);
}

bool APickPlaceTaskActor::IsCycleRunning() const
{
	return bCycleStarted
		&& Phase != EPickPlacePhase::Idle
		&& Phase != EPickPlacePhase::Done
		&& Phase != EPickPlacePhase::Aborted;
}

bool APickPlaceTaskActor::IsPaused() const
{
	return bPaused;
}

void APickPlaceTaskActor::SetPaused(bool bInPaused)
{
	bPaused = bInPaused;
}

float APickPlaceTaskActor::GetVelocityScale() const
{
	return static_cast<float>(VelocityScale);
}

void APickPlaceTaskActor::SetVelocityScale(float NewVelocityScale)
{
	// UPROPERTY meta의 ClampMin/ClampMax와 **같은 범위**로 막는다. 에디터 경로와 UI 경로가 다른 범위를
	// 허용하면 슬라이더로만 만들 수 있는 상태가 생긴다. 0이면 궤적 소요시간이 무한대가 된다.
	VelocityScale = FMath::Clamp(static_cast<double>(NewVelocityScale), 0.01, 1.0);
}

float APickPlaceTaskActor::GetDwellSec() const
{
	return static_cast<float>(DwellSec);
}

void APickPlaceTaskActor::SetDwellSec(float NewDwellSec)
{
	// 상한 5초는 UI 슬라이더의 실용 범위다 (그 이상은 데모가 멈춘 것처럼 보인다).
	DwellSec = FMath::Clamp(static_cast<double>(NewDwellSec), 0.0, 5.0);
}

void APickPlaceTaskActor::SaveMotionCsvNow()
{
	WriteCsvToDisk();
}

#pragma endregion

#pragma region APickPlaceTaskActor_DispatcherAPI

bool APickPlaceTaskActor::IsIdle() const
{
	// bCycleStarted 없이 판단하면 안 된다: dispatcher는 이 액터보다 먼저 틱하므로 첫 프레임에는
	// StartCycle이 아직 안 돌았고, 그때 배급하면 뒤이어 실행되는 StartCycle이 Phase를 Idle로
	// 되돌려 배급이 조용히 증발한다.
	//
	// 재생 중에는 Idle이 아니다 — Phase는 Idle이어도 팔이 저장된 궤적을 재생 중이므로, dispatcher가
	// 배급하면 두 모드가 같은 관절을 두고 다툰다.
	return bCycleStarted && !bReplayActive && Phase == EPickPlacePhase::Idle && !AssignedTask.IsValid();
}

void APickPlaceTaskActor::AssignTask(const FPickPlaceTask& InTask)
{
	if (!InTask.IsValid())
	{
		return;
	}

	AssignedTask = InTask;
	EnterPhase(EPickPlacePhase::ToPickApproach);
}

bool APickPlaceTaskActor::CanReachGraspPointWorld(const FVector& GraspPointWorld)
{
	if (!Robot)
	{
		return false;
	}

	const FVector Local = Robot->GetActorTransform().InverseTransformPosition(GraspPointWorld);

	// SolveForVisualGraspPoint가 내부에서 관절 상태를 스냅샷 → 판정 → 복원한다.
	// (복원이 없으면 도달성 조회만으로 팔이 튄다 — dispatcher가 사이클 시작 전에 수십 번 부른다.)
	FRobot6DJointState Solution;
	double ErrorCm = 0.0;

	// 목표 지점 자체.
	if (!SolveForVisualGraspPoint(FTransform(GraspRotation.Quaternion(), Local), Solution, ErrorCm))
	{
		return false;
	}

	// **접근 자세도 검사해야 한다.** 사이클은 목표만 가는 게 아니라 ApproachOffsetCm만큼 위에서
	// 수직 진입/이탈하고(ToPickApproach/ToLift/ToPlaceApproach/ToRetreat), 팔이 위로 뻗을수록
	// 반경이 줄어 접근 자세가 더 빡빡하다. 이걸 빠뜨리면 배급은 되는데 접근 단계에서 Aborted가 나서
	// 반납 → 재배급이 반복된다 — 도달성 판정이 사이클보다 낙관적이면 안 된다.
	return SolveForVisualGraspPoint(
		FTransform(GraspRotation.Quaternion(), Local + ApproachOffsetCm), Solution, ErrorCm);
}

APickPlaceBoxActor* APickPlaceTaskActor::GetCurrentBox() const
{
	if (Dispatcher)
	{
		return AssignedTask.Box;
	}

	return Boxes.IsValidIndex(CurrentBoxIndex) ? Boxes[CurrentBoxIndex].Get() : nullptr;
}

FTransform APickPlaceTaskActor::GetCurrentPlacePoseLocal(bool bApproach) const
{
	if (!Dispatcher)
	{
		return GetPlacePoseLocal(CurrentBoxIndex, bApproach);
	}

	// dispatcher 모드: 슬롯 풀의 소유자가 dispatcher이므로 월드 좌표를 받아 로봇 공간으로 내린다.
	FVector SlotWorld;
	if (!Robot || !Dispatcher->GetDestinationSlotWorld(AssignedTask.DestinationSlotIndex, SlotWorld))
	{
		// 슬롯을 못 얻으면 제자리에 머문다 — 임의의 좌표를 지어내면 팔이 엉뚱한 데로 날아간다.
		return FTransform(GraspRotation.Quaternion(), ComputeToolLocalTransform().GetLocation());
	}

	FVector Location = Robot->GetActorTransform().InverseTransformPosition(SlotWorld);
	if (bApproach)
	{
		Location += ApproachOffsetCm;
	}

	return FTransform(GraspRotation.Quaternion(), Location);
}

void APickPlaceTaskActor::LogRobotSkeletonGeometry()
{
	if (!Robot)
	{
		UE_LOG(LogRobotSim, Warning, TEXT("[APickPlaceTaskActor] Robot이 없어 스켈레톤을 덤프할 수 없습니다."));
		return;
	}

	// 로봇의 protected 멤버에 손대지 않고 컴포넌트를 찾는다 (AActor::GetComponentByClass는 public).
	const UPoseableMeshComponent* Poseable =
		Cast<UPoseableMeshComponent>(Robot->GetComponentByClass(UPoseableMeshComponent::StaticClass()));

	if (!Poseable || !Poseable->GetSkinnedAsset())
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[APickPlaceTaskActor] SkeletalMesh가 없습니다 — RobotConfig에 메시가 지정됐는지 확인하세요."));
		return;
	}

	// 현재 포즈가 아니라 **레퍼런스 스켈레톤(바인드 포즈)**을 읽는다. 수학 모델의 Q=0 자세가
	// 바인드 포즈에 대응한다는 것이 SkeletalMesh 시각화의 전제이므로, LinkOffsets와 비교할 기준은
	// 지금 화면에 보이는 포즈가 아니라 바인드 포즈다.
	const FReferenceSkeleton& RefSkeleton = Poseable->GetSkinnedAsset()->GetRefSkeleton();
	const TArray<FTransform>& RefPose = RefSkeleton.GetRefBonePose();
	const int32 NumBones = RefSkeleton.GetNum();

	UE_LOG(LogRobotSim, Log,
		TEXT("[APickPlaceTaskActor] ===== 레퍼런스 스켈레톤 덤프: %s (뼈 %d개) ====="),
		*Poseable->GetSkinnedAsset()->GetName(), NumBones);
	UE_LOG(LogRobotSim, Log,
		TEXT("[APickPlaceTaskActor] '부모 상대'가 LinkOffsets 후보입니다. J0~J5에 대응하는 뼈를 골라 ")
		TEXT("DA_RobotConfig의 LinkOffsetCm에 옮겨 적으면 수학 팔이 이 메시와 같은 크기가 됩니다."));

	// 컴포넌트 공간 누적: 부모가 항상 자신보다 낮은 인덱스라는 FReferenceSkeleton의 불변식을 이용한다.
	TArray<FTransform> ComponentSpace;
	ComponentSpace.SetNum(NumBones);

	for (int32 i = 0; i < NumBones; ++i)
	{
		const int32 ParentIndex = RefSkeleton.GetParentIndex(i);
		ComponentSpace[i] = ParentIndex == INDEX_NONE
			? RefPose[i]
			: RefPose[i] * ComponentSpace[ParentIndex];

		const FVector LocalLoc = RefPose[i].GetLocation();
		const FVector WorldLoc = ComponentSpace[i].GetLocation();
		const FName ParentName = ParentIndex == INDEX_NONE ? NAME_None : RefSkeleton.GetBoneName(ParentIndex);

		UE_LOG(LogRobotSim, Log,
			TEXT("[APickPlaceTaskActor] [본 %2d] '%s' (부모 %d '%s') — 부모 상대 (%.1f, %.1f, %.1f)cm, ")
			TEXT("컴포넌트 공간 (%.1f, %.1f, %.1f)cm"),
			i, *RefSkeleton.GetBoneName(i).ToString(), ParentIndex, *ParentName.ToString(),
			LocalLoc.X, LocalLoc.Y, LocalLoc.Z, WorldLoc.X, WorldLoc.Y, WorldLoc.Z);
	}

	// 현재 수학 모델과 나란히 찍어 크기 차이를 한눈에 보이게 한다.
	const FSerial6DoFModel& Model = Robot->GetModel();
	const FVector MathEE = Model.ComputeEndEffectorTransform(FRobot6DJointState()).GetLocation();
	const double ToolLen = Model.ToolOffset.GetLocation().Size();

	UE_LOG(LogRobotSim, Log,
		TEXT("[APickPlaceTaskActor] ----- 대조: 현재 수학 모델 홈 EE = (%.1f, %.1f, %.1f)cm [베이스에서 %.1fcm], ")
		TEXT("그중 ToolOffset이 %.1fcm를 차지 -----"),
		MathEE.X, MathEE.Y, MathEE.Z, MathEE.Size(), ToolLen);

	if (ToolLen > 30.0)
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[APickPlaceTaskActor] ToolOffset 길이가 %.1fcm입니다 — 그리퍼 치고 비정상적으로 깁니다. ")
			TEXT("수학 팔이 메시보다 작아서 캘리브레이션이 그 차이를 ToolOffset에 밀어넣었을 가능성이 큽니다. ")
			TEXT("이 경우 **홈 자세에서만** 수학 EE와 시각 그리퍼가 일치하고, 관절이 돌면 %.1fcm 지렛대가 ")
			TEXT("수학 EE를 엉뚱한 곳으로 휘둘러 파지가 어긋납니다. 위 뼈 치수로 LinkOffsets를 다시 authoring하세요."),
			ToolLen, ToolLen);
	}
}

#pragma endregion

#pragma region APickPlaceTaskActor_FSM

void APickPlaceTaskActor::StepFixed(double DeltaSec)
{
	PreviousState = ActiveState;

	PhaseElapsedSec += DeltaSec;
	SimTimeSec += DeltaSec;

	// Grasp/Release는 정지 대기이므로 궤적 평가가 없다 (ActiveState 불변). 나머지는 모두 궤적 추종이다.
	if (Phase != EPickPlacePhase::Grasp && Phase != EPickPlacePhase::Release)
	{
		EvaluateTrajectory();
	}

	// 반드시 EvaluateTrajectory **뒤**여야 한다 — 앞에서 부르면 ActiveState == PreviousState라
	// 각속도가 항상 0이 되고 마찰 항까지 죽는다. RecordCsvRow보다 앞이어야 CSV가 이 스텝의 토크를 쓴다.
	UpdateTelemetryCache();

	RecordCsvRow();

	if (PhaseElapsedSec >= PhaseDurationSec)
	{
		AdvanceToNextPhase();
	}
}

void APickPlaceTaskActor::EnterPhase(EPickPlacePhase NewPhase)
{
	Phase = NewPhase;
	PhaseElapsedSec = 0.0;
	PhaseDurationSec = 0.0;

	// 모드에 따라 배급된 박스 또는 자기 배열의 박스를 쓴다 (FSM 본문은 두 모드가 동일하다).
	APickPlaceBoxActor* Box = GetCurrentBox();

	switch (Phase)
	{
	case EPickPlacePhase::ToPickApproach:
	{
		if (!Box)
		{
			EnterPhase(EPickPlacePhase::Done);
			return;
		}

		// 파지 지점을 여기서 한 번 스냅샷한다. 박스는 물리 바디라 미세하게 굴러갈 수 있는데,
		// 접근(ToPickApproach)과 하강(ToPick)이 서로 다른 지점을 겨냥하면 수직 진입이 깨진다.
		// 박스는 월드 액터이므로 로봇 공간으로 내려서 다른 배치값과 같은 공간에 맞춘다.
		// GetActorLocation()이 아니라 파지점(바운드 윗면 중심)을 쓴다 — 메시 피벗 위치에 의존하지 않고,
		// 그리퍼가 박스 속이 아니라 윗면에 얹힌다.
		const FVector GraspLocal = Robot->GetActorTransform().InverseTransformPosition(Box->GetGraspPointWorld());
		PickPoseLocal = FTransform(GraspRotation.Quaternion(), GraspLocal);

		if (!BeginTrajectoryTo(GetPickApproachPoseLocal()))
		{
			return;
		}
		break;
	}

	case EPickPlacePhase::ToPick:
		if (!BeginTrajectoryTo(PickPoseLocal))
		{
			return;
		}
		break;

	case EPickPlacePhase::Grasp:
	{
		if (Box)
		{
			// 로봇을 현재 ActiveState로 먼저 동기화한다. EnterPhase는 StepFixed 안에서 불리므로
			// 이 시점의 로봇 상태는 아직 **이전 프레임 것**이고, 그대로 읽으면 한 프레임 어긋난
			// 그리퍼 기준으로 상대 변환이 굳어져 파지 내내 그 오차가 따라다닌다.
			Robot->SetJointAngles(ActiveState);

			Box->BeginGrasp();
			HeldBox = Box;

			// **박스는 수학 EE가 아니라 시각 파지점(흡착판)에 붙는다.** 메시를 축소하지 않기로 했으므로
			// 두 점은 홈 자세에서만 겹치고, 보이는 그리퍼가 물체를 잡는 쪽이 우선이기 때문이다.
			//
			// 파지 순간의 상대 변환을 그대로 보존한다 — 박스를 흡착판으로 순간이동시키지 않아야 자연스럽다.
			// 이게 성립하는 건 BeginTrajectoryTo의 반복 보정이 "시각 파지점이 박스 윗면에 MaxReachErrorCm
			// 이내로 도달"을 이미 보장했기 때문이다. 그 보장이 없으면 이 상대 변환이 곧 "박스가 그리퍼에서
			// 떨어진 거리"로 굳어져 공중에 뜬 채 따라다닌다 (실제로 그랬다).
			HeldBoxRelativeToTool =
				Box->GetActorTransform().GetRelativeTransform(Robot->GetVisualGraspPointWorld());

			const FVector GraspWorld = Robot->GetVisualGraspPointWorld().GetLocation();
			const FVector MathEEWorld = LocalToWorld(ComputeToolLocalTransform()).GetLocation();
			const FVector BoxTopWorld = Box->GetGraspPointWorld();
			const double GapCm = FVector::Dist(GraspWorld, BoxTopWorld);

			UE_LOG(LogRobotSim, Log,
				TEXT("[APickPlaceTaskActor] 파지 — 흡착판 (%.1f, %.1f, %.1f) / 박스 윗면 (%.1f, %.1f, %.1f) → 간격 %.2fcm. ")
				TEXT("[visual calibration offset: 수학 EE (%.1f, %.1f, %.1f)와 %.1fcm 차이 — 설계상 남기는 값]"),
				GraspWorld.X, GraspWorld.Y, GraspWorld.Z,
				BoxTopWorld.X, BoxTopWorld.Y, BoxTopWorld.Z, GapCm,
				MathEEWorld.X, MathEEWorld.Y, MathEEWorld.Z, FVector::Dist(GraspWorld, MathEEWorld));

			// 흡착판은 ToPickApproach에서 스냅샷한 파지 지점으로 정확히 갔는데(BeginTrajectoryTo가
			// MaxReachErrorCm 이내를 보장한다) 박스와 벌어져 있다면, 그 사이에 **박스가 움직인** 것이다.
			// 원인은 대개 스폰 시 박스끼리 겹쳐 물리가 밀어낸 경우다. 이 간격이 그대로 파지 상대 변환에
			// 굳어져 박스가 흡착판에서 그만큼 떨어진 채 따라다니므로, 조용히 넘기면 안 된다.
			if (GapCm > MaxReachErrorCm)
			{
				const FVector SnapshotWorld = LocalToWorld(PickPoseLocal).GetLocation();
				UE_LOG(LogRobotSim, Warning,
					TEXT("[APickPlaceTaskActor] **박스가 흡착판에서 %.1fcm 떨어져 파지됩니다** (허용 %.1fcm). ")
					TEXT("흡착판은 스냅샷 지점 (%.1f, %.1f, %.1f)에 제대로 갔으므로, ToPickApproach(%.2fs) 동안 ")
					TEXT("박스가 %.1fcm 움직였다는 뜻입니다 — SourceSlotStrideCm이 좁아 박스끼리 밀어냈을 가능성이 큽니다. ")
					TEXT("이 간격은 파지 내내 유지되어 박스가 공중에 뜬 것처럼 보입니다."),
					GapCm, MaxReachErrorCm, SnapshotWorld.X, SnapshotWorld.Y, SnapshotWorld.Z,
					PhaseDurationSec, FVector::Dist(SnapshotWorld, BoxTopWorld));
			}
		}

		CurrentTargetLocal = ComputeToolLocalTransform();
		PhaseDurationSec = DwellSec;
		break;
	}

	case EPickPlacePhase::ToLift:
		// 집은 지점 바로 위로 되돌아간다 — 내려온 경로를 그대로 거슬러 올라가므로 박스가 옆면에 긁히지 않는다.
		if (!BeginTrajectoryTo(GetPickApproachPoseLocal()))
		{
			return;
		}
		break;

	case EPickPlacePhase::ToPlaceApproach:
		if (!BeginTrajectoryTo(GetCurrentPlacePoseLocal(/*bApproach=*/true)))
		{
			return;
		}
		break;

	case EPickPlacePhase::ToPlace:
		if (!BeginTrajectoryTo(GetCurrentPlacePoseLocal(/*bApproach=*/false)))
		{
			return;
		}
		break;

	case EPickPlacePhase::Release:
	{
		// 대기 시작 시점에 놓는다. 툴은 Release 내내 정지해 있으므로 박스가 DwellSec 동안 자리를 잡은 뒤
		// 팔이 빠지게 되어, 이탈 궤적이 박스를 건드릴 여지가 줄어든다.
		if (HeldBox)
		{
			HeldBox->EndGrasp();
			HeldBox = nullptr;
		}

		CurrentTargetLocal = ComputeToolLocalTransform();
		PhaseDurationSec = DwellSec;
		break;
	}

	case EPickPlacePhase::ToRetreat:
		if (!BeginTrajectoryTo(GetCurrentPlacePoseLocal(/*bApproach=*/true)))
		{
			return;
		}
		break;

	case EPickPlacePhase::Done:
		UE_LOG(LogRobotSim, Log,
			TEXT("[APickPlaceTaskActor] 사이클 완료 — 시뮬레이션 시간 %.2fs, CSV 샘플 %d행"),
			SimTimeSec, bEnableCsvLogging ? FMath::Max(0, CsvRows.Num() - 1) : 0);
		WriteCsvToDisk();
		return;

	case EPickPlacePhase::Aborted:
		// AbortCycle이 이유를 이미 로그에 남겼다.
		WriteCsvToDisk();
		return;

	case EPickPlacePhase::Idle:
	default:
		return;
	}

	// dispatcher 모드에는 CurrentBoxIndex가 없다(배급된 박스를 직접 들고 있다) — 그대로 찍으면
	// "박스 -1"이 나와 배급이 실패한 것처럼 보인다. 모드에 맞는 식별자를 쓴다.
	const FVector TargetLocal = CurrentTargetLocal.GetLocation();
	const FString BoxLabel = Dispatcher
		? (Box ? Box->GetName() : TEXT("없음"))
		: FString::FromInt(CurrentBoxIndex);

	UE_LOG(LogRobotSim, Log,
		TEXT("[APickPlaceTaskActor] 단계 → %s (박스 %s, 소요 %.2fs, 목표 로봇공간 (%.1f, %.1f, %.1f)cm)"),
		PhaseToString(Phase), *BoxLabel, PhaseDurationSec, TargetLocal.X, TargetLocal.Y, TargetLocal.Z);
}

void APickPlaceTaskActor::AdvanceToNextPhase()
{
	switch (Phase)
	{
	case EPickPlacePhase::ToPickApproach:  EnterPhase(EPickPlacePhase::ToPick);          break;
	case EPickPlacePhase::ToPick:          EnterPhase(EPickPlacePhase::Grasp);           break;
	case EPickPlacePhase::Grasp:           EnterPhase(EPickPlacePhase::ToLift);          break;
	case EPickPlacePhase::ToLift:          EnterPhase(EPickPlacePhase::ToPlaceApproach); break;
	case EPickPlacePhase::ToPlaceApproach: EnterPhase(EPickPlacePhase::ToPlace);         break;
	case EPickPlacePhase::ToPlace:         EnterPhase(EPickPlacePhase::Release);         break;
	case EPickPlacePhase::Release:         EnterPhase(EPickPlacePhase::ToRetreat);       break;

	case EPickPlacePhase::ToRetreat:
	{
		// 여기가 "박스 하나를 끝낸" 유일한 지점이다 (두 모드 공통). Aborted는 세지 않는다 — 실패다.
		++CompletedBoxCount;

		// dispatcher 모드: 작업 하나가 끝났으므로 완료 보고하고 Idle로 돌아가 다음 배급을 기다린다.
		// 다음 작업을 스스로 고르지 않는 것이 핵심이다 — 그게 "할당"을 dispatcher가 소유한다는 뜻이다.
		if (Dispatcher)
		{
			Dispatcher->CompleteTask(this, AssignedTask);
			AssignedTask = FPickPlaceTask();
			EnterPhase(EPickPlacePhase::Idle);
			break;
		}

		// standalone: 남은 박스가 있으면 다음 pick으로 돌아가고, 없으면 종료한다.
		const int32 NextBox = FindNextValidBoxIndex(CurrentBoxIndex + 1);
		if (NextBox != INDEX_NONE)
		{
			CurrentBoxIndex = NextBox;
			EnterPhase(EPickPlacePhase::ToPickApproach);
		}
		else
		{
			EnterPhase(EPickPlacePhase::Done);
		}
		break;
	}

	case EPickPlacePhase::Idle:
	case EPickPlacePhase::Done:
	case EPickPlacePhase::Aborted:
	default:
		break;
	}
}

void APickPlaceTaskActor::AbortCycle(const FString& Reason)
{
	UE_LOG(LogRobotSim, Error, TEXT("[APickPlaceTaskActor] 사이클 중단 — %s"), *Reason);

	// 들고 있던 박스는 놓는다. 안 놓으면 물리가 꺼진 채 공중에 영원히 멈춰 있어 더 헷갈린다.
	if (HeldBox)
	{
		HeldBox->EndGrasp();
		HeldBox = nullptr;
	}

	// dispatcher 모드에서 중단은 **이 로봇의 죽음이 아니라 이 작업의 실패**다. 박스와 슬롯을 함께
	// 반납하고 블랙리스트에 올린 뒤 Idle로 돌아가면, dispatcher가 다른 박스를 주거나 다른 로봇이
	// 그 박스를 맡는다. Aborted로 굳히면 로봇 한 대가 통째로 죽어 나머지 작업이 멈춘다.
	if (Dispatcher)
	{
		Dispatcher->ReturnTask(this, AssignedTask, /*bBlacklist=*/true, Reason);
		AssignedTask = FPickPlaceTask();
		EnterPhase(EPickPlacePhase::Idle);
		return;
	}

	EnterPhase(EPickPlacePhase::Aborted);
}

int32 APickPlaceTaskActor::FindNextValidBoxIndex(int32 SearchFrom) const
{
	for (int32 i = FMath::Max(0, SearchFrom); i < Boxes.Num(); ++i)
	{
		if (Boxes[i])
		{
			return i;
		}
	}
	return INDEX_NONE;
}

#pragma endregion

#pragma region APickPlaceTaskActor_Trajectory

bool APickPlaceTaskActor::SolveForVisualGraspPoint(
	const FTransform& DesiredGraspLocal, FRobot6DJointState& OutState, double& OutFinalErrorCm)
{
	OutState = ActiveState;
	OutFinalErrorCm = TNumericLimits<double>::Max();

	if (!Robot)
	{
		return false;
	}

	FRobotDlsIKOptions Options;
	if (!bConstrainGraspRotation)
	{
		// 회전을 목적함수에서 완전히 빼 위치만 쫓게 한다. 손목이 제한된 팔레타이징 로봇에서는
		// 이게 필수다 — 안 그러면 낼 수 없는 회전을 쫓느라 위치를 희생한다.
		Options.RotationWeight = 0.0;
	}

	const FTransform RobotToWorld = Robot->GetActorTransform();
	const FVector DesiredGraspWorld = RobotToWorld.TransformPosition(DesiredGraspLocal.GetLocation());

	// 시각 파지점이 없으면(본 미지정 등) 로봇이 수학 EE로 폴백하므로, 반복은 1회에 수렴한다.
	// 즉 이 경로는 기존 "수학 EE를 목표로" 동작과 동일해진다.
	FTransform MathTargetLocal = DesiredGraspLocal;

	// 관절각을 실제로 적용해야 PoseableMesh가 동기화되어 시각 파지점을 읽을 수 있다. 원래 자세는
	// 복원해 둔다 — 이 함수는 조회이지 상태 변경이 아니어야 한다 (Tick이 매 프레임 ActiveState를 밀어넣는다).
	const FRobot6DJointState SavedState = Robot->GetJointState();

	constexpr int32 MaxVisualIterations = 12;
	bool bConverged = false;

	// 최선해를 따로 보관한다. 감쇠 반복은 단조 감소가 보장되지 않으므로, 마지막 반복이 최선이라는
	// 보장이 없다 — 중간에 가장 좋았던 해를 잃으면 안 된다.
	FRobot6DJointState BestState = ActiveState;
	double BestErrorCm = TNumericLimits<double>::Max();

	// IK seed 후보. 현재 자세를 먼저 쓴다 — 해가 현재 구성 근처에 나와야 궤적이 매끄럽다.
	// 실패하면 **홈 자세**로 재시도한다: DLS는 국소 수렴이라 seed에 민감하고, 작업 영역 가장자리에서는
	// "방금 있던 자리로 되돌아가는 것"조차 실패한다(실제로 ToPickApproach는 성공한 목표에 대해
	// 같은 좌표의 ToLift가 43cm로 실패했다 — 차이는 seed뿐이었다).
	//
	// dispatcher의 도달성 캐시는 홈 자세를 seed로 계산하므로, 이 재시도가 **캐시와 사이클의 판정을
	// 일치시킨다.** 없으면 "배급은 됐는데 실행하면 중단"이 계속 나고 블랙리스트만 쌓인다.
	const FRobot6DJointState SeedCandidates[] = { ActiveState, FRobot6DJointState() };

	for (const FRobot6DJointState& Seed : SeedCandidates)
	{
		// seed마다 목표를 원점에서 다시 시작한다 — 앞 seed가 밀어놓은 보정값을 물려받으면 안 된다.
		MathTargetLocal = DesiredGraspLocal;

		for (int32 Iter = 0; Iter < MaxVisualIterations; ++Iter)
		{
			const FRobotDlsIKResult IKResult =
				FRobotDlsIK::SolveDlsIK(Robot->GetModel(), Seed, MathTargetLocal, Options);

			// 해를 적용해 메시를 그 자세로 동기화한 뒤 시각 파지점을 측정한다.
			Robot->SetJointAngles(IKResult.Solution);
			const FVector ActualGraspWorld = Robot->GetVisualGraspPointWorld().GetLocation();

			const FVector ErrorWorld = DesiredGraspWorld - ActualGraspWorld;
			const double ErrorCm = ErrorWorld.Size();

			if (ErrorCm < BestErrorCm)
			{
				BestErrorCm = ErrorCm;
				BestState = IKResult.Solution;
			}

			if (ErrorCm <= MaxReachErrorCm)
			{
				bConverged = true;
				break;
			}

			// 수학 목표를 오차 방향으로 **감쇠해서** 민다. 감쇠 없이(α=1) 밀면 발산한다:
			// 시각 그리퍼는 수학 EE의 약 2배 반경으로 움직이므로 f(X)≈2X이고, 그러면
			// X ← X + (T − 2X) = T − X 가 되어 두 값 사이를 영원히 진동한다.
			// α=0.5면 반복 행렬이 (1−2α)=0이 되어 한 번에 수렴한다. VisualSolveDamping 주석 참조.
			MathTargetLocal.SetLocation(
				MathTargetLocal.GetLocation() + RobotToWorld.InverseTransformVector(ErrorWorld) * VisualSolveDamping);
		}

		if (bConverged)
		{
			break; // 첫 seed(현재 자세)로 풀렸으면 홈 재시도는 불필요하다
		}
	}

	Robot->SetJointAngles(SavedState);

	OutState = BestState;
	OutFinalErrorCm = BestErrorCm;
	return bConverged;
}

bool APickPlaceTaskActor::BeginTrajectoryTo(const FTransform& TargetLocal)
{
	CurrentTargetLocal = TargetLocal;
	TrajectoryStartState = ActiveState;
	TrajectoryGoalState = ActiveState;

	if (!Robot)
	{
		PhaseDurationSec = MinTrajectoryDurationSec;
		return true;
	}

	// 목표는 이미 로봇 액터 공간이므로 그대로 넘긴다 — 모델이 BaseTransform=Identity인 같은 공간에서
	// FK를 계산하기 때문이다. (배치값을 월드로 두면 여기서 매번 변환해야 하고, 로봇이 원점을 벗어나는
	// 순간 기본값이 전부 무의미해진다.)
	//
	// **목표는 시각 파지점이 도달할 지점이다** — 박스가 붙는 곳이 거기이기 때문이다. 수학 EE를 그 지점에
	// 보내면 그리퍼는 딴 데 있게 된다(visual calibration offset). 반복 보정이 그 차이를 흡수한다.
	//
	// IK는 단계 진입 시 한 번만 푼다(반복 보정 포함). 매 프레임 풀면 singularity 근처에서 해가
	// 프레임마다 튀어 궤적이 비결정론적으로 변한다 — 고정 타임스텝의 결정론이 무의미해진다.
	double VisualErrorCm = 0.0;
	FRobot6DJointState Solution;
	const bool bVisualConverged = SolveForVisualGraspPoint(TargetLocal, Solution, VisualErrorCm);

	// 성공/실패 양쪽 다 남긴다. 실패했을 때의 값이 오히려 UI에서 더 중요하다 — 중단 사유가 화면에 뜬다.
	LastReachErrorCm = VisualErrorCm;

	// 도달 판정은 **시각 파지점 오차** 기준이다 — 사용자가 눈으로 보는 것도, 박스가 실제로 붙는 것도
	// 그 점이기 때문이다. 수학 EE 오차가 0이어도 그리퍼가 박스에서 1m 떨어져 있으면 실패다.
	if (!bVisualConverged)
	{
		// 최선해로 진행하면 그리퍼가 목표에서 한참 떨어진 채 움직이고, 박스는 거기 붙어 날아다닌다 —
		// 겉보기엔 "이상하게 잡는" 버그처럼 보이지만 실제 원인은 배치이거나 시각 offset 발산이다.
		// 그래서 조용히 진행하지 않고 멈추고 숫자를 말한다.
		// 탐색 로그와 **같은 양(XY반경 + 높이)** 으로 찍는다. 3D 거리로 찍으면 "구간과 비교하라"는
		// 안내가 성립하지 않는다 (탐색은 XY반경 기준이다).
		const FVector T = TargetLocal.GetLocation();
		const double TargetRadiusXY = FVector2D(T.X, T.Y).Size();

		AbortCycle(FString::Printf(
			TEXT("%s 단계에서 **시각 그리퍼**를 목표 (%.1f, %.1f, %.1f)cm[로봇 공간, **XY반경 %.1fcm, 높이 z=%.1fcm**]에 ")
			TEXT("보낼 수 없습니다. 반복 보정 후 잔여 오차 %.1fcm (허용 %.1fcm). ")
			TEXT("위의 '시각 그리퍼 도달 XY반경 탐색' 구간과 이 XY반경 %.1fcm를 비교하세요 — 구간보다 멀면 목표를 당기고, ")
			TEXT("**구간보다 가까우면 오히려 밀어내야 합니다**(대형 팔은 최소 반경 dead zone이 있습니다). ")
			TEXT("구간 안인데도 실패한다면 반복 보정이 진동하는 것이므로 VisualSolveDamping(현재 %.2f)을 낮추세요."),
			PhaseToString(Phase), T.X, T.Y, T.Z, TargetRadiusXY, T.Z,
			VisualErrorCm, MaxReachErrorCm, TargetRadiusXY, VisualSolveDamping));
		return false;
	}

	TrajectoryGoalState = Solution;
	PhaseDurationSec = ComputeTrajectoryDuration(TrajectoryStartState, TrajectoryGoalState);
	return true;
}

double APickPlaceTaskActor::ComputeTrajectoryDuration(
	const FRobot6DJointState& From, const FRobot6DJointState& To) const
{
	double Duration = MinTrajectoryDurationSec;

	if (!Robot)
	{
		return Duration;
	}

	const FSerial6DoFModel& Model = Robot->GetModel();

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		const double DeltaRad = FMath::Abs(To.Q[i] - From.Q[i]);
		const double MaxVelRadPerSec = Model.JointLimits[i].MaxVelRadPerSec * VelocityScale;

		if (MaxVelRadPerSec <= UE_DOUBLE_KINDA_SMALL_NUMBER)
		{
			continue;
		}

		// smoothstep은 중간에서 평균의 15/8배 속도로 지나가므로, 평균속도로 역산하면 한계를 넘는다.
		// 피크 기준으로 역산해야 궤적 전 구간에서 |qd_i| <= MaxVel_i · VelocityScale이 보장된다.
		const double RequiredSec = QuinticPeakVelocityRatio * DeltaRad / MaxVelRadPerSec;
		Duration = FMath::Max(Duration, RequiredSec);
	}

	return Duration;
}

void APickPlaceTaskActor::EvaluateTrajectory()
{
	// PhaseDurationSec은 MinTrajectoryDurationSec 하한이 걸려 있어 0이 될 수 없다(0 나눗셈 방지).
	const double U = FMath::Clamp(PhaseElapsedSec / FMath::Max(PhaseDurationSec, UE_DOUBLE_KINDA_SMALL_NUMBER), 0.0, 1.0);
	const double S = QuinticSmoothStep(U);

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		ActiveState.Q[i] = TrajectoryStartState.Q[i] + (TrajectoryGoalState.Q[i] - TrajectoryStartState.Q[i]) * S;
	}
}

FTransform APickPlaceTaskActor::ComputeToolLocalTransform() const
{
	if (!Robot)
	{
		return FTransform::Identity;
	}

	// 로봇 컴포넌트의 트랜스폼이 아니라 수학 FK로 계산한다. 컴포넌트는 이번 프레임의
	// SetJointAngles 전이라 아직 이전 자세일 수 있지만, ActiveState는 항상 최신이기 때문이다.
	return Robot->GetModel().ComputeEndEffectorTransform(ActiveState);
}

FTransform APickPlaceTaskActor::LocalToWorld(const FTransform& Local) const
{
	return Robot ? Local * Robot->GetActorTransform() : Local;
}

void APickPlaceTaskActor::UpdateHeldBoxTransform()
{
	if (!HeldBox)
	{
		return;
	}

	if (!Robot)
	{
		return;
	}

	// **시각 파지점을 따라간다** (수학 EE가 아니다) — 박스가 보이는 흡착판에 붙어 있어야 하기 때문이다.
	// 파지 중 박스는 물리가 꺼져 있으므로 트랜스폼을 직접 써도 Chaos와 싸우지 않는다.
	// TeleportPhysics로 이동 속도가 물리 바디에 누적되지 않게 한다 (놓는 순간의 튐 방지).
	HeldBox->SetActorTransform(HeldBoxRelativeToTool * Robot->GetVisualGraspPointWorld(),
		/*bSweep=*/false, /*OutSweepHitResult=*/nullptr, ETeleportType::TeleportPhysics);
}

FTransform APickPlaceTaskActor::GetPickApproachPoseLocal() const
{
	FTransform Pose = PickPoseLocal;
	Pose.SetLocation(Pose.GetLocation() + ApproachOffsetCm);
	return Pose;
}

FTransform APickPlaceTaskActor::GetPlacePoseLocal(int32 BoxIndex, bool bApproach) const
{
	// StartCycle의 BuildPalletSlots가 DestinationSurfaceActor 기준으로 확정한 슬롯을 쓴다.
	// 슬롯이 없으면(도착지 미지정) 현재 툴 위치를 그대로 반환해 제자리에 머문다 — StartCycle이
	// 이미 Warning을 냈고, 여기서 임의의 좌표를 지어내면 팔이 엉뚱한 데로 날아간다.
	if (!PalletSlotLocations.IsValidIndex(BoxIndex))
	{
		return FTransform(GraspRotation.Quaternion(), ComputeToolLocalTransform().GetLocation());
	}

	FVector Location = PalletSlotLocations[BoxIndex];

	if (bApproach)
	{
		Location += ApproachOffsetCm;
	}

	return FTransform(GraspRotation.Quaternion(), Location);
}

#pragma endregion

#pragma region APickPlaceTaskActor_Telemetry

void APickPlaceTaskActor::UpdateTelemetryCache()
{
	if (!Robot)
	{
		LastJointVelocityRadPerSec = FRobot6DJointVelocity();
		LastJointTorqueNm = FRobot6DJointTorque();
		return;
	}

	// 각속도: 고정 스텝 유한차분. 스텝 간격이 항상 FixedTimeStepSec라 프레임레이트에 오염되지 않는다.
	// (StepFixed가 EvaluateTrajectory 전에 PreviousState를 잡아 두므로 두 상태는 정확히 한 스텝 차다.)
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		LastJointVelocityRadPerSec.Qd[i] = (ActiveState.Q[i] - PreviousState.Q[i]) / FixedTimeStepSec;
	}

	// qdd = 0 — **의도적이다.** 현 구동은 위치 지령 + 관절공간 보간이라 관절 가속도가 동역학 적분의
	// 결과가 아니다. 궤적의 2차 미분을 넣어도 "실제 구동 토크"라는 물리적 의미가 없으므로,
	// 정직하게 준정적(quasi-static) 추정으로 남긴다. 중력·마찰은 살아 있고 관성·코리올리는 빠진다.
	const FRobot6DJointAcceleration ZeroAccel;

	LastJointTorqueNm = SolveInverseDynamicsRNEA(
		Robot->GetModel(), ActiveState, LastJointVelocityRadPerSec, ZeroAccel, MakeTelemetryRNEAOptions());
}

void APickPlaceTaskActor::UpdateHoldingTorqueCache()
{
	if (!Robot)
	{
		LastJointVelocityRadPerSec = FRobot6DJointVelocity();
		LastJointTorqueNm = FRobot6DJointTorque();
		return;
	}

	// 정지 상태이므로 각속도는 0이다. 마찰(qd에 비례)도 자연히 사라져 순수 중력 보상 토크만 남는다.
	LastJointVelocityRadPerSec = FRobot6DJointVelocity();
	LastJointTorqueNm = ComputeGravityTorque(Robot->GetModel(), ActiveState, MakeTelemetryRNEAOptions());
}

#pragma endregion

#pragma region APickPlaceTaskActor_Logging

void APickPlaceTaskActor::RecordCsvRow()
{
	if (!bEnableCsvLogging || !Robot || bPaused)
	{
		return;
	}

	// CSV는 월드 좌표로 남긴다 — 레벨에서 눈으로 확인한 값과 바로 대조할 수 있어야 하기 때문이다.
	// 두 값 모두 ActiveState/CurrentTargetLocal에서 직접 계산하므로 로봇 컴포넌트 상태와 무관하다.
	const FTransform ToolWorld = LocalToWorld(ComputeToolLocalTransform());
	const FTransform TargetWorld = LocalToWorld(CurrentTargetLocal);
	const FVector ToolLocation = ToolWorld.GetLocation();
	const FRotator ToolRotation = ToolWorld.Rotator();

	// 수학 EE와 목표의 오차는 기록하지 않는다. 목표는 **시각 그리퍼**가 가야 할 지점이고 수학 EE는
	// 거기서 visual calibration offset만큼 떨어져 있는 것이 정상이므로(ToPick에서 100cm 넘게 나온다),
	// "위치 오차"라는 이름으로 남기면 IK가 고장난 것처럼 읽힌다. 도달 실패는 Aborted 로그가 말한다.

	FString Row = FString::Printf(TEXT("%.5f,%s,%d"), SimTimeSec, PhaseToString(Phase), CurrentBoxIndex);

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		Row += FString::Printf(TEXT(",%.5f"), FMath::RadiansToDegrees(ActiveState.Q[i]));
	}

	// 각속도는 고정 타임스텝 유한차분으로 구한다. 스텝 간격이 항상 FixedTimeStepSec로 일정하므로
	// 프레임레이트에 오염되지 않은 물리값이 된다 — 가변 DeltaSeconds로는 얻을 수 없는 성질이다.
	// UpdateTelemetryCache가 이미 계산해 뒀다 — 여기서 다시 계산하면 UI 게이지와 CSV가 갈라질 수 있다.
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		Row += FString::Printf(TEXT(",%.5f"), FMath::RadiansToDegrees(LastJointVelocityRadPerSec.Qd[i]));
	}

	Row += FString::Printf(TEXT(",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f"),
		ToolLocation.X, ToolLocation.Y, ToolLocation.Z,
		ToolRotation.Roll, ToolRotation.Pitch, ToolRotation.Yaw);

	const FVector TargetLocation = TargetWorld.GetLocation();
	Row += FString::Printf(TEXT(",%.4f,%.4f,%.4f,%d"),
		TargetLocation.X, TargetLocation.Y, TargetLocation.Z, HeldBox ? 1 : 0);

	// B-02 RNEA 토크 (N·m). **캐시를 읽을 뿐 여기서 RNEA를 돌리지 않는다** — UI 게이지와 같은 값이어야
	// "화면에서 본 토크"와 "CSV의 토크"가 같은 스텝의 같은 수라는 것이 보장된다.
	// qdd=0 준정적 근사다 (LastJointTorqueNm 주석 참조).
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		Row += FString::Printf(TEXT(",%.5f"), LastJointTorqueNm.TauNm[i]);
	}

	CsvRows.Add(MoveTemp(Row));
}

void APickPlaceTaskActor::ResolveCsvFileName()
{
	ResolvedCsvFileName = CsvFileName;

	const UWorld* World = GetWorld();
	if (!bEnableCsvLogging || !World || CsvFileName.IsEmpty())
	{
		return;
	}

	// 같은 파일명을 쓰는 다른 태스크 액터가 있는지 본다. 있으면 서로 덮어써서 한쪽 데이터가
	// 조용히 사라진다 — 둘 다 SaveStringArrayToFile로 같은 경로에 쓰고 나중에 끝난 쪽이 이긴다.
	bool bCollides = false;
	for (TActorIterator<APickPlaceTaskActor> It(World); It; ++It)
	{
		const APickPlaceTaskActor* Other = *It;
		if (Other != this && Other->bEnableCsvLogging && Other->CsvFileName == CsvFileName)
		{
			bCollides = true;
			break;
		}
	}

	if (!bCollides)
	{
		return;
	}

	// 충돌 시 **양쪽 모두** 액터 이름을 붙인다. 한쪽만 붙이면 남은 'PickPlace.csv'가 누구 것인지
	// 알 수 없어 오히려 헷갈린다 — 각자 자기를 기준으로 판단하므로 자연히 양쪽 다 붙는다.
	const FString Base = FPaths::GetBaseFilename(CsvFileName);
	const FString Ext = FPaths::GetExtension(CsvFileName, /*bIncludeDot=*/true);
	ResolvedCsvFileName = FString::Printf(TEXT("%s_%s%s"), *Base, *GetName(), *Ext);

	UE_LOG(LogRobotSim, Warning,
		TEXT("[APickPlaceTaskActor] CSV 파일명 '%s'을(를) 다른 태스크 액터도 쓰고 있어 서로 덮어쓸 뻔했습니다 — ")
		TEXT("이 액터는 '%s'로 기록합니다. 로봇마다 CsvFileName을 직접 지정하는 편이 읽기 좋습니다."),
		*CsvFileName, *ResolvedCsvFileName);
}

FString APickPlaceTaskActor::GetMotionCsvPath() const
{
	// StartCycle이 확정한 이름을 쓴다. EndPlay가 StartCycle 없이 불릴 수도 있으므로 폴백을 둔다.
	const FString FileName = ResolvedCsvFileName.IsEmpty() ? CsvFileName : ResolvedCsvFileName;
	return FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("RobotSim"), FileName);
}

void APickPlaceTaskActor::LoadPriorRowsForAccumulation()
{
	AccumulatedPriorRows.Reset();

	const FString FilePath = GetMotionCsvPath();

	TArray<FString> Existing;
	if (!FFileHelper::LoadFileToStringArray(Existing, *FilePath) || Existing.Num() < 2)
	{
		return; // 파일 없음 또는 데이터 없음 — 새로 시작 (조용히, 첫 누적이므로 정상)
	}

	// **헤더가 현재와 다르면 섞지 않는다.** 컬럼 스키마(예: tau 추가로 25→31)가 바뀐 파일을 이어붙이면
	// 재생/분석이 조용히 틀어진다 — D-02의 "컬럼 이름 계약"과 같은 이유다.
	const FString CurrentHeader = (CsvRows.Num() > 0) ? CsvRows[0] : FString();
	if (Existing[0].TrimStartAndEnd() != CurrentHeader.TrimStartAndEnd())
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[APickPlaceTaskActor] 누적 대상 '%s'의 헤더가 현재 헤더와 달라 이어붙이지 않고 새로 시작합니다 (컬럼 스키마 변경)."),
			*FilePath);
		return;
	}

	// 헤더(0행)를 뺀 나머지가 과거 데이터다.
	AccumulatedPriorRows.Reserve(Existing.Num() - 1);
	for (int32 i = 1; i < Existing.Num(); ++i)
	{
		AccumulatedPriorRows.Add(Existing[i]);
	}

	UE_LOG(LogRobotSim, Log,
		TEXT("[APickPlaceTaskActor] 누적 모드 — 기존 %d행 뒤에 이번 사이클을 이어붙입니다: %s"),
		AccumulatedPriorRows.Num(), *FilePath);
}

void APickPlaceTaskActor::WriteCsvToDisk()
{
	// 헤더 한 줄뿐이면 이번 사이클 샘플이 없다는 뜻이므로 빈 파일을 만들지 않는다.
	// (누적 모드에서 과거 행만 있고 새 샘플이 0인 경우도 다시 쓸 이유가 없다.)
	if (!bEnableCsvLogging || CsvRows.Num() <= 1)
	{
		return;
	}

	const FString FilePath = GetMotionCsvPath();
	const FString Directory = FPaths::GetPath(FilePath);

	// SaveStringArrayToFile은 디렉터리를 만들어 주지 않는다 — 첫 실행에서 조용히 실패하지 않도록 먼저 만든다.
	IFileManager::Get().MakeDirectory(*Directory, /*Tree=*/true);

	// 출력 = [헤더] + [과거 누적 행] + [이번 사이클 행]. 누적을 끄면 AccumulatedPriorRows가 비어
	// 기존 동작(이번 사이클만 덮어쓰기)과 정확히 같다.
	//
	// **디스크 append가 아니라 전체 덮어쓰기**를 쓰는 이유: append는 ForceUTF8이 붙이는 BOM이 파일
	// 중간에 박혀 파서를 깨뜨린다. 매번 전체를 쓰면 BOM은 항상 맨 앞 한 번뿐이고, 몇 번을 flush해도
	// 파일이 처음부터 끝까지 온전하다 (덮어쓰기라 중복도 없다).
	TArray<FString> Output;
	Output.Reserve(1 + AccumulatedPriorRows.Num() + (CsvRows.Num() - 1));
	Output.Add(CsvRows[0]); // 헤더
	Output.Append(AccumulatedPriorRows);
	for (int32 i = 1; i < CsvRows.Num(); ++i)
	{
		Output.Add(CsvRows[i]);
	}

	// UTF-8(BOM 포함) 강제 — 기본값(AutoDetect)은 비ASCII가 하나라도 있으면 UTF-16으로 써서
	// pandas/awk가 못 읽는다. BOM이 있어야 Excel도 UTF-8로 인식한다.
	if (FFileHelper::SaveStringArrayToFile(Output, *FilePath, FFileHelper::EEncodingOptions::ForceUTF8))
	{
		UE_LOG(LogRobotSim, Log,
			TEXT("[APickPlaceTaskActor] CSV %d행 기록%s: %s"),
			Output.Num() - 1,
			AccumulatedPriorRows.Num() > 0 ? TEXT(" (누적)") : TEXT(""), *FilePath);
	}
	else
	{
		UE_LOG(LogRobotSim, Warning, TEXT("[APickPlaceTaskActor] CSV 기록 실패: %s (경로/권한 확인)"), *FilePath);
	}

	// CsvRows는 비우지 않는다 — 사이클 도중 여러 번 flush해도 매번 전체를 다시 쓰므로 온전하다.
}

const TCHAR* APickPlaceTaskActor::PhaseToString(EPickPlacePhase InPhase)
{
	switch (InPhase)
	{
	case EPickPlacePhase::Idle:            return TEXT("Idle");
	case EPickPlacePhase::ToPickApproach:  return TEXT("ToPickApproach");
	case EPickPlacePhase::ToPick:          return TEXT("ToPick");
	case EPickPlacePhase::Grasp:           return TEXT("Grasp");
	case EPickPlacePhase::ToLift:          return TEXT("ToLift");
	case EPickPlacePhase::ToPlaceApproach: return TEXT("ToPlaceApproach");
	case EPickPlacePhase::ToPlace:         return TEXT("ToPlace");
	case EPickPlacePhase::Release:         return TEXT("Release");
	case EPickPlacePhase::ToRetreat:       return TEXT("ToRetreat");
	case EPickPlacePhase::Done:            return TEXT("Done");
	case EPickPlacePhase::Aborted:         return TEXT("Aborted");
	default:                               return TEXT("Unknown");
	}
}

#pragma endregion
