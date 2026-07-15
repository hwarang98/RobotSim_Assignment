// Fill out your copyright notice in the Description page of Project Settings.

#include "Robot/PickPlaceTaskActor.h"

#include "Components/PoseableMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/Engine.h"
#include "Engine/SkinnedAsset.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Robot/RobotDlsIK.h"
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

	if (bSpawnBoxesOnBeginPlay)
	{
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
	// 사이클이 Done에 도달하기 전에 PIE를 멈춰도 그때까지의 샘플은 남긴다.
	WriteCsvToDisk();

	Super::EndPlay(EndPlayReason);
}

void APickPlaceTaskActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

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

	// Idle에서는 로봇이 스스로를 소유한다 (STEP A 동작 그대로). 아래 SetJointAngles로 개입하지 않는다.
	if (Phase == EPickPlacePhase::Idle)
	{
		return;
	}

	// Done/Aborted에서는 FSM을 더 전진시키지 않지만 SetJointAngles는 계속해야 한다.
	// 멈추면 로봇 Tick의 JointAnglesDeg 되쓰기가 다시 이겨서 팔이 홈 자세로 튕겨 돌아간다.
	if (Phase != EPickPlacePhase::Done && Phase != EPickPlacePhase::Aborted)
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
	TArray<FVector> SlotWorld;
	if (!BuildSlotsOnSurface(SourceSurfaceActor, NumBoxesToSpawn, SourceSlotStrideCm,
			SourceSlotOffsetCm, /*HeightAboveSurfaceCm=*/0.0, SlotWorld))
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[APickPlaceTaskActor] SourceSurfaceActor가 없거나 바운드가 없어 박스를 스폰하지 못했습니다 — ")
			TEXT("팔레트 액터를 레벨에 배치하고 할당하세요."));
		return;
	}

	UClass* ClassToSpawn = BoxClass ? BoxClass.Get() : APickPlaceBoxActor::StaticClass();

	for (int32 i = 0; i < SlotWorld.Num(); ++i)
	{
		FActorSpawnParameters SpawnParams;
		SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
		SpawnParams.Owner = this;

		// 먼저 슬롯 위치에 스폰한다. 실제 바운드는 스폰 후에만 알 수 있고, 그 값이 있어야
		// "상판 위에 정확히" 앉힐 수 있다. 회전은 출발지 액터를 따라간다(팔레트 위에 정렬돼 보이도록).
		const FTransform SpawnTransform(SourceSurfaceActor->GetActorRotation(), SlotWorld[i]);
		APickPlaceBoxActor* Box = World->SpawnActor<APickPlaceBoxActor>(ClassToSpawn, SpawnTransform, SpawnParams);

		if (!Box)
		{
			UE_LOG(LogRobotSim, Warning, TEXT("[APickPlaceTaskActor] 박스 %d 스폰 실패 (BoxClass 확인)"), i);
			continue;
		}

		// 박스 바닥이 상판에 정확히 닿게 앉힌다. 뜬 채로 스폰되면 낙하 중에 파지 지점이 스냅샷돼
		// 엉뚱한 곳을 집게 되고, 옆 박스를 밀어내기도 한다.
		//
		// 액터 위치가 아니라 **바운드 바닥면**을 기준으로 보정한다. 프롭 메시는 피벗이 바닥이나
		// 구석에 있는 경우가 흔해서 "액터 위치 = 기하학적 중심"이 성립하지 않는다 —
		// 그 가정 때문에 박스가 절반 높이만큼 뜬 채 스폰돼 떨어졌다.
		const double DropZ = SlotWorld[i].Z - Box->GetBoundsBottomZWorld();
		Box->AddActorWorldOffset(FVector(0.0, 0.0, DropZ),
			/*bSweep=*/false, /*OutSweepHitResult=*/nullptr, ETeleportType::TeleportPhysics);

		Boxes.Add(Box);

		const FVector GraspLocal = Robot->GetActorTransform().InverseTransformPosition(Box->GetGraspPointWorld());
		UE_LOG(LogRobotSim, Log,
			TEXT("[APickPlaceTaskActor] 박스 %d 스폰 — 월드 (%.1f, %.1f, %.1f)cm, 높이 %.1fcm, ")
			TEXT("파지점(윗면 중심) 로봇공간 (%.1f, %.1f, %.1f)cm [XY반경 %.1fcm]"),
			i, Box->GetActorLocation().X, Box->GetActorLocation().Y, Box->GetActorLocation().Z, Box->GetHeightCm(),
			GraspLocal.X, GraspLocal.Y, GraspLocal.Z, FVector2D(GraspLocal.X, GraspLocal.Y).Size());
	}

	UE_LOG(LogRobotSim, Log,
		TEXT("[APickPlaceTaskActor] 출발지 '%s' 상판에 박스 %d개 스폰 — 간격 (%.0f, %.0f, %.0f)cm"),
		*SourceSurfaceActor->GetName(), Boxes.Num(),
		SourceSlotStrideCm.X, SourceSlotStrideCm.Y, SourceSlotStrideCm.Z);

	// 간격이 박스 크기보다 좁으면 스폰 순간 서로 겹쳐 물리가 밀어낸다. 그러면 박스가 파지 지점
	// 스냅샷에서 벗어나 흡착판이 빈 곳을 집는데, 겉보기엔 "IK가 이상하다"로 보여 원인을 찾기 어렵다.
	// 조용히 틀리는 종류라 여기서 미리 말한다.
	if (Boxes.Num() >= 2 && Boxes[0])
	{
		// 간격 방향으로 박스가 차지하는 폭. 바운드는 월드 AABB이므로 방향 성분의 절대값 합으로 근사한다.
		const FVector Extent = Boxes[0]->GetBoundsExtentCm();
		const FVector StrideDir = SourceSlotStrideCm.GetSafeNormal();
		const double BoxWidthAlongStride =
			2.0 * (FMath::Abs(Extent.X * StrideDir.X) + FMath::Abs(Extent.Y * StrideDir.Y) + FMath::Abs(Extent.Z * StrideDir.Z));
		const double StrideLen = SourceSlotStrideCm.Size();

		if (StrideLen < BoxWidthAlongStride)
		{
			UE_LOG(LogRobotSim, Warning,
				TEXT("[APickPlaceTaskActor] **SourceSlotStrideCm이 너무 좁습니다** — 간격 %.1fcm < 박스 폭 %.1fcm. ")
				TEXT("스폰 순간 박스들이 겹쳐 물리가 서로 밀어내고, 그러면 파지 지점 스냅샷이 어긋나 흡착판이 빈 곳을 집습니다. ")
				TEXT("간격을 %.0fcm 이상으로 키우세요."),
				StrideLen, BoxWidthAlongStride, FMath::CeilToDouble(BoxWidthAlongStride * 1.1));
		}
	}
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

bool APickPlaceTaskActor::BuildSlotsOnSurface(
	const AActor* SurfaceActor, int32 SlotCount, const FVector& StrideCm, const FVector& OffsetCm,
	double HeightAboveSurfaceCm, TArray<FVector>& OutSlotWorld) const
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

	for (int32 i = 0; i < SlotCount; ++i)
	{
		// 행을 액터 중심에 맞춰 정렬한다: i - (N-1)/2 → 짝수 개면 중심 양옆, 홀수 개면 가운데가 중심.
		// 이렇게 하면 박스 개수를 바꿔도 액터를 다시 옮길 필요가 없다.
		const double Centered = static_cast<double>(i) - (static_cast<double>(SlotCount) - 1.0) * 0.5;

		// 액터 로컬 → 월드. 액터를 회전시키면 슬롯 행도 같이 돈다.
		FVector SlotWorld = SurfaceToWorld.TransformPosition(OffsetCm + StrideCm * Centered);

		// XY는 액터 로컬을 따르되 Z만 상판 기준으로 덮어쓴다 — 액터가 조금 기울어져 있어도
		// 박스는 중력 기준 수평으로 놓여야 하기 때문이다.
		SlotWorld.Z = SurfaceTopWorldZ + HeightAboveSurfaceCm;

		OutSlotWorld.Add(SlotWorld);
	}

	return true;
}

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
	if (!BuildSlotsOnSurface(DestinationSurfaceActor, Boxes.Num(), DestinationSlotStrideCm,
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

	CsvRows.Reset();
	if (bEnableCsvLogging)
	{
		// 여기 있는 값은 전부 ActiveState(이 액터가 소유한 관절 상태)에서 직접 계산한다 — 로봇/메시
		// 컴포넌트를 읽지 않는다. 그래야 프레임 중 어느 시점에 기록하든 값이 일관된다.
		//
		// (한때 흡착판/박스 월드 좌표도 기록했는데, 그건 로봇 컴포넌트를 읽는 값이라 로봇 Tick이
		//  JointAnglesDeg로 되쓴 홈 자세를 찍었다. 화면은 멀쩡한데 CSV만 틀리는 함정이라 걷어냈다.)
		CsvRows.Add(TEXT("time_s,phase,box_index,")
			TEXT("q0_deg,q1_deg,q2_deg,q3_deg,q4_deg,q5_deg,")
			TEXT("qd0_degps,qd1_degps,qd2_degps,qd3_degps,qd4_degps,qd5_degps,")
			TEXT("ee_x_cm,ee_y_cm,ee_z_cm,ee_roll_deg,ee_pitch_deg,ee_yaw_deg,")
			TEXT("target_x_cm,target_y_cm,target_z_cm,box_held"));
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

	Phase = EPickPlacePhase::Idle;
	CurrentBoxIndex = INDEX_NONE;
	PhaseElapsedSec = 0.0;
	PhaseDurationSec = 0.0;
	SimTimeSec = 0.0;
	TimeAccumulatorSec = 0.0;

	UE_LOG(LogRobotSim, Log,
		TEXT("[APickPlaceTaskActor] 사이클을 Idle로 리셋했습니다 — 관절 소유권을 로봇에 반환하므로 팔이 JointAnglesDeg 자세로 돌아갑니다."));
}

void APickPlaceTaskActor::FlushCsvNow()
{
	WriteCsvToDisk();
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

	APickPlaceBoxActor* Box = Boxes.IsValidIndex(CurrentBoxIndex) ? Boxes[CurrentBoxIndex].Get() : nullptr;

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
		if (!BeginTrajectoryTo(GetPlacePoseLocal(CurrentBoxIndex, /*bApproach=*/true)))
		{
			return;
		}
		break;

	case EPickPlacePhase::ToPlace:
		if (!BeginTrajectoryTo(GetPlacePoseLocal(CurrentBoxIndex, /*bApproach=*/false)))
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
		if (!BeginTrajectoryTo(GetPlacePoseLocal(CurrentBoxIndex, /*bApproach=*/true)))
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

	const FVector TargetLocal = CurrentTargetLocal.GetLocation();
	UE_LOG(LogRobotSim, Log,
		TEXT("[APickPlaceTaskActor] 단계 → %s (박스 %d, 소요 %.2fs, 목표 로봇공간 (%.1f, %.1f, %.1f)cm)"),
		PhaseToString(Phase), CurrentBoxIndex, PhaseDurationSec, TargetLocal.X, TargetLocal.Y, TargetLocal.Z);
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
		// 사이클의 유일한 분기: 남은 박스가 있으면 다음 pick으로 돌아가고, 없으면 종료한다.
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

	for (int32 Iter = 0; Iter < MaxVisualIterations; ++Iter)
	{
		const FRobotDlsIKResult IKResult =
			FRobotDlsIK::SolveDlsIK(Robot->GetModel(), ActiveState, MathTargetLocal, Options);

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

#pragma region APickPlaceTaskActor_Logging

void APickPlaceTaskActor::RecordCsvRow()
{
	if (!bEnableCsvLogging || !Robot)
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
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		const double VelDegPerSec = FMath::RadiansToDegrees(ActiveState.Q[i] - PreviousState.Q[i]) / FixedTimeStepSec;
		Row += FString::Printf(TEXT(",%.5f"), VelDegPerSec);
	}

	Row += FString::Printf(TEXT(",%.4f,%.4f,%.4f,%.4f,%.4f,%.4f"),
		ToolLocation.X, ToolLocation.Y, ToolLocation.Z,
		ToolRotation.Roll, ToolRotation.Pitch, ToolRotation.Yaw);

	const FVector TargetLocation = TargetWorld.GetLocation();
	Row += FString::Printf(TEXT(",%.4f,%.4f,%.4f,%d"),
		TargetLocation.X, TargetLocation.Y, TargetLocation.Z, HeldBox ? 1 : 0);

	CsvRows.Add(MoveTemp(Row));
}

void APickPlaceTaskActor::WriteCsvToDisk()
{
	// 헤더 한 줄뿐이면 실제 샘플이 없다는 뜻이므로 빈 파일을 만들지 않는다.
	if (!bEnableCsvLogging || CsvRows.Num() <= 1)
	{
		return;
	}

	const FString Directory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("RobotSim"));
	const FString FilePath = FPaths::Combine(Directory, CsvFileName);

	// SaveStringArrayToFile은 디렉터리를 만들어 주지 않는다 — 첫 실행에서 조용히 실패하지 않도록 먼저 만든다.
	IFileManager::Get().MakeDirectory(*Directory, /*Tree=*/true);

	if (FFileHelper::SaveStringArrayToFile(CsvRows, *FilePath))
	{
		UE_LOG(LogRobotSim, Log, TEXT("[APickPlaceTaskActor] CSV %d행 기록: %s"), CsvRows.Num() - 1, *FilePath);
	}
	else
	{
		UE_LOG(LogRobotSim, Warning, TEXT("[APickPlaceTaskActor] CSV 기록 실패: %s (경로/권한 확인)"), *FilePath);
	}

	// 헤더만 남겨 재기록 시 중복 append를 막는다 (FlushCsvNow를 여러 번 눌러도 안전).
	CsvRows.SetNum(1);
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
