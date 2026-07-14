// Fill out your copyright notice in the Description page of Project Settings.

#include "Robot/Serial6DoFRobotActor.h"

#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Robot/RobotSimLog.h"
#include "UObject/ConstructorHelpers.h"

namespace Serial6DoFRobotActorConstants
{
	/** 수학-비주얼 일관성 검증 허용 오차: 위치 (cm) */
	constexpr double VisualMathPosToleranceCm = 0.1;

	/** 수학-비주얼 일관성 검증 허용 오차: 각도 (radian) */
	constexpr double VisualMathAngToleranceRad = 1e-3;

	/** 디버그 좌표축 크기 (cm) */
	constexpr float DebugAxisLength = 15.0f;

	/** 디버그 좌표축 선 두께 */
	constexpr float DebugAxisThickness = 0.8f;
}

ASerial6DoFRobotActor::ASerial6DoFRobotActor()
{
	PrimaryActorTick.bCanEverTick = true;

	// 기구학 파라미터는 수학 모델에서만 정의한다. 아래 컴포넌트 계층은
	// 이 모델을 미러링할 뿐이며, 오프셋/축 값을 여기서 다시 쓰지 않는다.
	Model = FSerial6DoFModel::CreateDefault();

	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);

	// J0~J5 관절 프레임을 부모-자식 체인으로 생성. 상대 위치 = 모델의 LinkOffsets.
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		JointComponents[i] = CreateDefaultSubobject<USceneComponent>(*FString::Printf(TEXT("Joint%d"), i));
		JointComponents[i]->SetupAttachment(i == 0 ? Root : JointComponents[i - 1].Get());
		JointComponents[i]->SetRelativeLocation(Model.LinkOffsets[i]);
	}

	// End Effector 프레임: J5의 자식, 상대 변환 = 모델의 ToolOffset.
	ToolTipComponent = CreateDefaultSubobject<USceneComponent>(TEXT("ToolTip"));
	ToolTipComponent->SetupAttachment(JointComponents[FSerial6DoFModel::NumJoints - 1].Get());
	ToolTipComponent->SetRelativeTransform(Model.ToolOffset);

	// 링크 디버그 메시: 엔진 기본 도형 (100cm 기준, 실린더 축 = 로컬 Z).
	// 각 메시는 다음 관절의 형제로 붙으므로 관절 프레임(FK)에는 영향이 없다.
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMeshFinder(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderMeshFinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereMeshFinder(TEXT("/Engine/BasicShapes/Sphere.Sphere"));

	if (CubeMeshFinder.Succeeded() && CylinderMeshFinder.Succeeded() && SphereMeshFinder.Succeeded())
	{
		UStaticMesh* Cube = CubeMeshFinder.Object;
		UStaticMesh* Cylinder = CylinderMeshFinder.Object;
		UStaticMesh* Sphere = SphereMeshFinder.Object;

		// +X 방향 링크는 실린더 로컬 +Z를 부모 +X로 눕히기 위해 Pitch -90도를 준다.
		const FRotator AlignZToX(-90.0, 0.0, 0.0);

		CreateLinkMesh(Root, TEXT("Mesh_BaseColumn"), Cube, FVector(0.0, 0.0, 20.0), FRotator::ZeroRotator, FVector(0.45, 0.45, 0.40));
		CreateLinkMesh(JointComponents[0].Get(), TEXT("Mesh_Shoulder"), Cylinder, FVector(0.0, 0.0, 10.0), FRotator::ZeroRotator, FVector(0.25, 0.25, 0.20));
		CreateLinkMesh(JointComponents[1].Get(), TEXT("Mesh_UpperArm"), Cylinder, FVector(0.0, 0.0, 30.0), FRotator::ZeroRotator, FVector(0.18, 0.18, 0.60));
		CreateLinkMesh(JointComponents[2].Get(), TEXT("Mesh_Forearm"), Cylinder, FVector(30.0, 0.0, 0.0), AlignZToX, FVector(0.15, 0.15, 0.60));
		CreateLinkMesh(JointComponents[3].Get(), TEXT("Mesh_Wrist"), Cylinder, FVector(10.0, 0.0, 0.0), AlignZToX, FVector(0.12, 0.12, 0.20));
		CreateLinkMesh(JointComponents[4].Get(), TEXT("Mesh_Flange"), Cylinder, FVector(7.5, 0.0, 0.0), AlignZToX, FVector(0.10, 0.10, 0.15));
		CreateLinkMesh(JointComponents[5].Get(), TEXT("Mesh_Tool"), Cylinder, FVector(5.0, 0.0, 0.0), AlignZToX, FVector(0.08, 0.08, 0.10));
		CreateLinkMesh(ToolTipComponent.Get(), TEXT("Mesh_ToolTipMarker"), Sphere, FVector::ZeroVector, FRotator::ZeroRotator, FVector(0.06, 0.06, 0.06));
	}
	else
	{
		UE_LOG(LogRobotSim, Warning, TEXT("[ASerial6DoFRobotActor] 엔진 기본 도형 메시를 찾지 못해 링크 메시 없이 생성합니다."));
	}
}

void ASerial6DoFRobotActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	// 에디터 배치/디테일 편집 시 각도를 즉시 반영한다.
	ApplyAnglesFromEditor();
}

#if WITH_EDITOR
void ASerial6DoFRobotActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	// 고정 배열 UPROPERTY는 변경된 원소 인덱스를 알려주지 않으므로 6개 전부 재적용한다.
	if (PropertyChangedEvent.GetPropertyName() == GET_MEMBER_NAME_CHECKED(ASerial6DoFRobotActor, JointAnglesDeg))
	{
		ApplyAnglesFromEditor();
	}
}
#endif

void ASerial6DoFRobotActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// PIE 중 디테일 패널 편집도 반영되도록 매 틱 적용한다 (클램프 포함, 저비용/멱등).
	ApplyAnglesFromEditor();

	if (bDrawDebugFrames)
	{
		DrawDebugJointFrames();
	}

	// EE 자세 주기 로그는 게임 월드(PIE/게임)에서만 출력해 에디터 로그 스팸을 막는다.
	const UWorld* World = GetWorld();
	if (PoseLogIntervalSeconds > 0.0f && World && World->IsGameWorld())
	{
		TimeSinceLastPoseLog += DeltaSeconds;
		if (TimeSinceLastPoseLog >= PoseLogIntervalSeconds)
		{
			TimeSinceLastPoseLog = 0.0;
			LogEndEffectorPose();
		}
	}
}

void ASerial6DoFRobotActor::SetJointAngles(const FRobot6DJointState& NewState)
{
	CurrentState = Model.ClampToLimits(NewState);
	ApplyJointState();
}

FRobot6DPose ASerial6DoFRobotActor::GetEndEffectorPose() const
{
	// 모델은 액터 공간(BaseTransform=Identity)에서 FK를 계산하므로 액터 트랜스폼을 합성해 월드로 올린다.
	const FTransform EEWorld = Model.ComputeEndEffectorTransform(CurrentState) * GetActorTransform();

	FRobot6DPose Pose;
	Pose.PositionCm = EEWorld.GetLocation();
	Pose.Orientation = EEWorld.GetRotation();
	return Pose;
}

void ASerial6DoFRobotActor::ApplyJointState()
{
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		if (JointComponents[i])
		{
			// FQuat 오버로드 사용: FRotator 경유 변환은 +-180도 초과 각도의 winding을 잃는다.
			JointComponents[i]->SetRelativeRotation(FQuat(Model.JointAxes[i], CurrentState.Q[i]));
		}
	}

	CheckVisualMatchesMath();
}

void ASerial6DoFRobotActor::LogEndEffectorPose()
{
	const FRobot6DPose EEPose = GetEndEffectorPose();
	const FRotator EulerDeg = EEPose.Orientation.Rotator();

	UE_LOG(LogRobotSim, Log,
		TEXT("[ASerial6DoFRobotActor] EE 위치=(%.2f, %.2f, %.2f)cm, 회전(Roll,Pitch,Yaw)=(%.2f, %.2f, %.2f)도, 쿼터니언=(%.4f, %.4f, %.4f, %.4f)"),
		EEPose.PositionCm.X, EEPose.PositionCm.Y, EEPose.PositionCm.Z,
		EulerDeg.Roll, EulerDeg.Pitch, EulerDeg.Yaw,
		EEPose.Orientation.X, EEPose.Orientation.Y, EEPose.Orientation.Z, EEPose.Orientation.W);

	CheckVisualMatchesMath();
}

void ASerial6DoFRobotActor::ResetJointAngles()
{
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		JointAnglesDeg[i] = 0.0;
	}
	ApplyAnglesFromEditor();

	UE_LOG(LogRobotSim, Log, TEXT("[ASerial6DoFRobotActor] 모든 관절 각도를 0으로 초기화했습니다."));
}

void ASerial6DoFRobotActor::ApplyAnglesFromEditor()
{
	// 도 단위는 이 함수의 입구에서만 radian으로 변환한다 (단위 규약의 경계).
	FRobot6DJointState NewState;
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		NewState.Q[i] = FMath::DegreesToRadians(JointAnglesDeg[i]);
	}

	SetJointAngles(NewState);

	// 클램프 결과를 도로 되돌려 써서 디테일 패널이 실제 적용 값을 보여주게 한다.
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		JointAnglesDeg[i] = FMath::RadiansToDegrees(CurrentState.Q[i]);
	}
}

void ASerial6DoFRobotActor::CheckVisualMatchesMath() const
{
	using namespace Serial6DoFRobotActorConstants;

	// CDO이거나 컴포넌트가 아직 등록되지 않은 시점(생성자 직후 등)에는 검증하지 않는다.
	if (IsTemplate() || !ToolTipComponent || !ToolTipComponent->IsRegistered())
	{
		return;
	}

	const FTransform VisualEE = ToolTipComponent->GetComponentTransform();
	const FTransform MathEE = Model.ComputeEndEffectorTransform(CurrentState) * GetActorTransform();

	const double PosError = FVector::Distance(VisualEE.GetLocation(), MathEE.GetLocation());
	const double AngError = VisualEE.GetRotation().AngularDistance(MathEE.GetRotation());

	if (PosError > VisualMathPosToleranceCm || AngError > VisualMathAngToleranceRad)
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[ASerial6DoFRobotActor] 수학 FK와 컴포넌트 트랜스폼 불일치: 위치 오차 %.4fcm, 각도 오차 %.6frad (비주얼=(%.2f, %.2f, %.2f), 수학=(%.2f, %.2f, %.2f))"),
			PosError, AngError,
			VisualEE.GetLocation().X, VisualEE.GetLocation().Y, VisualEE.GetLocation().Z,
			MathEE.GetLocation().X, MathEE.GetLocation().Y, MathEE.GetLocation().Z);
	}
}

void ASerial6DoFRobotActor::DrawDebugJointFrames() const
{
	using namespace Serial6DoFRobotActorConstants;

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	// 관절 프레임은 컴포넌트 트랜스폼에서, EE 프레임은 수학 FK에서 그린다.
	// 둘의 소스가 다르므로 미러링이 깨지면 화면에서도 축이 어긋나 보인다.
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		if (JointComponents[i])
		{
			const FTransform JointTransform = JointComponents[i]->GetComponentTransform();
			DrawDebugCoordinateSystem(World, JointTransform.GetLocation(), JointTransform.Rotator(),
				DebugAxisLength, false, -1.0f, 0, DebugAxisThickness);
		}
	}

	const FTransform MathEE = Model.ComputeEndEffectorTransform(CurrentState) * GetActorTransform();
	DrawDebugCoordinateSystem(World, MathEE.GetLocation(), MathEE.Rotator(),
		DebugAxisLength * 1.5f, false, -1.0f, 0, DebugAxisThickness * 1.5f);
}

UStaticMeshComponent* ASerial6DoFRobotActor::CreateLinkMesh(
	USceneComponent* Parent, const FName& Name, UStaticMesh* Mesh,
	const FVector& RelativeLocation, const FRotator& RelativeRotation, const FVector& RelativeScale)
{
	UStaticMeshComponent* MeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(Name);
	MeshComponent->SetupAttachment(Parent);
	MeshComponent->SetStaticMesh(Mesh);
	MeshComponent->SetRelativeLocation(RelativeLocation);
	MeshComponent->SetRelativeRotation(RelativeRotation);
	MeshComponent->SetRelativeScale3D(RelativeScale);

	// 디버그 비주얼 전용: 충돌/물리에 관여하지 않는다.
	MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	MeshComponent->SetGenerateOverlapEvents(false);

	LinkMeshComponents.Add(MeshComponent);
	return MeshComponent;
}
