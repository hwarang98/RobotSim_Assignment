// Fill out your copyright notice in the Description page of Project Settings.

#include "Robot/Serial6DoFRobotActor.h"

#include "Components/PoseableMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/SkeletalMesh.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Robot/EndEffectorTargetActor.h"
#include "Robot/RobotDlsIK.h"
#include "Robot/RobotConfig.h"
#include "Robot/RobotJacobian.h"
#include "Robot/RobotPoseError.h"
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

		// 비주얼 슬롯 i와 겹침 숨김 처리할 디버그 도형을 명시적으로 매핑해 둔다
		// (ToolTipMarker는 대응 슬롯 없음 → bShowDebugLinks만 따름).
		DebugMeshForVisualSlot[0] = CreateLinkMesh(Root, TEXT("Mesh_BaseColumn"), Cube, FVector(0.0, 0.0, 20.0), FRotator::ZeroRotator, FVector(0.45, 0.45, 0.40));
		DebugMeshForVisualSlot[1] = CreateLinkMesh(JointComponents[0].Get(), TEXT("Mesh_Shoulder"), Cylinder, FVector(0.0, 0.0, 10.0), FRotator::ZeroRotator, FVector(0.25, 0.25, 0.20));
		DebugMeshForVisualSlot[2] = CreateLinkMesh(JointComponents[1].Get(), TEXT("Mesh_UpperArm"), Cylinder, FVector(0.0, 0.0, 30.0), FRotator::ZeroRotator, FVector(0.18, 0.18, 0.60));
		DebugMeshForVisualSlot[3] = CreateLinkMesh(JointComponents[2].Get(), TEXT("Mesh_Forearm"), Cylinder, FVector(30.0, 0.0, 0.0), AlignZToX, FVector(0.15, 0.15, 0.60));
		DebugMeshForVisualSlot[4] = CreateLinkMesh(JointComponents[3].Get(), TEXT("Mesh_Wrist"), Cylinder, FVector(10.0, 0.0, 0.0), AlignZToX, FVector(0.12, 0.12, 0.20));
		DebugMeshForVisualSlot[5] = CreateLinkMesh(JointComponents[4].Get(), TEXT("Mesh_Flange"), Cylinder, FVector(7.5, 0.0, 0.0), AlignZToX, FVector(0.10, 0.10, 0.15));
		DebugMeshForVisualSlot[6] = CreateLinkMesh(JointComponents[5].Get(), TEXT("Mesh_Tool"), Cylinder, FVector(5.0, 0.0, 0.0), AlignZToX, FVector(0.08, 0.08, 0.10));
		CreateLinkMesh(ToolTipComponent.Get(), TEXT("Mesh_ToolTipMarker"), Sphere, FVector::ZeroVector, FRotator::ZeroRotator, FVector(0.06, 0.06, 0.06));
	}
	else
	{
		UE_LOG(LogRobotSim, Warning, TEXT("[ASerial6DoFRobotActor] 엔진 기본 도형 메시를 찾지 못해 링크 메시 없이 생성합니다."));
	}

	// 링크별 사용자 할당 StaticMesh 슬롯: [0]=Base(Root), [i]=LinkI(J(i-1)).
	// 컴포넌트는 항상 만들어 두고 메시는 비워 둔다 (미할당 슬롯은 아무것도 그리지 않음).
	// 관절 프레임 체인(Root→J0→…→J5→ToolTip) 밖의 자식이므로 FK에는 영향이 없다.
	LinkVisuals.SetNum(NumVisualLinks);
	for (int32 i = 0; i < NumVisualLinks; ++i)
	{
		const FString SlotName = (i == 0) ? TEXT("Base") : FString::Printf(TEXT("Link%d_J%d"), i, i - 1);
		LinkVisuals[i].LinkName = FName(*SlotName);

		VisualMeshComponents[i] = CreateDefaultSubobject<UStaticMeshComponent>(*FString::Printf(TEXT("VisualMesh_%s"), *SlotName));
		VisualMeshComponents[i]->SetupAttachment(i == 0 ? Root : JointComponents[i - 1].Get());

		// 비주얼 전용: 충돌/물리에 관여하지 않는다.
		VisualMeshComponents[i]->SetCollisionEnabled(ECollisionEnabled::NoCollision);
		VisualMeshComponents[i]->SetGenerateOverlapEvents(false);
	}

	// SkeletalMesh 시각화: AnimBP 없이 본 트랜스폼을 직접 쓰기 위해 PoseableMesh를 쓴다.
	// Root 직속이며 관절 프레임 체인과 독립 — 본은 수학 FK 결과를 따라가기만 한다.
	SkeletalVisualComponent = CreateDefaultSubobject<UPoseableMeshComponent>(TEXT("SkeletalVisualMesh"));
	SkeletalVisualComponent->SetupAttachment(Root);
	SkeletalVisualComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	SkeletalVisualComponent->SetGenerateOverlapEvents(false);
	SkeletalVisualComponent->SetVisibility(false); // 에셋/본 매핑이 유효해질 때 ApplySkeletalMeshVisual()이 켠다

	// 본 매핑과 SkeletalMesh 에셋은 이제 URobotConfig가 소유한다 (기본값은 URobotConfig 생성자 참조).
}

void ASerial6DoFRobotActor::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

	// RobotConfig(있으면)로 모델을 재구성하고 오프셋을 재미러링한 뒤 각도를 반영한다.
	// 이어서 비주얼 레이어를 갱신한다.
	RefreshFromConfig();
	ApplyLinkVisuals();
	ApplySkeletalMeshVisual();
}

void ASerial6DoFRobotActor::BeginPlay()
{
	Super::BeginPlay();

	// PIE/게임 시작 시 컴포넌트가 완전히 등록된 상태에서 비주얼을 다시 초기화한다.
	// OnConstruction만으로는 PIE에서 PoseableMesh 포즈 버퍼(BoneSpaceTransforms)가 준비되지 않아
	// SyncSkeletalPoseToMath가 매 틱 조기 return → SkeletalMesh가 bind pose에 정지하는 문제가 있다.
	// 여기서 ApplySkeletalMeshVisual을 다시 호출해 포즈 버퍼를 확실히 할당한다(리타겟 수식은 불변).
	RefreshFromConfig();
	ApplyLinkVisuals();
	ApplySkeletalMeshVisual();
}

#if WITH_EDITOR
void ASerial6DoFRobotActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	const FName MemberName = PropertyChangedEvent.GetMemberPropertyName();

	// RobotConfig 포인터를 바꾸면 모델(기구학)과 SkeletalMesh 시각화를 모두 재구성한다.
	// (에셋 '내부' 값 편집은 액터의 PostEditChangeProperty로 오지 않으므로, 그 경우엔
	//  액터를 다시 선택/이동해 OnConstruction을 트리거하면 반영된다.)
	if (MemberName == GET_MEMBER_NAME_CHECKED(ASerial6DoFRobotActor, RobotConfig))
	{
		RefreshFromConfig();
		ApplySkeletalMeshVisual();
	}

	// 고정 배열 UPROPERTY는 변경된 원소 인덱스를 알려주지 않으므로 6개 전부 재적용한다.
	if (PropertyChangedEvent.GetPropertyName() == GET_MEMBER_NAME_CHECKED(ASerial6DoFRobotActor, JointAnglesDeg))
	{
		ApplyAnglesFromEditor();
	}

	// 구조체 배열 내부 필드 편집 시 GetPropertyName()은 내부 필드명(Mesh 등)을 주므로
	// 소유 멤버(MemberProperty) 이름으로 판별한다.
	if (MemberName == GET_MEMBER_NAME_CHECKED(ASerial6DoFRobotActor, LinkVisuals)
		|| MemberName == GET_MEMBER_NAME_CHECKED(ASerial6DoFRobotActor, bShowStaticMeshes)
		|| MemberName == GET_MEMBER_NAME_CHECKED(ASerial6DoFRobotActor, bShowDebugLinks))
	{
		ApplyLinkVisuals();
	}

	if (MemberName == GET_MEMBER_NAME_CHECKED(ASerial6DoFRobotActor, bShowSkeletalMesh))
	{
		ApplySkeletalMeshVisual();
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

	// SkeletalMesh 본도 수학 FK 결과를 따라가게 한다 (읽기만 하므로 FK 불변).
	SyncSkeletalPoseToMath();

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

void ASerial6DoFRobotActor::LogCurrentEndEffectorPoseErrorToTarget()
{
	// 현재 EE 월드 자세를 수학 FK로 구해 Transform으로 재구성한다.
	const FRobot6DPose CurrentPose = GetEndEffectorPose();
	const FTransform CurrentTransform(CurrentPose.Orientation, CurrentPose.PositionCm);

	// 순수 수학 레이어로 6D pose error 계산 (target − current, world 프레임).
	const FRobot6DPoseError Error = FRobotPoseError::ComputePoseError(CurrentTransform, TargetEndEffectorWorld);

	UE_LOG(LogRobotSim, Log,
		TEXT("[ASerial6DoFRobotActor] Pose Error → 위치=(%.2f, %.2f, %.2f)cm |크기 %.2fcm|, ")
		TEXT("회전=(%.4f, %.4f, %.4f)rad |크기 %.4frad = %.2f도|"),
		Error.PositionErrorCm.X, Error.PositionErrorCm.Y, Error.PositionErrorCm.Z, Error.PositionErrorNorm(),
		Error.RotationErrorRad.X, Error.RotationErrorRad.Y, Error.RotationErrorRad.Z,
		Error.RotationErrorNorm(), FMath::RadiansToDegrees(Error.RotationErrorNorm()));
}

void ASerial6DoFRobotActor::LogCurrentNumericalJacobian()
{
	// 현재 관절 자세 기준 6×6 numerical Jacobian을 순수 수학 레이어로 계산한다 (관절 변경 없음).
	const FRobotJacobian6x6 J = FRobotJacobian::ComputeNumericalJacobian(GetModel(), GetJointState());

	// 행 = pose error [px,py,pz,rx,ry,rz], 열 = 관절 J0~J5.
	static const TCHAR* RowLabels[6] = { TEXT("px"), TEXT("py"), TEXT("pz"), TEXT("rx"), TEXT("ry"), TEXT("rz") };

	UE_LOG(LogRobotSim, Log,
		TEXT("[ASerial6DoFRobotActor] Numerical Jacobian 6×6 (행=pose error px/py/pz[cm/rad] rx/ry/rz[rad/rad], 열=J0~J5)"));
	UE_LOG(LogRobotSim, Log,
		TEXT("[ASerial6DoFRobotActor]         J0          J1          J2          J3          J4          J5"));
	for (int32 Row = 0; Row < 6; ++Row)
	{
		UE_LOG(LogRobotSim, Log,
			TEXT("[ASerial6DoFRobotActor] %s  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f"),
			RowLabels[Row],
			J.At(Row, 0), J.At(Row, 1), J.At(Row, 2), J.At(Row, 3), J.At(Row, 4), J.At(Row, 5));
	}
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

void ASerial6DoFRobotActor::SolveIKToTarget()
{
	// 현재 관절 상태를 초기값으로, target을 목표로 순수 수학 DLS IK를 푼다.
	// solver는 로봇 모델 공간(액터 로컬, BaseTransform=Identity 기준) target을 받는다.
	//  - EndEffectorTargetActor(있으면): GetActorTransform()은 월드이므로 로봇 액터 기준 상대 변환으로 내린다.
	//  - 없으면: 기존 TargetEndEffectorWorld는 이미 모델 공간 값이므로 그대로 사용(기존 동작 100% 보존).
	// 주의: TargetActor의 월드 트랜스폼을 그대로 넘기면 로봇이 이동/회전된 맵에서 target이 틀어진다.
	FTransform TargetModel;
	if (EndEffectorTargetActor)
	{
		const FTransform TargetWorld = EndEffectorTargetActor->GetActorTransform();
		TargetModel = TargetWorld.GetRelativeTransform(GetActorTransform());
	}
	else
	{
		TargetModel = TargetEndEffectorWorld;
	}

	// 기본 옵션 + 디테일 패널의 nullspace 토글만 반영한다(과도한 UI 확장 없음).
	FRobotDlsIKOptions Options;
	Options.bUseNullspaceJointLimitAvoidance = bUseNullspaceJointLimitAvoidance;
	const FRobotDlsIKResult IKResult =
		FRobotDlsIK::SolveDlsIK(GetModel(), GetJointState(), TargetModel, Options);

	UE_LOG(LogRobotSim, Log,
		TEXT("[ASerial6DoFRobotActor] IK %s: %d회 반복, 최종 위치오차 %.3fcm, 회전오차 %.4frad = %.2f도, nullspace=%s, max관절편차 %.3f"),
		IKResult.bConverged ? TEXT("수렴") : TEXT("미수렴"),
		IKResult.Iterations, IKResult.FinalPositionErrorCm,
		IKResult.FinalRotationErrorRad, FMath::RadiansToDegrees(IKResult.FinalRotationErrorRad),
		IKResult.bNullspaceUsed ? TEXT("사용") : TEXT("미사용"), IKResult.MaxAbsNormalizedJointDistance);

	// 결과 관절 해를 적용한다(SetJointAngles 내부에서 ClampToLimits + 비주얼 반영).
	SetJointAngles(IKResult.Solution);

	// 에디터 디테일 패널의 도 단위 표시를 실제 적용된 관절 값으로 동기화한다.
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		JointAnglesDeg[i] = FMath::RadiansToDegrees(CurrentState.Q[i]);
	}
}

void ASerial6DoFRobotActor::SpawnOrAlignTargetToCurrentEndEffector()
{
	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	// 현재 EE 월드 자세 (모델 FK × 액터 트랜스폼). Target Actor는 월드에 배치되므로 월드 값을 쓴다.
	const FTransform CurrentEEWorld = Model.ComputeEndEffectorTransform(CurrentState) * GetActorTransform();

	if (!EndEffectorTargetActor)
	{
		FActorSpawnParameters SpawnParams;
		SpawnParams.Owner = this;
		EndEffectorTargetActor =
			World->SpawnActor<AEndEffectorTargetActor>(AEndEffectorTargetActor::StaticClass(), CurrentEEWorld, SpawnParams);

		if (EndEffectorTargetActor)
		{
			UE_LOG(LogRobotSim, Log,
				TEXT("[ASerial6DoFRobotActor] End Effector Target Actor를 현재 EE 위치에 생성했습니다."));
		}
		else
		{
			UE_LOG(LogRobotSim, Warning,
				TEXT("[ASerial6DoFRobotActor] End Effector Target Actor 생성에 실패했습니다."));
		}
	}
	else
	{
		EndEffectorTargetActor->SetActorTransform(CurrentEEWorld);
		UE_LOG(LogRobotSim, Log,
			TEXT("[ASerial6DoFRobotActor] 기존 Target Actor를 현재 EE 위치로 정렬했습니다."));
	}
}

void ASerial6DoFRobotActor::CopyCurrentEndEffectorToTarget()
{
	if (EndEffectorTargetActor)
	{
		// Target Actor는 월드 공간이므로 현재 EE 월드 자세로 맞춘다.
		const FTransform CurrentEEWorld = Model.ComputeEndEffectorTransform(CurrentState) * GetActorTransform();
		EndEffectorTargetActor->SetActorTransform(CurrentEEWorld);
		UE_LOG(LogRobotSim, Log, TEXT("[ASerial6DoFRobotActor] Target Actor를 현재 EE로 리셋했습니다."));
	}
	else
	{
		// TargetEndEffectorWorld는 모델 공간(actor-relative) 규약이므로 액터 트랜스폼을 합성하지 않는다.
		TargetEndEffectorWorld = Model.ComputeEndEffectorTransform(CurrentState);
		UE_LOG(LogRobotSim, Log, TEXT("[ASerial6DoFRobotActor] TargetEndEffectorWorld를 현재 EE(모델 공간)로 리셋했습니다."));
	}
}

void ASerial6DoFRobotActor::CalibrateToolOffsetFromTarget()
{
	if (!EndEffectorTargetActor)
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[ASerial6DoFRobotActor] Target Actor가 없습니다. SpawnOrAlign으로 생성한 뒤 target을 실제 그리퍼 끝으로 옮기고 다시 실행하세요."));
		return;
	}

	// 사용자가 target을 실제 그리퍼 끝에 배치했다고 보고, 그 월드 자세를 로봇 모델 공간으로 내린다.
	const FTransform TipWorld = EndEffectorTargetActor->GetActorTransform();
	const FTransform TipModel = TipWorld.GetRelativeTransform(GetActorTransform());

	// J5(마지막 관절) 프레임 (모델 공간, 현재 자세 기준).
	const FTransform J5Model = Model.ComputeJointWorldTransform(FSerial6DoFModel::NumJoints - 1, CurrentState);

	// FK 합성이 EE_model = ToolOffset * J5_model 이므로, ToolOffset = TipModel을 J5Model 기준 상대 변환으로 역산.
	const FTransform NewToolOffset = TipModel.GetRelativeTransform(J5Model);

	const FVector Loc = NewToolOffset.GetLocation();
	const FRotator Rot = NewToolOffset.Rotator();
	UE_LOG(LogRobotSim, Log,
		TEXT("[ASerial6DoFRobotActor] ToolOffset 캘리브레이션 결과: 위치=(%.3f, %.3f, %.3f)cm, 회전(Roll,Pitch,Yaw)=(%.3f, %.3f, %.3f)도 (현재 자세 기준)."),
		Loc.X, Loc.Y, Loc.Z, Rot.Roll, Rot.Pitch, Rot.Yaw);

	if (RobotConfig)
	{
		// 영구 저장: DataAsset에 기록하고 모델을 재구성한다 (사용자가 에셋을 저장해야 유지됨).
		RobotConfig->Modify();
		RobotConfig->ToolOffset = NewToolOffset;
		RobotConfig->MarkPackageDirty();
		RefreshFromConfig();
		UE_LOG(LogRobotSim, Log,
			TEXT("[ASerial6DoFRobotActor] RobotConfig '%s'의 ToolOffset을 갱신했습니다. 에셋을 저장하세요(Ctrl+S)."),
			*RobotConfig->GetName());
	}
	else
	{
		// RobotConfig가 없으면 모델에 직접 적용한다 (일시적: OnConstruction/BeginPlay 재구성 시 CreateDefault로 되돌아감).
		Model.ToolOffset = NewToolOffset;
		MirrorModelToComponents();
		ApplyJointState();
		UE_LOG(LogRobotSim, Warning,
			TEXT("[ASerial6DoFRobotActor] RobotConfig가 없어 ToolOffset을 모델에 일시 적용했습니다(재구성 시 초기화됨). 영구 반영하려면 RobotConfig 에셋을 만들어 위 값을 ToolOffset에 입력하세요."));
	}
}

void ASerial6DoFRobotActor::RefreshFromConfig()
{
	// 기구학 파라미터의 해석 지점: RobotConfig가 있으면 그 값으로, 없으면 코드 기본값으로.
	// 수학 레이어(FSerial6DoFModel)는 DataAsset을 알지 못하며, 여기서만 값을 주입받는다.
	Model = RobotConfig ? RobotConfig->ToModel() : FSerial6DoFModel::CreateDefault();

	// 모델이 바뀌었을 수 있으므로 관절 컴포넌트의 오프셋을 다시 미러링한 뒤 각도를 재적용한다.
	MirrorModelToComponents();
	ApplyAnglesFromEditor();
}

void ASerial6DoFRobotActor::MirrorModelToComponents()
{
	// 관절 프레임의 상대 위치 = 모델 LinkOffsets, EE 프레임 상대 변환 = 모델 ToolOffset.
	// 회전은 ApplyJointState()가 관절 각도로 매 적용하므로 여기서는 위치/툴 변환만 다룬다.
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		if (JointComponents[i])
		{
			JointComponents[i]->SetRelativeLocation(Model.LinkOffsets[i]);
		}
	}
	if (ToolTipComponent)
	{
		ToolTipComponent->SetRelativeTransform(Model.ToolOffset);
	}
}

USkeletalMesh* ASerial6DoFRobotActor::GetConfiguredSkeletalMesh() const
{
	return RobotConfig ? RobotConfig->SkeletalMeshAsset : nullptr;
}

FName ASerial6DoFRobotActor::GetConfiguredBoneName(int32 JointIndex) const
{
	if (!RobotConfig || JointIndex < 0 || JointIndex >= FSerial6DoFModel::NumJoints)
	{
		return NAME_None;
	}
	return RobotConfig->JointBoneNames[JointIndex];
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

	// 현재 EE와 IK target 사이 링크 라인 (target이 지정됐을 때만). solve 전/후 오차를 눈으로 확인.
	if (bDrawTargetLink && EndEffectorTargetActor)
	{
		DrawDebugLine(World, MathEE.GetLocation(), EndEffectorTargetActor->GetActorLocation(),
			FColor(255, 140, 0), false, -1.0f, 0, DebugAxisThickness);
	}
}

void ASerial6DoFRobotActor::ApplyLinkVisuals()
{
	// 이 함수는 메시/가시성만 다룬다. 관절 프레임의 상대 변환(FK 미러링)은
	// ApplyJointState()가 소유하며 여기서는 절대 건드리지 않는다.

	// 디버그 도형은 일단 토글 값대로 켠 뒤, StaticMesh가 실제 표시되는 슬롯만 아래에서 숨긴다.
	for (const TObjectPtr<UStaticMeshComponent>& DebugMesh : LinkMeshComponents)
	{
		if (DebugMesh)
		{
			DebugMesh->SetVisibility(bShowDebugLinks);
		}
	}

	for (int32 i = 0; i < NumVisualLinks; ++i)
	{
		if (!VisualMeshComponents[i] || !LinkVisuals.IsValidIndex(i))
		{
			continue;
		}

		const FLinkVisualConfig& Config = LinkVisuals[i];

		VisualMeshComponents[i]->SetStaticMesh(Config.Mesh);
		VisualMeshComponents[i]->SetRelativeLocation(Config.RelativeLocation);
		VisualMeshComponents[i]->SetRelativeRotation(Config.RelativeRotation);
		VisualMeshComponents[i]->SetRelativeScale3D(Config.RelativeScale);

		// 메시 미할당 슬롯은 숨겨서 기존 디버그 도형만 보이게 한다 (요구: 미할당 링크는 디버그 표시 유지).
		const bool bMeshShown = bShowStaticMeshes && Config.Mesh != nullptr;
		VisualMeshComponents[i]->SetVisibility(bMeshShown);

		// StaticMesh가 표시 중인 링크는 대응 디버그 도형을 숨겨 겹침을 막는다.
		if (bMeshShown && DebugMeshForVisualSlot[i])
		{
			DebugMeshForVisualSlot[i]->SetVisibility(false);
		}
	}
}

void ASerial6DoFRobotActor::ApplySkeletalMeshVisual()
{
	if (!SkeletalVisualComponent)
	{
		return;
	}

	// 메시/본 이름은 RobotConfig가 소유한다. 여기서 한 번 해석해 로컬로 쓴다.
	USkeletalMesh* const MeshAsset = GetConfiguredSkeletalMesh();

	// 에셋이 바뀌었거나 포즈 버퍼가 본 개수와 맞지 않을 때 재init한다.
	// (버퍼 크기가 틀리면 Reset/Sync가 조기 return하거나 인덱싱에서 크래시하므로 반드시 재할당한다.
	//  PIE 시작 시 버퍼가 비었거나 다른 크기로 남아 있는 경우를 여기서 흡수한다.)
	const int32 ExpectedBoneCount = MeshAsset ? MeshAsset->GetRefSkeleton().GetNum() : 0;
	const bool bAssetChanged = (SkeletalVisualComponent->GetSkinnedAsset() != MeshAsset);
	const bool bPoseBufferInvalid =
		(MeshAsset != nullptr && SkeletalVisualComponent->BoneSpaceTransforms.Num() != ExpectedBoneCount);
	if (bAssetChanged || bPoseBufferInvalid)
	{
		SkeletalVisualComponent->SetSkinnedAssetAndUpdate(MeshAsset, /*bReinitPose=*/true);
	}

	// 메시 활성 여부는 에셋 할당만으로 결정한다. KUKA 메시는 수학 6R과 1:1이 아니므로
	// 일부 관절(예: J3 forearm roll)이 None이어도 메시를 숨기지 않고 해당 관절만 skip한다.
	bSkeletalMeshActive = (MeshAsset != nullptr);
	if (MeshAsset && !IsTemplate())
	{
		for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
		{
			const FName BoneName = GetConfiguredBoneName(i);
			if (BoneName.IsNone())
			{
				// 의도적 미매핑 (수학에는 있으나 메시 외형이 지원하지 않는 자유도).
				UE_LOG(LogRobotSim, Log,
					TEXT("[ASerial6DoFRobotActor] J%d: visual-only unmapped joint — SkeletalMesh 동기화에서 skip합니다 (수학 FK/디버그 링크에는 그대로 존재)."), i);
			}
			else if (SkeletalVisualComponent->GetBoneIndex(BoneName) == INDEX_NONE)
			{
				// 이름은 있으나 에셋에 없음 — 오타 가능성. 해당 관절만 skip하고 메시는 계속 표시한다.
				UE_LOG(LogRobotSim, Warning,
					TEXT("[ASerial6DoFRobotActor] J%d의 본 '%s'을(를) SkeletalMesh '%s'에서 찾지 못했습니다. 이 관절만 skip합니다 (이름 오타 확인)."),
					i, *BoneName.ToString(), *MeshAsset->GetName());
			}
		}
	}

	// 에셋만 있으면 표시한다. 개별 관절 매핑 실패는 표시에 영향 없음.
	SkeletalVisualComponent->SetVisibility(bSkeletalMeshActive && bShowSkeletalMesh);

	if (bSkeletalMeshActive)
	{
		// 미매핑 본을 ref pose로 되돌린 뒤 매핑된 본만 동기화한다.
		ResetPoseToRefPose();
		SyncSkeletalPoseToMath();
	}
}

void ASerial6DoFRobotActor::SyncSkeletalPoseToMath()
{
	// 델타 회전 리타겟: 수학 모델이 source of truth이고, 각 관절의 회전각 Q[i]만
	// 대응 본의 바인드 로컬 회전 위에 얹는다. 본의 바인드 평행이동(=메시 고유
	// 링크 길이)은 그대로 유지하므로 수학 모델과 메시의 비율이 달라도 스키닝이
	// 찌그러지지 않는다. 대신 본 위치는 메시 비율을 따르므로 수학 프레임
	// (디버그 링크)과 위치가 어긋나는 것은 의도된 동작이다 — 수치 검증은
	// 디버그 링크/CheckVisualMatchesMath()가 담당한다.
	//
	// 전제: 메시의 바인드 포즈가 수학 Q=0 자세와 같은 구성이고, 본 체인 순서가
	// J0→J5 순서와 일치한다 (확정된 본 매핑).
	if (!bSkeletalMeshActive || !SkeletalVisualComponent || !SkeletalVisualComponent->GetSkinnedAsset())
	{
		return;
	}

	const FReferenceSkeleton& RefSkeleton = SkeletalVisualComponent->GetSkinnedAsset()->GetRefSkeleton();
	const TArray<FTransform>& RefPose = RefSkeleton.GetRefBonePose();
	const int32 NumBones = RefSkeleton.GetNum();

	if (SkeletalVisualComponent->BoneSpaceTransforms.Num() != NumBones)
	{
		return;
	}

	// 바인드 포즈의 본별 컴포넌트 공간 회전을 누적한다 (부모 인덱스가 항상 앞이므로 단일 패스).
	TArray<FQuat> BindComponentSpaceRotations;
	BindComponentSpaceRotations.SetNum(NumBones);
	for (int32 BoneIdx = 0; BoneIdx < NumBones; ++BoneIdx)
	{
		const int32 ParentIdx = RefSkeleton.GetParentIndex(BoneIdx);
		const FQuat LocalRot = RefPose[BoneIdx].GetRotation();
		BindComponentSpaceRotations[BoneIdx] = (ParentIdx != INDEX_NONE)
			? BindComponentSpaceRotations[ParentIdx] * LocalRot
			: LocalRot;
	}

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		const int32 BoneIndex = RefSkeleton.FindBoneIndex(GetConfiguredBoneName(i));
		if (BoneIndex == INDEX_NONE)
		{
			continue; // 유효성 검증/Warning은 ApplySkeletalMeshVisual()이 담당
		}

		// 수학 Q=0에서 모든 관절 프레임 회전은 identity이므로 관절 축은 컴포넌트
		// 공간에서 Model.JointAxes[i] 그대로다. 로컬 회전 델타는 부모 본 공간에서
		// 작용하므로 축을 부모 본의 바인드 프레임으로 옮긴 뒤 Q[i]만큼 회전을 얹는다.
		const int32 ParentIndex = RefSkeleton.GetParentIndex(BoneIndex);
		const FQuat ParentBindComponentSpace = (ParentIndex != INDEX_NONE)
			? BindComponentSpaceRotations[ParentIndex]
			: FQuat::Identity;

		const FVector AxisInParentBind = ParentBindComponentSpace.Inverse().RotateVector(Model.JointAxes[i]);
		const FQuat DeltaRotation(AxisInParentBind, CurrentState.Q[i]);

		FTransform NewLocal = RefPose[BoneIndex]; // 바인드 평행이동/스케일 유지
		NewLocal.SetRotation(DeltaRotation * RefPose[BoneIndex].GetRotation());
		SkeletalVisualComponent->BoneSpaceTransforms[BoneIndex] = NewLocal;
	}

	SkeletalVisualComponent->RefreshBoneTransforms();
}

bool ASerial6DoFRobotActor::ResetPoseToRefPose()
{
	if (!SkeletalVisualComponent || !SkeletalVisualComponent->GetSkinnedAsset())
	{
		UE_LOG(LogRobotSim, Warning, TEXT("[ASerial6DoFRobotActor] SkeletalMeshAsset이 할당되지 않아 ref pose로 되돌릴 수 없습니다."));
		return false;
	}

	const TArray<FTransform>& RefPose = SkeletalVisualComponent->GetSkinnedAsset()->GetRefSkeleton().GetRefBonePose();

	// GetNumBones()는 ref skeleton 기준이라 포즈 버퍼(BoneSpaceTransforms)가 아직
	// 할당되지 않아도 nonzero다. 버퍼가 비어 있으면 인덱싱이 크래시하므로, 버퍼가
	// ref pose와 같은 크기로 할당됐을 때만 직접 덮어쓴다 (Sync와 동일한 가드).
	if (SkeletalVisualComponent->BoneSpaceTransforms.Num() != RefPose.Num())
	{
		return false;
	}

	for (int32 i = 0; i < RefPose.Num(); ++i)
	{
		SkeletalVisualComponent->BoneSpaceTransforms[i] = RefPose[i];
	}
	SkeletalVisualComponent->RefreshBoneTransforms();
	return true;
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
