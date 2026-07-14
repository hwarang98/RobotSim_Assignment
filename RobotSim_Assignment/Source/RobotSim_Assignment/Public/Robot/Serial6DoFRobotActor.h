// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Robot/RobotTypes.h"
#include "Robot/Serial6DoFModel.h"
#include "Serial6DoFRobotActor.generated.h"

class UPoseableMeshComponent;
class USkeletalMesh;
class UStaticMesh;
class UStaticMeshComponent;

/** Bone Probe에서 본을 회전시킬 로컬 축 */
UENUM()
enum class EProbeAxis : uint8
{
	X,
	Y,
	Z
};

/**
 * @struct FLinkVisualConfig
 * @brief 링크 하나에 표시할 StaticMesh와 시각 보정값 설정.
 *
 * 순수 비주얼 레이어 전용 구조체. 여기의 어떤 값도 FK 수학 모델이나
 * 관절 프레임(SceneComponent 체인)에는 영향을 주지 않는다.
 * 보정값(위치/회전/스케일)은 관절 프레임 기준의 상대 변환으로,
 * 외부 메시의 피벗/축 정렬을 관절 프레임에 맞추는 용도다.
 */
USTRUCT(BlueprintType)
struct FLinkVisualConfig
{
	GENERATED_BODY()

	/** 슬롯 라벨 (Details 가독성용, 생성자에서 고정 설정) */
	UPROPERTY(VisibleAnywhere, Category = "Robot|Visual")
	FName LinkName;

	/** 이 링크에 표시할 StaticMesh. 미할당(None)이면 기존 디버그 도형만 표시된다. */
	UPROPERTY(EditAnywhere, Category = "Robot|Visual")
	TObjectPtr<UStaticMesh> Mesh;

	/** 메시 피벗 보정: 관절 프레임 기준 상대 위치 (cm, 순수 시각 보정) */
	UPROPERTY(EditAnywhere, Category = "Robot|Visual")
	FVector RelativeLocation = FVector::ZeroVector;

	/** 메시 축 정렬 보정: 관절 프레임 기준 상대 회전 (순수 시각 보정) */
	UPROPERTY(EditAnywhere, Category = "Robot|Visual")
	FRotator RelativeRotation = FRotator::ZeroRotator;

	/** 메시 크기 보정: 상대 스케일 (순수 시각 보정) */
	UPROPERTY(EditAnywhere, Category = "Robot|Visual")
	FVector RelativeScale = FVector::OneVector;
};

/**
 * @class ASerial6DoFRobotActor
 * @brief 6R serial 로봇의 비주얼 표현 액터.
 *
 * J0~J5 6개 revolute 관절을 SceneComponent 부모-자식 체인으로 구성하고,
 * 링크는 엔진 기본 도형(Cube/Cylinder) 디버그 메시로 표시한다.
 * 추가로 두 가지 순수 비주얼 레이어를 제공한다:
 * - 링크별 StaticMesh 슬롯(LinkVisuals, Base + Link1~6): 분리 메시/디버그용
 * - SkeletalMesh 시각화(SkeletalVisualComponent): 수학 FK의 관절 회전각을
 *   J0~J5 대응 본(JointBoneNames)에 델타 회전 리타겟으로 얹어 본 체인을 동기화한다.
 * 어느 레이어도 FK 결과에 영향을 주지 않는다. 수학 모델이 source of truth이고
 * 본 트랜스폼은 그 결과를 따라가기만 한다.
 *
 * @details
 * 관절 축과 링크 오프셋의 유일한 정의처는 FSerial6DoFModel이며, 생성자에서
 * 모델 값을 그대로 컴포넌트 계층에 미러링한다 (이중 정의 금지).
 * 따라서 컴포넌트의 월드 트랜스폼과 수학 FK 결과는 항상 일치해야 하고,
 * 그 일치 여부를 CheckVisualMatchesMath()로 매 적용 시 검증한다.
 *
 * 각도 단위 규약: 내부 상태(CurrentState)는 radian, 에디터 노출 프로퍼티
 * (JointAnglesDeg)만 도(degree)를 사용한다.
 */
UCLASS()
class ROBOTSIM_ASSIGNMENT_API ASerial6DoFRobotActor : public AActor
{
	GENERATED_BODY()

public:
	/** 비주얼 링크 슬롯 개수: Base(Root) 1개 + Link1~6(J0~J5) */
	static constexpr int32 NumVisualLinks = FSerial6DoFModel::NumJoints + 1;
	static_assert(NumVisualLinks == 7, "UPROPERTY 고정 배열(VisualMeshComponents 등)의 리터럴 크기 7과 일치해야 한다");

	ASerial6DoFRobotActor();

	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void Tick(float DeltaSeconds) override;

	/** 에디터 뷰포트에서도 Tick을 돌려 디버그 축을 PIE 밖에서도 볼 수 있게 한다. */
	virtual bool ShouldTickIfViewportsOnly() const override { return true; }

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	#pragma region RobotAPI

	/**
	 * @brief 관절 각도를 설정한다 (radian).
	 *
	 * 입력을 모델의 관절 한계로 클램프한 뒤 저장하고 즉시 컴포넌트에 반영한다.
	 */
	void SetJointAngles(const FRobot6DJointState& NewState);

	/** End Effector의 월드 공간 6D 자세를 수학 FK로 계산해 반환한다. */
	FRobot6DPose GetEndEffectorPose() const;

	/** 현재 관절 상태(CurrentState)를 SceneComponent 계층에 반영한다. */
	void ApplyJointState();

	FORCEINLINE const FSerial6DoFModel& GetModel() const { return Model; }
	FORCEINLINE const FRobot6DJointState& GetJointState() const { return CurrentState; }

	#pragma endregion

	#pragma region EditorUtility

	/** 현재 EE 자세를 로그로 출력하고 수학-비주얼 일관성을 검증한다. */
	UFUNCTION(CallInEditor, Category = "Robot")
	void LogEndEffectorPose();

	/**
	 * 현재 EE 자세와 TargetEndEffectorWorld 사이의 6D pose error를 로그로 출력한다 (디버그 전용).
	 * 순수 수학 FRobotPoseError로 계산하며, IK 이동이나 UI는 수행하지 않는다.
	 */
	UFUNCTION(CallInEditor, Category = "Robot")
	void LogCurrentEndEffectorPoseErrorToTarget();

	/** 모든 관절 각도를 0으로 초기화한다. */
	UFUNCTION(CallInEditor, Category = "Robot")
	void ResetJointAngles();

	#pragma endregion

	#pragma region BoneProbe

	/**
	 * ProbeBoneName 본 하나만 ProbeAxis(본 로컬 축) 기준으로 ProbeAngleDeg만큼 회전시킨다.
	 * 실행 전 전체를 ref pose로 되돌리므로 이전 Probe 잔재가 누적되지 않는다.
	 * 어떤 본이 어떤 파츠를 움직이는지 확인해 JointBoneNames 매핑을 확정하기 위한 조사 도구.
	 */
	UFUNCTION(CallInEditor, Category = "Robot|BoneProbe")
	void ApplyBoneProbe();

	/** SkeletalMesh 포즈를 ref pose로 되돌린다. */
	UFUNCTION(CallInEditor, Category = "Robot|BoneProbe")
	void ResetBoneProbe();

	/** SkeletalMesh의 모든 본을 [인덱스] 이름 (parent: 부모이름) 형식으로 로그에 출력한다. */
	UFUNCTION(CallInEditor, Category = "Robot|BoneProbe")
	void DumpSkeletonBones();

	#pragma endregion

protected:
	#pragma region EditorProperties

	/**
	 * J0~J5 관절 각도 (도 단위, 에디터 조작용).
	 * 관절 한계를 벗어난 입력은 적용 시 한계값으로 클램프되어 되돌아 써진다.
	 */
	UPROPERTY(EditAnywhere, Category = "Robot|Joints", meta = (UIMin = "-350.0", UIMax = "350.0"))
	double JointAnglesDeg[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

	/**
	 * 링크별 StaticMesh 시각화 설정 (7개 고정: [0]=Base, [1..6]=Link1~6).
	 * 각 슬롯은 대응 관절 프레임(Base는 Root, LinkN은 J(N-1))의 자식 메시를 구성한다.
	 */
	UPROPERTY(EditAnywhere, EditFixedSize, Category = "Robot|Visual")
	TArray<FLinkVisualConfig> LinkVisuals;

	/** 할당된 링크 StaticMesh를 표시할지 여부 */
	UPROPERTY(EditAnywhere, Category = "Robot|Visual")
	bool bShowStaticMeshes = true;

	/**
	 * 기존 기본 도형 디버그 링크 메시를 표시할지 여부.
	 * 켜져 있어도 StaticMesh가 실제 표시 중인 링크는 겹침 방지를 위해 디버그 도형을 숨긴다.
	 */
	UPROPERTY(EditAnywhere, Category = "Robot|Visual")
	bool bShowDebugLinks = true;

	/**
	 * 시각화에 사용할 로봇 SkeletalMesh 에셋.
	 * 본 체인은 수학 FK 결과를 따라가기만 하는 순수 비주얼이며, AnimBP 없이
	 * PoseableMesh로 본 트랜스폼을 직접 쓴다.
	 */
	UPROPERTY(EditAnywhere, Category = "Robot|Visual")
	TObjectPtr<USkeletalMesh> SkeletalMeshAsset;

	/**
	 * J0~J5 관절에 대응하는 SkeletalMesh 본 이름 매핑.
	 * 비어 있거나 에셋에 없는 이름이 있으면 Warning을 남기고 SkeletalMesh를 숨긴 채
	 * 디버그 링크만 표시한다.
	 */
	UPROPERTY(EditAnywhere, Category = "Robot|Visual")
	FName JointBoneNames[6];

	/** SkeletalMesh 시각화를 표시할지 여부 (본 매핑이 유효할 때만 실제로 보인다) */
	UPROPERTY(EditAnywhere, Category = "Robot|Visual")
	bool bShowSkeletalMesh = true;

	/** Probe로 회전시킬 본 이름 (DumpSkeletonBones 출력에서 선택) */
	UPROPERTY(EditAnywhere, Category = "Robot|BoneProbe")
	FName ProbeBoneName;

	/** Probe 회전 축 (본 로컬 기준) */
	UPROPERTY(EditAnywhere, Category = "Robot|BoneProbe")
	EProbeAxis ProbeAxis = EProbeAxis::Z;

	/** Probe 회전 각도 (도) */
	UPROPERTY(EditAnywhere, Category = "Robot|BoneProbe")
	double ProbeAngleDeg = 45.0;

	/**
	 * Probe 모드: 본 매핑(JointBoneNames)이 미완성이어도 SkeletalMesh를 표시하고,
	 * FK 자동 동기화를 멈춰 Probe 포즈가 덮어써지지 않게 한다.
	 * 본 매핑 조사 단계에서만 켜고, 매핑 확정 후에는 끈다.
	 */
	UPROPERTY(EditAnywhere, Category = "Robot|BoneProbe")
	bool bShowProbeMesh = false;

	/**
	 * LogCurrentEndEffectorPoseErrorToTarget이 사용하는 월드 공간 목표 EE 자세 (디버그 전용).
	 * pose error = 이 목표 − 현재 EE. IK 입력이 아니라 오차 계산 확인용 값이다.
	 */
	UPROPERTY(EditAnywhere, Category = "Robot|PoseError")
	FTransform TargetEndEffectorWorld = FTransform::Identity;

	/** 각 관절 프레임과 EE 프레임에 디버그 좌표축을 그릴지 여부 */
	UPROPERTY(EditAnywhere, Category = "Robot|Debug")
	bool bDrawDebugFrames = true;

	/** EE 자세 주기 로그 간격 (초). 0 이하면 주기 로그를 끈다. */
	UPROPERTY(EditAnywhere, Category = "Robot|Debug", meta = (ClampMin = "0.0"))
	float PoseLogIntervalSeconds = 1.0f;

	#pragma endregion

	#pragma region Components

	/** J0~J5 관절 프레임. 부모-자식 체인으로 연결되며 상대 위치는 모델의 LinkOffsets를 따른다. */
	UPROPERTY(VisibleAnywhere, Category = "Robot")
	TObjectPtr<USceneComponent> JointComponents[6];

	/** End Effector(툴 끝단) 프레임. J5의 자식이며 상대 변환은 모델의 ToolOffset을 따른다. */
	UPROPERTY(VisibleAnywhere, Category = "Robot")
	TObjectPtr<USceneComponent> ToolTipComponent;

	/** 링크 디버그 메시 (비주얼 전용, 충돌 없음, FK에 영향 없음) */
	UPROPERTY(VisibleAnywhere, Category = "Robot")
	TArray<TObjectPtr<UStaticMeshComponent>> LinkMeshComponents;

	/**
	 * 링크별 사용자 할당 StaticMesh 컴포넌트 ([0]=Base→Root, [i]=LinkI→J(i-1)의 자식).
	 * 항상 생성해 두고 LinkVisuals의 메시/보정값을 ApplyLinkVisuals()로 반영한다.
	 * 관절 프레임 체인 밖의 자식이므로 FK에 영향 없음.
	 * (UHT 고정 배열 크기는 리터럴만 허용되어 NumVisualLinks 대신 7을 쓴다.)
	 */
	UPROPERTY(VisibleAnywhere, Category = "Robot")
	TObjectPtr<UStaticMeshComponent> VisualMeshComponents[7];

	/**
	 * 비주얼 슬롯 i에 대응하는 디버그 도형 메시 매핑 (StaticMesh 표시 시 겹침 숨김용).
	 * LinkMeshComponents의 생성 순서에 암묵 의존하지 않도록 생성자에서 명시적으로 기록한다.
	 */
	UPROPERTY(VisibleAnywhere, Category = "Robot")
	TObjectPtr<UStaticMeshComponent> DebugMeshForVisualSlot[7];

	/**
	 * SkeletalMesh 시각화 컴포넌트 (PoseableMesh, Root 직속).
	 * SyncSkeletalPoseToMath()가 수학 FK의 관절 회전각을 매핑된 본의 바인드 로컬
	 * 회전 위에 델타로 얹는다 (본 길이는 메시 고유 값 유지 → 찌그러짐 없음).
	 * 관절 프레임 체인과 독립이므로 FK에 영향 없음.
	 */
	UPROPERTY(VisibleAnywhere, Category = "Robot")
	TObjectPtr<UPoseableMeshComponent> SkeletalVisualComponent;

	#pragma endregion

private:
	/** 기구학 파라미터의 유일한 정의처. 생성자에서 CreateDefault()로 초기화한다. */
	FSerial6DoFModel Model;

	/** 현재 관절 상태 (radian) */
	FRobot6DJointState CurrentState;

	/** 주기 로그 타이머 */
	double TimeSinceLastPoseLog = 0.0;

	/** JointAnglesDeg(도)를 radian으로 변환해 SetJointAngles를 호출하고, 클램프 결과를 도로 되돌려 쓴다. */
	void ApplyAnglesFromEditor();

	/** ToolTip 컴포넌트의 월드 트랜스폼과 수학 FK 결과를 비교해 불일치 시 Warning을 남긴다. */
	void CheckVisualMatchesMath() const;

	/** 관절 프레임/EE에 디버그 좌표축을 그린다. */
	void DrawDebugJointFrames() const;

	/**
	 * LinkVisuals 설정을 VisualMeshComponents에 반영한다 (메시/보정 변환/가시성).
	 * StaticMesh가 표시되는 슬롯은 대응 디버그 도형을 숨긴다. 순수 비주얼 갱신이며
	 * 관절 프레임/FK에는 관여하지 않는다.
	 */
	void ApplyLinkVisuals();

	/**
	 * SkeletalMesh 에셋을 컴포넌트에 반영하고 본 매핑 상태를 로그로 보고한다.
	 * KUKA 메시는 수학 6R과 1:1이 아니므로 매핑은 approximate overlay다:
	 * - None인 관절 → "visual-only unmapped joint" 로그 후 skip (예: J3 forearm roll)
	 * - 이름이 있으나 에셋에 없는 관절 → Warning(오타 가능성) 후 skip
	 * 어느 경우든 메시 전체를 숨기지 않고(에셋만 있으면 표시), 매핑된 관절만 동기화한다.
	 * 수학 FK 6-DOF 전체는 디버그 링크/ToolTip이 계속 표시한다.
	 */
	void ApplySkeletalMeshVisual();

	/**
	 * 수학 FK의 관절 회전각(Q)을 J0~J5 대응 본에 델타 회전 리타겟으로 반영한다.
	 * 본의 바인드 평행이동(메시 고유 링크 길이)은 유지하고 회전만 얹으므로
	 * 메시 비율이 수학 모델과 달라도 스키닝이 찌그러지지 않는다.
	 * 수학 모델이 source of truth이며 본은 결과를 따라가기만 한다 (FK 불변).
	 * 전제: 바인드 포즈 = 수학 Q=0 자세, 본 체인 순서 = J0→J5 순서.
	 */
	void SyncSkeletalPoseToMath();

	/** SkeletalMesh의 모든 본을 ref pose로 되돌린다. 성공 여부를 반환한다 (Probe 공용 헬퍼). */
	bool ResetPoseToRefPose();

	/** ProbeAxis enum을 본 로컬 단위 축 벡터로 변환한다. */
	static FVector ProbeAxisToVector(EProbeAxis Axis);

	/** SkeletalMesh가 표시/동기화 대상인지 (= 에셋 할당됨). ApplySkeletalMeshVisual에서 갱신.
	 *  개별 관절의 매핑 성공 여부와 무관하다 (미매핑 관절은 sync에서 개별 skip). */
	bool bSkeletalMeshActive = false;

	/** 링크 디버그 메시 컴포넌트를 생성하는 생성자 전용 헬퍼 */
	UStaticMeshComponent* CreateLinkMesh(
		USceneComponent* Parent, const FName& Name, UStaticMesh* Mesh,
		const FVector& RelativeLocation, const FRotator& RelativeRotation, const FVector& RelativeScale);
};
