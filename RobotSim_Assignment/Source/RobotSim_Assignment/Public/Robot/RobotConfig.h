// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Robot/Serial6DoFModel.h"
#include "RobotConfig.generated.h"

class USkeletalMesh;

/**
 * @struct FRobotJointConfig
 * @brief DataAsset에서 관절 하나를 authoring하기 위한 에디터 친화 표현.
 *
 * 각도는 여기(에디터 경계)에서만 도(degree)로 다루고, ToModel()에서 radian으로 변환한다.
 * 순수 수학 타입 FSerial6DoFModel/FRobotJointLimit(내부 radian)와 분리된 authoring 레이어다.
 */
USTRUCT(BlueprintType)
struct FRobotJointConfig
{
	GENERATED_BODY()

	/** 관절 회전축 (자기 프레임 기준 단위 벡터). ToModel에서 정규화한다. */
	UPROPERTY(EditAnywhere, Category = "Robot|Kinematics")
	FVector Axis = FVector::ZAxisVector;

	/** 이전 관절 프레임에서 이 관절 피벗까지의 평행이동 (cm). */
	UPROPERTY(EditAnywhere, Category = "Robot|Kinematics")
	FVector LinkOffsetCm = FVector::ZeroVector;

	/** 관절 최소 각도 (도). */
	UPROPERTY(EditAnywhere, Category = "Robot|Kinematics")
	double MinDeg = -180.0;

	/** 관절 최대 각도 (도). */
	UPROPERTY(EditAnywhere, Category = "Robot|Kinematics")
	double MaxDeg = 180.0;

	/** 관절 최대 각속도 (도/초). 이후 궤적/속도 제한 단계용 예약 값. */
	UPROPERTY(EditAnywhere, Category = "Robot|Kinematics")
	double MaxVelDegPerSec = 180.0;

	/**
	 * 관절 최대 토크 (N·m). Step B-01에서는 저장만 하고 계산에 쓰이지 않는다.
	 * 각도만 도(degree)로 authoring하고, 동역학 값은 SI 그대로 둔다 (STEP_B-01.md 참조).
	 */
	UPROPERTY(EditAnywhere, Category = "Robot|Dynamics")
	double MaxTorqueNm = 100.0;
};

/**
 * @struct FRobotLinkDynamicsConfig
 * @brief DataAsset에서 링크 하나의 동역학 파라미터를 authoring하기 위한 에디터 표현.
 *
 * 순수 수학 타입 FRobotLinkDynamics와 1:1 대응하며 **단위도 동일한 SI**다. 각도(도↔radian)와 달리
 * 여기서는 변환을 두지 않는다: ToModel()을 순수 복사로 유지해 Step B-01에서 단위 변환 버그가
 * 끼어들 여지를 없애기 위함이다. cm↔m 경계 변환은 Step B-02에서 RNEA 진입점 한 곳
 * (LinkOffsets 등 기구학 cm 값을 SI로 올리는 지점)에 모아 처리한다.
 */
USTRUCT(BlueprintType)
struct FRobotLinkDynamicsConfig
{
	GENERATED_BODY()

	/** 링크 질량 (kg). 0 이하면 ToModel()에서 기본 로봇 값으로 대체된다. */
	UPROPERTY(EditAnywhere, Category = "Robot|Dynamics")
	double MassKg = 1.0;

	/**
	 * 질량중심 위치 (m) — 관절 프레임 기준 로컬 좌표.
	 * 같은 에셋의 LinkOffsetCm은 cm인데 이 값만 m이므로 혼동 주의 (필드명의 단위 접미사를 따를 것).
	 */
	UPROPERTY(EditAnywhere, Category = "Robot|Dynamics")
	FVector CenterOfMassLocalM = FVector::ZeroVector;

	/** COM 기준 관성 텐서 대각 성분 (kg·m²). 성분이 0 이하면 ToModel()에서 대체된다. */
	UPROPERTY(EditAnywhere, Category = "Robot|Dynamics")
	FVector InertiaDiagonalKgM2 = FVector(1.0e-3, 1.0e-3, 1.0e-3);

	/** 감속기 반영 로터 관성 (kg·m²). 0이면 미모델링(유효한 값). */
	UPROPERTY(EditAnywhere, Category = "Robot|Dynamics")
	double RotorInertiaKgM2 = 0.0;

	/** 점성 마찰 계수 (N·m·s/rad). 0이면 미모델링. */
	UPROPERTY(EditAnywhere, Category = "Robot|Dynamics")
	double ViscousFrictionNmsPerRad = 0.0;

	/** Coulomb 마찰 토크 크기 (N·m). 0이면 미모델링. */
	UPROPERTY(EditAnywhere, Category = "Robot|Dynamics")
	double CoulombFrictionNm = 0.0;
};

/**
 * @class URobotConfig
 * @brief 6R serial 로봇의 완전한 정의를 담는 DataAsset.
 *
 * 하나의 에셋이 로봇 하나를 서술한다:
 * - 기구학: 축/링크 오프셋/관절 한계/툴 오프셋 → ToModel()로 순수 수학 모델 FSerial6DoFModel 생성
 * - 시각화 배선: SkeletalMesh 에셋 + J0~J5 대응 본 이름(JointBoneNames)
 *
 * 컴파일 없이 데이터로 로봇을 정의/교체/복제하기 위한 authoring 레이어다. 순수 수학 레이어
 * (FSerial6DoFModel)는 UObject에 의존하지 않는 원칙을 유지하기 위해, 이 DataAsset은 값만 보관하고
 * ToModel()로 순수 struct를 "생성"만 한다 (수학 레이어가 이 DataAsset을 참조하지 않는다 →
 * 단위테스트는 그대로 CreateDefault 사용).
 *
 * 기본값은 생성자에서 CreateDefault() + 확정된 KUKA 본 매핑으로 채우므로, 새로 만든 에셋은
 * SkeletalMesh만 지정하면 곧바로 기본 로봇으로 동작한다.
 */
UCLASS(BlueprintType)
class ROBOTSIM_ASSIGNMENT_API URobotConfig : public UDataAsset
{
	GENERATED_BODY()

public:
	URobotConfig();

	//~ 기구학 정의 -------------------------------------------------------------

	/** 로봇 베이스 기준 변환. 보통 Identity. */
	UPROPERTY(EditAnywhere, Category = "Robot|Kinematics")
	FTransform BaseTransform = FTransform::Identity;

	/** J0~J5 관절 정의 (축/오프셋/한계). UHT 고정 배열 크기는 리터럴만 허용하므로 6을 쓴다. */
	UPROPERTY(EditAnywhere, Category = "Robot|Kinematics")
	FRobotJointConfig Joints[6];
	static_assert(FSerial6DoFModel::NumJoints == 6, "Joints 배열 리터럴 크기 6은 FSerial6DoFModel::NumJoints와 일치해야 한다");

	/** 마지막 관절(J5) 프레임에서 End Effector까지의 변환. */
	UPROPERTY(EditAnywhere, Category = "Robot|Kinematics")
	FTransform ToolOffset = FTransform::Identity;

	//~ 동역학 정의 (Step B) ----------------------------------------------------

	/**
	 * J0~J5 링크의 동역학 파라미터 (SI). Step B-01에서는 저장만 하며 FK/IK에 전혀 영향이 없다.
	 * 링크 i의 span은 LinkOffsetCm[i]가 아니라 LinkOffsetCm[i+1]임에 주의 (FRobotLinkDynamics 주석 참조).
	 */
	UPROPERTY(EditAnywhere, Category = "Robot|Dynamics")
	FRobotLinkDynamicsConfig LinkDynamics[6];
	static_assert(FSerial6DoFModel::NumJoints == 6, "LinkDynamics 배열 리터럴 크기 6은 FSerial6DoFModel::NumJoints와 일치해야 한다");

	/** 중력 가속도 벡터 (m/s²), 베이스 프레임 기준. Unreal Z-up이므로 기본은 −Z. */
	UPROPERTY(EditAnywhere, Category = "Robot|Dynamics")
	FVector GravityMPerSec2 = FVector(0.0, 0.0, -9.81);

	//~ 시각화 배선 -------------------------------------------------------------

	/**
	 * 시각화에 사용할 로봇 SkeletalMesh 에셋. 미지정(None)이면 SkeletalMesh 시각화가 비활성화되고
	 * 수학 FK는 디버그 링크로 계속 표시된다.
	 */
	UPROPERTY(EditAnywhere, Category = "Robot|Visual")
	TObjectPtr<USkeletalMesh> SkeletalMeshAsset;

	/**
	 * J0~J5 관절에 대응하는 SkeletalMesh 본 이름. None인 관절은 시각화에서 skip한다
	 * (예: J3 forearm roll — 메시에 대응 자유도 없음). 본 이름은 특정 메시에 종속된다.
	 */
	UPROPERTY(EditAnywhere, Category = "Robot|Visual")
	FName JointBoneNames[6];

	//~ 시각 파지점 (Visual Grasp Point) ----------------------------------------

	/**
	 * 그리퍼/흡착판이 달린 본의 이름. ASerial6DoFRobotActor의 VisualGraspPoint 컴포넌트가 이 본에
	 * 부착되어 메시를 따라다닌다.
	 *
	 * @details
	 * **왜 별도 파지점이 필요한가**: KUKA 메시(3.2m)와 수학 모델(105cm)은 크기가 달라서, 델타 회전
	 * 리타겟이 본 길이를 메시 고유값으로 유지하는 이상 시각 그리퍼와 수학 EE는 홈 자세에서만 겹치고
	 * 관절이 돌면 갈라진다(SyncSkeletalPoseToMath 주석 참조). 메시를 수학 크기로 줄이면 겹치지만
	 * 3.2m 산업용 로봇이 장난감이 된다. 그래서 **메시는 원래 크기/비율 그대로 두고**, 박스는 수학 EE가
	 * 아니라 이 본에 달린 VisualGraspPoint에 붙인다 — 보이는 그리퍼가 물체를 잡는 것이 우선이다.
	 *
	 * 수학 EE는 여전히 IK/FK의 기준이자 검증 대상으로 남는다. 둘의 차이는
	 * LogVisualToolAlignment()가 "visual calibration offset"으로 보고한다.
	 *
	 * 비워두면 VisualGraspPoint는 메시 루트에 붙으며(본을 따라가지 않음) 파지가 어긋난다.
	 */
	UPROPERTY(EditAnywhere, Category = "Robot|Visual|Grasp")
	FName VisualGraspBoneName;

	//~ 변환 -------------------------------------------------------------------

	/**
	 * @brief 이 DataAsset의 기구학 + 동역학 값으로 순수 수학 모델 FSerial6DoFModel을 생성한다.
	 *
	 * 도→radian 변환은 여기서만 일어난다. Axis는 정규화하며, 0 벡터 등 비정상 축은
	 * CreateDefault()의 대응 축으로 대체해 항상 유효한 모델을 반환한다.
	 *
	 * 동역학 값(LinkDynamics/Gravity/MaxTorqueNm)은 authoring 단위가 이미 SI라 **단순 복사**하되,
	 * NaN/Inf/0 이하 등 물리적으로 불가능한 값은 CreateDefault()의 대응 값으로 대체해
	 * 모델이 항상 유효한 상태로 나오도록 방어한다. Step B-01에서는 저장만 하므로 이 값들이
	 * FK 결과를 바꾸지 않는다.
	 */
	FSerial6DoFModel ToModel() const;
};
