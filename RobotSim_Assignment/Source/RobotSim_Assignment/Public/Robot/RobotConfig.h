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

	//~ 변환 -------------------------------------------------------------------

	/**
	 * @brief 이 DataAsset의 기구학 값으로 순수 수학 모델 FSerial6DoFModel을 생성한다.
	 *
	 * 도→radian 변환은 여기서만 일어난다. Axis는 정규화하며, 0 벡터 등 비정상 축은
	 * CreateDefault()의 대응 축으로 대체해 항상 유효한 모델을 반환한다.
	 */
	FSerial6DoFModel ToModel() const;
};
