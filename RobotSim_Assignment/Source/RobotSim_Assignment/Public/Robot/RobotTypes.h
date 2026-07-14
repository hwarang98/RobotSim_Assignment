// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "RobotTypes.generated.h"

/**
 * @struct FRobotJointLimit
 * @brief 단일 revolute 관절의 가동 범위와 최대 각속도를 정의하는 구조체.
 *
 * 모든 각도는 radian으로 저장한다. 도(degree) 단위는 에디터/UI 경계에서만 사용하고
 * 내부 계산에 들어오기 전에 radian으로 변환할 것.
 */
USTRUCT(BlueprintType)
struct FRobotJointLimit
{
	GENERATED_BODY()

	/** 관절 최소 각도 (radian) */
	UPROPERTY(EditAnywhere, Category = "Robot|JointLimit")
	double MinRad = -PI;

	/** 관절 최대 각도 (radian) */
	UPROPERTY(EditAnywhere, Category = "Robot|JointLimit")
	double MaxRad = PI;

	/**
	 * 관절 최대 각속도 (radian/sec).
	 * Step A-1에서는 사용하지 않으며, 이후 궤적 보간/속도 제한 단계를 위한 예약 필드.
	 */
	UPROPERTY(EditAnywhere, Category = "Robot|JointLimit")
	double MaxVelRadPerSec = PI;

	/** 입력 각도를 [MinRad, MaxRad] 범위로 잘라 반환한다. */
	double Clamp(double AngleRad) const
	{
		return FMath::Clamp(AngleRad, MinRad, MaxRad);
	}

	/** 입력 각도가 가동 범위 안에 있는지 검사한다. */
	bool Contains(double AngleRad) const
	{
		return AngleRad >= MinRad && AngleRad <= MaxRad;
	}
};

/**
 * @struct FRobot6DPose
 * @brief End Effector의 6D 자세(위치 + 방향)를 표현하는 구조체.
 *
 * 위치는 Unreal 단위(cm), 방향은 쿼터니언으로 표현한다.
 * FK 결과 출력 및 이후 IK 단계의 목표 자세 표현에 공통으로 사용한다.
 */
USTRUCT(BlueprintType)
struct FRobot6DPose
{
	GENERATED_BODY()

	/** 위치 (cm) */
	UPROPERTY(VisibleAnywhere, Category = "Robot|Pose")
	FVector PositionCm = FVector::ZeroVector;

	/** 방향 (쿼터니언) */
	UPROPERTY(VisibleAnywhere, Category = "Robot|Pose")
	FQuat Orientation = FQuat::Identity;
};

/**
 * @struct FRobot6DJointState
 * @brief 6개 관절 각도를 담는 관절 공간 상태.
 *
 * 각도는 radian, double 정밀도. 수학 모델(FSerial6DoFModel)의 입력으로 사용한다.
 * 리플렉션이 필요 없는 순수 계산용 타입이므로 USTRUCT로 만들지 않는다.
 */
struct FRobot6DJointState
{
	/** 관절 각도 Q[0]~Q[5] (radian) */
	double Q[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
};
