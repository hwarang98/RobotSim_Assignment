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

	/**
	 * 관절 액추에이터가 낼 수 있는 최대 토크 (N·m).
	 *
	 * Step B-01에서는 저장만 하며 어떤 계산에도 쓰이지 않는다. Step B-02(RNEA)에서 정지 자세
	 * 중력 토크를 산출한 뒤 여유를 반영해 확정할 예정이고, Step B-06 토크 제어 루프에서
	 * 지령 토크를 이 값으로 saturate하는 데 사용된다.
	 *
	 * 단위 주의: 각도/길이는 이 구조체 전반이 radian/cm 규약이지만, 이 필드만 SI(N·m)다.
	 * 동역학 파라미터는 SI로 저장한다는 Step B 규약을 따른다 (STEP_B-01.md 참조).
	 */
	UPROPERTY(EditAnywhere, Category = "Robot|JointLimit")
	double MaxTorqueNm = 100.0;

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
 * @struct FRobotLinkDynamics
 * @brief 링크 하나의 강체 동역학 파라미터 + 그 링크를 구동하는 관절의 액추에이터 특성.
 *
 * 관절 i와 링크 i는 1:1 대응이므로(관절 i의 회전에 딸려 도는 강체가 링크 i) 두 종류의 값을
 * 한 구조체에 담는다. 리플렉션이 필요 없는 순수 계산용 타입이므로 USTRUCT로 만들지 않는다.
 *
 * @details
 * 단위 규약 — **이 구조체는 전부 SI다** (kg, m, s, N·m).
 * FK/IK 레이어(FSerial6DoFModel의 LinkOffsets/ToolOffset, FRobot6DPose 등)는 cm를 유지하고,
 * 동역학 값만 SI로 저장한다. 토크가 N·m로 나와야 실제 로봇 스펙과 대조할 수 있고 CSV 로그도
 * 의미를 갖기 때문이다. cm↔m 경계 변환은 Step B-02에서 RNEA 진입점 한 곳에 모아 처리한다
 * (Step B-01은 값을 저장만 하고 어떤 계산에도 쓰지 않는다). 자세한 근거는 STEP_B-01.md 참조.
 *
 * 링크 i의 기하학적 정의 (Step B-02 RNEA가 전제하는 규약):
 * FSerial6DoFModel의 FK 합성은 "LinkOffsets[i]만큼 이동한 지점에서 관절 i를 회전"이므로,
 * 관절 i에 딸려 도는 강체는 **프레임 i 원점에서 다음 프레임 원점까지**를 차지한다.
 * 즉 링크 i의 span 벡터는 LinkOffsets[i]가 아니라 LinkOffsets[i+1]이며(i=5는 ToolOffset),
 * LinkOffsets[0]=베이스 기둥은 J0 피벗보다 아래라 어떤 관절과도 회전하지 않아 동역학 대상이 아니다.
 */
struct FRobotLinkDynamics
{
	//~ 강체 관성 파라미터 -------------------------------------------------------

	/** 링크 질량 (kg). 항상 양수여야 한다. */
	double MassKg = 1.0;

	/** 질량중심 위치 (m). 관절 i 프레임 기준의 로컬 좌표다. */
	FVector CenterOfMassLocalM = FVector::ZeroVector;

	/**
	 * 질량중심 기준 관성 텐서의 대각 성분 (kg·m²), 관절 i 프레임 축에 정렬.
	 *
	 * 기본 로봇의 링크를 축 정렬 균일 실린더로 근사하므로 비대각 성분이 0이 되어 대각만 저장한다.
	 * 일반 텐서(비대각 포함)가 필요해지면 이 필드를 3×3으로 확장하면 되고, RNEA 수식 자체는
	 * 대각/일반을 구분하지 않는다.
	 */
	FVector InertiaDiagonalKgM2 = FVector(1.0e-3, 1.0e-3, 1.0e-3);

	//~ 액추에이터 특성 (관절 i, Step B-06 토크 제어에서 사용 예정) ---------------

	/**
	 * 감속기 반영 로터 관성 (kg·m²). 관절 축에 대한 스칼라 관성으로, 링크 관성에 더해진다.
	 * 0이면 로터 관성을 모델링하지 않는다는 뜻이며 물리적으로 유효하다.
	 */
	double RotorInertiaKgM2 = 0.0;

	/** 점성 마찰 계수 (N·m·s/rad). 토크 τ_v = −ViscousFriction · qd. 0이면 미모델링. */
	double ViscousFrictionNmsPerRad = 0.0;

	/** Coulomb 마찰 토크 크기 (N·m). 토크 τ_c = −CoulombFriction · sign(qd). 0이면 미모델링. */
	double CoulombFrictionNm = 0.0;
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

/**
 * @struct FRobot6DPoseError
 * @brief 현재 EE 자세와 목표 자세 사이의 6D pose error(위치 + 회전).
 *
 * 부호 규약: "current → target" 방향의 오차다 (target − current).
 * 이후 IK 단계(A-03 Jacobian, A-04 DLS)에서 Δx 입력 6-벡터로 사용된다.
 * 리플렉션이 필요 없는 순수 계산용 타입이므로 USTRUCT로 만들지 않는다.
 *
 * @details
 * 단위 규약:
 * - 위치 오차: Unreal cm
 * - 회전 오차: radian. axis-angle(= quaternion log × 2) 형태의 rotation vector로,
 *   벡터 방향 = 회전축, 벡터 크기 = 최단 회전각(rad). Euler 각 차이가 아니다.
 */
struct FRobot6DPoseError
{
	/** 위치 오차 (cm) = TargetPosition − CurrentPosition */
	FVector PositionErrorCm = FVector::ZeroVector;

	/** 회전 오차 (radian) = axis × angle rotation vector (shortest path) */
	FVector RotationErrorRad = FVector::ZeroVector;

	/**
	 * 6-벡터 [px, py, pz, rx, ry, rz]로 채워 반환한다.
	 * A-03 Jacobian / A-04 DLS의 오차 입력 벡터 레이아웃과 동일하다.
	 */
	void ToArray6(double Out[6]) const
	{
		Out[0] = PositionErrorCm.X;
		Out[1] = PositionErrorCm.Y;
		Out[2] = PositionErrorCm.Z;
		Out[3] = RotationErrorRad.X;
		Out[4] = RotationErrorRad.Y;
		Out[5] = RotationErrorRad.Z;
	}

	/** 위치 오차 크기 (cm) */
	double PositionErrorNorm() const
	{
		return PositionErrorCm.Size();
	}

	/** 회전 오차 크기 = 최단 회전각 (radian) */
	double RotationErrorNorm() const
	{
		return RotationErrorRad.Size();
	}
};
