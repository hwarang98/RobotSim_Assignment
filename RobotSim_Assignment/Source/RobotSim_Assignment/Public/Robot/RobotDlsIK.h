// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Robot/RobotTypes.h"

struct FSerial6DoFModel;

/**
 * @struct FRobotDlsIKOptions
 * @brief Damped Least Squares(DLS) IK solver의 반복/수렴/안정화 파라미터.
 *
 * 단위 규약(기존 수학 레이어와 동일): 길이 cm, 각도 radian.
 * 리플렉션이 필요 없는 순수 계산용 타입이므로 USTRUCT로 만들지 않는다.
 */
struct FRobotDlsIKOptions
{
	/** 최대 반복 횟수. 도달 불가/local minimum에서도 무한 루프를 방지한다. */
	int32 MaxIterations = 80;

	/** 위치 수렴 허용 오차 (cm). PositionErrorNorm이 이 값 이하이면 수렴으로 본다. */
	double PositionToleranceCm = 0.5;

	/** 회전 수렴 허용 오차 (radian). RotationErrorNorm이 이 값 이하이면 수렴으로 본다. */
	double RotationToleranceRad = FMath::DegreesToRadians(2.0);

	/** damping 계수 λ. singularity 근처에서 inverse 폭주를 막는 안정화 항. */
	double DampingLambda = 0.15;

	/** 한 반복에서 각 관절 Δq의 최대 크기 (radian). 선형 근사가 유효한 범위로 스텝을 제한한다. */
	double MaxStepRad = FMath::DegreesToRadians(8.0);

	/** 위치 오차/Jacobian 위치 행에 곱하는 가중치. */
	double PositionWeight = 1.0;

	/** 회전 오차/Jacobian 회전 행에 곱하는 가중치. cm와 rad의 스케일 차이를 보정한다. */
	double RotationWeight = 30.0;

	/** numerical Jacobian의 유한차분 스텝 (radian). */
	double JacobianEpsilonRad = 1e-4;

	/** true면 매 반복마다 결과를 관절 가동 범위로 clamp한다. */
	bool bClampJointLimits = true;
};

/**
 * @struct FRobotDlsIKResult
 * @brief DLS IK solver의 결과. 최종 관절 상태와 수렴/오차 정보를 담는다.
 *
 * 리플렉션이 필요 없는 순수 계산용 타입이므로 USTRUCT로 만들지 않는다.
 */
struct FRobotDlsIKResult
{
	/** tolerance 이내로 수렴했으면 true. 실패/발산/반복 소진 시 false. */
	bool bConverged = false;

	/** 실제 수행한 반복 횟수. */
	int32 Iterations = 0;

	/** 최종 관절 상태(해). 수렴 여부와 무관하게 마지막 유효 상태를 담는다. */
	FRobot6DJointState Solution;

	/** 최종 상태 기준 pose error (가중치 미적용, target − current). */
	FRobot6DPoseError FinalError;

	/** 최종 위치 오차 크기 (cm). */
	double FinalPositionErrorCm = 0.0;

	/** 최종 회전 오차 크기 (radian). */
	double FinalRotationErrorRad = 0.0;
};

/**
 * @struct FRobotDlsIK
 * @brief Damped Least Squares(DLS) IK를 계산하는 순수 수학 함수 모음.
 *
 * 기존 검증된 레이어만 조합한다(신규 기구학 없음):
 * - FK: FSerial6DoFModel::ComputeEndEffectorTransform
 * - 오차 Δx: FRobotPoseError::ComputePoseError (target − current, 6-벡터)
 * - Jacobian J: FRobotJacobian::ComputeNumericalJacobian (6×6)
 *
 * 반복 갱신식(핵심):
 *   e   = [Wp·Δpos(cm), Wr·Δrot(rad)]           (가중 오차 6-벡터)
 *   J   = row-scaled Jacobian (행0~2 ×Wp, 행3~5 ×Wr)
 *   dq  = Jᵀ (J Jᵀ + λ² I)⁻¹ e                   (Damped Least Squares)
 *   q  += clamp(dq, ±MaxStepRad)
 *
 * pseudo-inverse(J⁺) 대신 DLS를 쓰는 이유: singularity 근처에서 J⁺가 폭주하는 것을
 * A = J Jᵀ + λ² I (λ>0이면 양의정부호에 근접)로 안정화하기 위함이다.
 *
 * UObject/World에 의존하지 않으므로 액터 스폰 없이 단위테스트할 수 있다
 * (FRobotJacobian/FRobotPoseError와 동일한 static-struct 패턴).
 */
struct FRobotDlsIK
{
	/**
	 * @brief 단일 목표 Transform에 대해 DLS IK를 반복적으로 풀어 관절 해를 구한다.
	 *
	 * @param Model          FK/한계 source of truth (호출 후 불변)
	 * @param InitialState   초기 관절 자세 (radian). 호출 후에도 불변.
	 * @param TargetTransform 목표 End Effector 변환
	 * @param Options        반복/수렴/damping 파라미터
	 * @return 최종 관절 상태와 수렴/오차 정보
	 */
	static FRobotDlsIKResult SolveDlsIK(
		const FSerial6DoFModel& Model,
		const FRobot6DJointState& InitialState,
		const FTransform& TargetTransform,
		const FRobotDlsIKOptions& Options);
};
