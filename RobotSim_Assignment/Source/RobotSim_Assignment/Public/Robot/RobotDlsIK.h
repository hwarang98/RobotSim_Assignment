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

	//~ Nullspace joint-limit avoidance (STEP A-05) ---------------------------------

	/**
	 * true면 primary task(EE pose 추종)의 nullspace로 관절을 중립 자세 쪽으로 되돌리는
	 * 보조 항을 추가한다. task-space에는 영향을 주지 않으므로 pose 수렴을 해치지 않는다.
	 * 기본값 false: 켜지 않으면 STEP A-04와 완전히 동일하게 동작한다.
	 */
	bool bUseNullspaceJointLimitAvoidance = false;

	/**
	 * nullspace로 중립 자세를 향해 되돌리려는 보조 관절 속도의 크기(gain).
	 * 너무 크면 매 반복 nullspace step이 커져 task 수렴을 방해할 수 있으므로 작게 둔다.
	 */
	double NullspaceGain = 0.05;

	/**
	 * 관절 한계 회피를 시작하는 normalized 거리 임계값 (0=중립, 1=한계).
	 * |normalized| < 이 값이면 회피 속도를 0으로 두어, 중앙부에서는 task를 방해하지 않고
	 * 한계 근처에서만 되돌림을 활성화한다.
	 */
	double JointLimitActivationRatio = 0.65;
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

	//~ Nullspace joint-limit avoidance 진단값 (STEP A-05) --------------------------

	/** 이번 solve에서 nullspace joint-limit avoidance 항을 실제로 적용했으면 true. */
	bool bNullspaceUsed = false;

	/** 최종 상태의 max |normalized joint distance| (0=모두 중립, 1=한계에 붙음). */
	double MaxAbsNormalizedJointDistance = 0.0;

	/** 마지막 반복에서 적용된 nullspace step(N·dq_null)의 크기(norm). 미사용 시 0. */
	double NullspaceStepNorm = 0.0;
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

	/**
	 * @brief 각 관절을 중립(midpoint) 쪽으로 되돌리려는 desired nullspace 관절 속도를 계산한다.
	 *
	 * projection(N) 적용 전의 "원하는 속도"이며, task-space 제약은 아직 반영하지 않았다.
	 * 관절 i에 대해:
	 *   midpoint = (min+max)/2, halfRange = (max-min)/2, normalized = (q-midpoint)/halfRange
	 *   |normalized| < JointLimitActivationRatio → 0
	 *   그 이상 → -NullspaceGain · normalized  (중립 방향, 크기에 비례)
	 *
	 * @param OutVelocity 관절공간 속도 6개 (radian 스케일, weighting 없음).
	 */
	static void ComputeJointLimitAvoidanceVelocity(
		const FSerial6DoFModel& Model,
		const FRobot6DJointState& State,
		const FRobotDlsIKOptions& Options,
		double OutVelocity[6]);

	/**
	 * @brief 상태의 max |normalized joint distance|를 반환한다 (0=모두 중립, 1=한계에 붙음).
	 *        로그/테스트에서 "관절이 한계에 얼마나 가까운가"를 한 값으로 보기 위한 진단 함수.
	 */
	static double ComputeMaxAbsNormalizedJointDistance(
		const FSerial6DoFModel& Model,
		const FRobot6DJointState& State);
};
