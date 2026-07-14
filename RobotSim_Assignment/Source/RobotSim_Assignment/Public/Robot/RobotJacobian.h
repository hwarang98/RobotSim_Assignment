// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Robot/RobotTypes.h"

struct FSerial6DoFModel;

/**
 * @struct FRobotJacobian6x6
 * @brief 6-DOF serial robot의 6×6 Jacobian 행렬.
 *
 * 행/열 규약(FRobot6DPoseError::ToArray6 6-벡터 레이아웃과 동일):
 * - 행(row) 0~2: position error derivative, 단위 cm/rad
 * - 행(row) 3~5: rotation error derivative(quaternion-log rotation vector), 단위 rad/rad
 * - 열(col) 0~5: 관절 J0~J5
 *
 * 즉 M[row][col] = ∂(pose error 성분 row) / ∂(관절 col 각도).
 * 리플렉션이 필요 없는 순수 계산용 타입이므로 USTRUCT로 만들지 않는다.
 */
struct FRobotJacobian6x6
{
	/** 행렬 원소. M[row][col], row = pose error 성분, col = 관절 인덱스. */
	double M[6][6] = {};

	/** 모든 원소를 0으로 초기화한다. */
	void SetZero();

	/** (Row, Col) 원소 참조 (쓰기용). */
	double& At(int32 Row, int32 Col);

	/** (Row, Col) 원소 값 (읽기용). */
	double At(int32 Row, int32 Col) const;
};

/**
 * @struct FRobotJacobian
 * @brief numerical(finite-difference) Jacobian을 계산하는 순수 수학 함수 모음.
 *
 * FSerial6DoFModel(검증된 FK source of truth)과 FRobotPoseError(A-02 quaternion-log
 * pose error)를 그대로 재사용하는 수치 미분이므로, 유도 오류 가능성이 없어
 * 이후 analytic/geometric Jacobian의 검증 기준(gold reference)이 된다.
 *
 * UObject/World에 의존하지 않으므로 액터 스폰 없이 단위테스트할 수 있다
 * (FRobotPoseError와 동일한 static-struct 패턴).
 */
struct FRobotJacobian
{
	/** 기본 유한차분 스텝 (radian). truncation error와 round-off error의 균형점. */
	static constexpr double DefaultEpsilonRad = 1e-4;

	/**
	 * @brief 전진 유한차분으로 6×6 numerical Jacobian을 계산한다.
	 *
	 * 절차(각 관절 i에 대해):
	 *   current = FK(State)
	 *   StatePlus = State; StatePlus.Q[i] += epsilon
	 *   delta = ComputePoseError(current, FK(StatePlus))   // = plus − current
	 *   J.column(i) = delta / epsilon
	 *
	 * 회전 성분은 A-02의 quaternion-log rotation vector를 그대로 사용한다(Euler 차이 아님).
	 * 입력 State는 변경하지 않는다(로컬 복사본만 섭동).
	 *
	 * @param Model      FK source of truth
	 * @param State      기준 관절 자세 (radian). 호출 후에도 불변.
	 * @param EpsilonRad 유한차분 스텝. 0 이하이거나 지나치게 작으면 DefaultEpsilonRad로 대체.
	 * @return 6×6 numerical Jacobian
	 */
	static FRobotJacobian6x6 ComputeNumericalJacobian(
		const FSerial6DoFModel& Model,
		const FRobot6DJointState& State,
		double EpsilonRad = DefaultEpsilonRad);
};
