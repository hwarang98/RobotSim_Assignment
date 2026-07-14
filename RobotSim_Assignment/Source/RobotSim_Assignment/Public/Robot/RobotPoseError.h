// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Robot/RobotTypes.h"

/**
 * @struct FRobotPoseError
 * @brief 6D pose error를 계산하는 순수 수학 함수 모음.
 *
 * UObject/World에 의존하지 않으므로 액터 스폰 없이 단위테스트할 수 있다.
 * 데이터 타입(FRobot6DPoseError)과 알고리즘을 분리한다 —
 * FSerial6DoFModel(알고리즘)이 RobotTypes(데이터)와 분리된 것과 같은 패턴.
 *
 * @details
 * 좌표/단위 규약(FSerial6DoFModel과 동일):
 * - 위치: Unreal cm
 * - 회전: radian
 * - 회전 오차는 Euler 각 차이가 아니라 quaternion 기반 axis-angle(rotation vector)로 계산한다.
 *
 * 부호 규약: 모든 오차는 "current → target" 방향(target − current)이다.
 * 이후 IK(A-03 Jacobian, A-04 DLS)의 Δx 입력 벡터가 된다.
 */
struct FRobotPoseError
{
	/**
	 * @brief 두 회전의 상대 회전을 axis-angle rotation vector(radian)로 계산한다.
	 *
	 * 절차:
	 *   q_err = Target * Current⁻¹        (월드 프레임 상대 회전)
	 *   if q_err.W < 0: q_err = -q_err     (shortest path: 각도 ∈ [0, π])
	 *   r_err = axis(단위) × angle         (= quaternion log × 2)
	 *
	 * @param Current 현재 회전
	 * @param Target  목표 회전
	 * @return 회전축 방향, 크기가 최단 회전각(rad)인 rotation vector. 180° 근처에서도 NaN이 없다.
	 */
	static FVector ComputeRotationError(const FQuat& Current, const FQuat& Target);

	/**
	 * @brief 현재 Transform과 목표 Transform 사이의 6D pose error를 계산한다 (가중치 적용 전).
	 * @param Current 현재 EE 변환
	 * @param Target  목표 EE 변환
	 */
	static FRobot6DPoseError ComputePoseError(const FTransform& Current, const FTransform& Target);

	/**
	 * @brief ComputePoseError 결과에 위치/회전 가중치를 각각 곱한 값을 반환한다.
	 *
	 * 가중치 적용 전 값(ComputePoseError)과 명확히 구분되는, 스케일된 오차다.
	 * @param PositionWeight 위치 오차 스칼라 가중치
	 * @param RotationWeight 회전 오차 스칼라 가중치
	 */
	static FRobot6DPoseError ComputeWeightedPoseError(
		const FTransform& Current, const FTransform& Target,
		double PositionWeight, double RotationWeight);
};
