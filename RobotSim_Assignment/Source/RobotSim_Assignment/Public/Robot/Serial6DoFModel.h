// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Robot/RobotTypes.h"

/**
 * @struct FSerial6DoFModel
 * @brief 6R serial 로봇의 순수 수학 기구학 모델.
 *
 * UObject/World에 의존하지 않는 plain struct이므로 액터 스폰 없이 단위테스트할 수 있다.
 * 액터(ASerial6DoFRobotActor)는 이 모델의 축/오프셋을 그대로 SceneComponent 계층에
 * 미러링하므로, 파라미터의 유일한 정의처(single source of truth)는 이 모델이다.
 *
 * @details
 * 좌표/단위 규약:
 * - 길이: Unreal cm
 * - 각도: radian (도 단위는 에디터 경계에서만 사용)
 * - 좌표계: Unreal 왼손 Z-up. 회전은 FQuat(Axis, AngleRad) 규약을 따름
 *   (+Z축 +90° 회전 시 +X → +Y, +Y축 +90° 회전 시 +X → -Z)
 *
 * FK 정의:
 * 각 관절 i의 로컬 변환은 "링크 오프셋만큼 평행이동 후 관절 축 기준 회전"이며,
 * FTransform(FQuat(Axis, Q), Offset)으로 표현된다. 이는 SceneComponent의
 * RelativeLocation + RelativeRotation 의미와 정확히 일치한다.
 * 월드 누적은 UE 규약대로 World = Local * Parent (child-first) 순서로 합성한다.
 */
struct FSerial6DoFModel
{
	/** 관절 개수 (6R 고정) */
	static constexpr int32 NumJoints = 6;

	/** 로봇 베이스의 기준 변환. 단위테스트에서는 Identity를 사용한다. */
	FTransform BaseTransform = FTransform::Identity;

	/** 각 관절의 회전축 (자기 프레임 기준 단위 벡터) */
	FVector JointAxes[NumJoints] = {
		FVector::ZAxisVector, FVector::YAxisVector, FVector::YAxisVector,
		FVector::XAxisVector, FVector::YAxisVector, FVector::XAxisVector
	};

	/** 이전 관절 프레임에서 이 관절 피벗까지의 평행이동 (cm) */
	FVector LinkOffsets[NumJoints] = {
		FVector::ZeroVector, FVector::ZeroVector, FVector::ZeroVector,
		FVector::ZeroVector, FVector::ZeroVector, FVector::ZeroVector
	};

	/** 각 관절의 가동 범위/속도/토크 제한 */
	FRobotJointLimit JointLimits[NumJoints];

	/** 마지막 관절(J5) 프레임에서 End Effector(툴 끝단)까지의 변환 */
	FTransform ToolOffset = FTransform::Identity;

	/**
	 * 각 링크의 강체 동역학 파라미터 + 관절 액추에이터 특성 (SI 단위).
	 *
	 * Step B-01에서는 **저장만 하며 어떤 계산에도 쓰이지 않는다.** 아래 FK 함수들
	 * (ComputeJointWorldTransform / ComputeEndEffectorTransform / ComputeEndEffectorPose /
	 * ClampToLimits / IsWithinLimits)은 이 필드를 읽지 않으므로, 값을 어떻게 바꿔도
	 * Step A의 기구학 결과는 변하지 않는다. Step B-02(RNEA)부터 사용된다.
	 *
	 * 링크 i의 span은 LinkOffsets[i]가 아니라 LinkOffsets[i+1]이다 (i=5는 ToolOffset).
	 * 자세한 규약은 FRobotLinkDynamics의 주석 참조.
	 */
	FRobotLinkDynamics LinkDynamics[NumJoints];

	/**
	 * 중력 가속도 벡터 (m/s²), 베이스 프레임 기준. Unreal Z-up이므로 기본값은 −Z 방향이다.
	 *
	 * LinkDynamics와 마찬가지로 Step B-01에서는 저장만 하고 FK는 이 값을 읽지 않는다.
	 * Step B-02 RNEA의 base acceleration 초기값으로 사용된다(중력 보상 토크의 원천).
	 */
	FVector GravityMPerSec2 = FVector(0.0, 0.0, -9.81);

	/**
	 * @brief 기본 6R 로봇 파라미터를 생성한다.
	 *
	 * KUKA류 산업용 암을 단순화한 구성:
	 * J0 yaw(+Z) → J1/J2 pitch(+Y) → J3 roll(+X) → J4 pitch(+Y) → J5 roll(+X).
	 * Q=0에서 베이스 기둥은 +Z 수직, 팔꿈치 이후는 +X 수평으로 전개되며
	 * End Effector는 액터 공간 (105, 0, 120)cm에 위치한다.
	 *
	 * Step B-01부터 LinkDynamics/GravityMPerSec2도 함께 채우지만, FK는 이 값을 읽지 않으므로
	 * 위 기하 결과는 불변이다.
	 */
	static FSerial6DoFModel CreateDefault();

	/**
	 * @brief 현재 기하(LinkOffsets/ToolOffset)로부터 LinkDynamics 기본값을 유도해 채운다.
	 *
	 * 각 링크를 span 방향 균일 밀도 실린더로 근사한다: COM = span 중점, 관성은
	 * 축방향 ½mr² / 횡방향 (1/12)m(3r²+L²). 질량·반경·액추에이터 값은 CAD 근거가 없는
	 * 잠정 추정치다(STEP_B-01.md의 "한계" 참조). CreateDefault()가 호출하며,
	 * 기하를 바꾼 뒤 동역학 값을 다시 유도하고 싶을 때도 쓸 수 있다.
	 *
	 * FK에는 영향이 없다 (LinkDynamics를 읽는 FK 함수가 없다).
	 */
	void InitializeDefaultLinkDynamics();

	/**
	 * @brief 관절 JointIndex의 회전이 적용된 프레임의 월드 변환을 계산한다.
	 * @param JointIndex 0~5
	 * @param State 관절 각도 (radian)
	 */
	FTransform ComputeJointWorldTransform(int32 JointIndex, const FRobot6DJointState& State) const;

	/**
	 * @brief Forward Kinematics: Base → J0~J5 → ToolOffset을 누적한 End Effector 변환을 계산한다.
	 * @param State 관절 각도 (radian)
	 */
	FTransform ComputeEndEffectorTransform(const FRobot6DJointState& State) const;

	/** ComputeEndEffectorTransform 결과를 위치/쿼터니언 쌍(FRobot6DPose)으로 반환한다. */
	FRobot6DPose ComputeEndEffectorPose(const FRobot6DJointState& State) const;

	/** 모든 관절 각도를 각자의 가동 범위로 잘라낸 상태를 반환한다. */
	FRobot6DJointState ClampToLimits(const FRobot6DJointState& State) const;

	/** 모든 관절 각도가 가동 범위 안에 있는지 검사한다. */
	bool IsWithinLimits(const FRobot6DJointState& State) const;
};
