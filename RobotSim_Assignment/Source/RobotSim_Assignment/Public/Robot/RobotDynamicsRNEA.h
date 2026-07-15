// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Robot/RobotTypes.h"
#include "Robot/Serial6DoFModel.h"

/**
 * cm → m 변환 계수. **동역학 진입 경계 전용**이다.
 *
 * FSerial6DoFModel의 기구학 값(LinkOffsets/ToolOffset)은 Unreal cm이고 동역학은 전부 SI(m)이므로,
 * 둘이 만나는 지점에서 반드시 이 계수를 거쳐야 한다. Step B-01의 설계 제약대로 이 변환은
 * SolveInverseDynamicsRNEA() 진입부 한 곳에만 존재한다 (STEP_B-02.md의 "cm→m 경계 변환" 참조).
 *
 * 함정: 같은 모델 안에서도 LinkOffsets는 cm인데 FRobotLinkDynamics::CenterOfMassLocalM은
 * **이미 m**다. COM에 이 계수를 다시 곱하면 안 된다.
 */
constexpr double RobotCmToM = 0.01;

/**
 * @struct FRobotRNEAOptions
 * @brief RNEA 역동역학의 항 구성 옵션.
 *
 * 기본값은 **순수 강체 역동역학 + 중력**이다. 로터 관성과 마찰은 Step B-01에서 채운 값이 아직
 * 검증되지 않았고(Step B-06 토크 제어에서 튜닝 예정), 강체 항의 정확성을 흐리지 않도록
 * 기본적으로 꺼 둔다. 켜면 강체 토크에 별도 항으로 **가산**된다.
 */
struct FRobotRNEAOptions
{
	/**
	 * 중력을 포함할지 여부. false면 forward pass의 base acceleration이 0이 되어
	 * 중력 항이 완전히 사라진다 (qd=qdd=0이면 토크가 정확히 0).
	 */
	bool bEnableGravity = true;

	/** 로터 관성 항(τ += RotorInertiaKgM2 · qdd)을 포함할지 여부. */
	bool bIncludeRotorInertia = false;

	/** 마찰 항(τ += ViscousFriction · qd + CoulombFriction · sign(qd))을 포함할지 여부. */
	bool bIncludeFriction = false;

	/**
	 * 중력 벡터 오버라이드 (m/s², 베이스 프레임 기준).
	 * 미지정이면 Model.GravityMPerSec2를 사용한다 — RNEA는 중력 상수를 소유하지 않는다.
	 */
	TOptional<FVector> GravityOverrideMPerSec2;
};

/**
 * @brief Recursive Newton-Euler 역동역학: 관절 상태 (q, qd, qdd) → 관절 토크 τ (N·m).
 *
 * UObject/Actor/World에 의존하지 않는 순수 함수다. FSerial6DoFModel을 읽기만 하며 수정하지 않는다.
 *
 * @details
 * **단위 규약 — 내부는 전부 SI다** (m, kg, kg·m², m/s², N·m). 입력 Model의 LinkOffsets/ToolOffset은
 * cm이므로 진입부에서 RobotCmToM으로 m에 올린다. CenterOfMassLocalM/InertiaDiagonalKgM2/
 * GravityMPerSec2는 이미 SI라 변환하지 않는다.
 *
 * **프레임 규약.** 기존 FK가 `JointLocal = FTransform(FQuat(JointAxes[i], Q[i]), LinkOffsets[i])`,
 * `World = Local * Parent`이고 FTransform(Rot, Trans)가 `p ↦ Rot·p + Trans`이므로:
 * - `R[i] = FQuat(JointAxes[i], Q[i])`는 **프레임 i → 프레임 i−1** 사상이다 (부모→자식은 역).
 * - `LinkOffsets[i]`는 프레임 i 원점의 **부모 프레임 좌표**이며 Q[i]로 회전하지 않는다.
 *   회전은 오프셋만큼 이동한 **뒤** 프레임 i 원점에서 일어난다.
 * - `JointAxes[i]`는 프레임 i 기준이지만, 회전이 자기 축을 불변으로 두므로 프레임 i−1에서도
 *   수치가 같다. 따라서 축 변환 없이 그대로 쓴다 (교재의 z_i와 정확히 일치).
 * - **링크 i의 강체는 프레임 i 원점 → 프레임 i+1 원점 구간**이다. 즉 span은 LinkOffsets[i]가
 *   아니라 LinkOffsets[i+1]이며 i=5는 ToolOffset이다 (Step B-01의 "한 칸 밀림" 함정).
 * - `LinkOffsets[0]`(베이스 기둥)은 베이스가 정지 상태(ω=α=0)라 재귀에서 자동으로 탈락한다.
 *   J0 피벗보다 아래라 어떤 관절과도 회전하지 않는다는 사실이 수식으로 그대로 나온다.
 * - 모든 spatial 속도/가속도/힘(ω, α, a, f, n)은 **자기 링크 프레임 i 기준**으로 표현한다.
 *   n[i]는 프레임 i **원점**에 대한 모멘트이며, τ[i] = Dot(n[i], z[i])다.
 * - **BaseTransform은 무시한다.** GravityMPerSec2가 "베이스 프레임 기준"으로 정의돼 있어
 *   RNEA는 베이스 프레임에서 닫힌다. 베이스가 월드에서 기울어졌다면 호출자가 중력을 미리
 *   베이스 프레임으로 회전시켜 GravityOverrideMPerSec2로 넘겨야 한다.
 *
 * **중력 처리.** forward pass의 base acceleration을 `a[−1] = −g`로 두는 고전적 트릭을 쓴다.
 * 별도의 중력 항 없이 각 링크의 관성력에 중력이 자동으로 섞여 나오므로, qd=qdd=0이면 결과가
 * 그대로 **중력 보상 토크**(자세를 유지하는 데 필요한 정지 토크) g(q) = ∂U/∂q가 된다.
 *
 * **한계.** 3D 벡터 기반 revolute-only 재귀다 (spatial 6D 대수 아님). prismatic 관절, 분기 트리,
 * 말단 외력(grasp 반력)은 지원하지 않는다. 자세한 내용은 STEP_B-02.md의 "한계" 참조.
 *
 * @param Q       관절 각도 (radian)
 * @param Qd      관절 각속도 (radian/sec)
 * @param Qdd     관절 각가속도 (radian/sec²)
 * @param Options 항 구성 옵션 (기본: 중력 on, 로터 관성/마찰 off)
 * @return 각 관절이 내야 하는 토크 (N·m). 부호는 관절 축 양의 방향 기준.
 */
FRobot6DJointTorque SolveInverseDynamicsRNEA(
	const FSerial6DoFModel& Model,
	const FRobot6DJointState& Q,
	const FRobot6DJointVelocity& Qd,
	const FRobot6DJointAcceleration& Qdd,
	const FRobotRNEAOptions& Options = FRobotRNEAOptions());

/**
 * @brief 정지 자세(qd = qdd = 0)의 중력 보상 토크를 계산한다 (N·m).
 *
 * SolveInverseDynamicsRNEA의 편의 래퍼다. 결과는 해당 자세를 유지하는 데 필요한 정지 토크이며,
 * Step B-06 토크 제어의 feedforward 항이자 MaxTorqueNm 산정의 근거가 된다.
 * Options.bEnableGravity가 false면 당연히 0이 나온다.
 */
FRobot6DJointTorque ComputeGravityTorque(
	const FSerial6DoFModel& Model,
	const FRobot6DJointState& Q,
	const FRobotRNEAOptions& Options = FRobotRNEAOptions());
