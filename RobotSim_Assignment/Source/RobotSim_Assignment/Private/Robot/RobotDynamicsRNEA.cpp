// Fill out your copyright notice in the Description page of Project Settings.

#include "Robot/RobotDynamicsRNEA.h"

namespace
{
	/** 링크 개수 = 관절 개수. 프레임은 0~5, 프레임 6은 툴(가상 프레임)이다. */
	constexpr int32 NumJoints = FSerial6DoFModel::NumJoints;

	/**
	 * 대각 관성 텐서와 벡터의 곱. InertiaDiagonalKgM2가 프레임 축 정렬 대각이므로 성분곱이면 충분하다.
	 * (Step B-01에서 비대각을 3×3으로 확장하더라도 아래 RNEA 수식 자체는 바뀌지 않는다 —
	 *  이 함수만 행렬곱으로 교체하면 된다.)
	 */
	FVector MultiplyDiagonalInertia(const FVector& InertiaDiagonalKgM2, const FVector& VectorIn)
	{
		return FVector(
			InertiaDiagonalKgM2.X * VectorIn.X,
			InertiaDiagonalKgM2.Y * VectorIn.Y,
			InertiaDiagonalKgM2.Z * VectorIn.Z);
	}
}

FRobot6DJointTorque SolveInverseDynamicsRNEA(
	const FSerial6DoFModel& Model,
	const FRobot6DJointState& Q,
	const FRobot6DJointVelocity& Qd,
	const FRobot6DJointAcceleration& Qdd,
	const FRobotRNEAOptions& Options)
{
	//~ 0. 진입 경계: 기구학 값을 cm에서 m로 올린다 -----------------------------------
	// 이 블록이 **파일 전체에서 유일한 cm→m 지점**이다. 이후 코드는 전부 SI만 다룬다.
	// LinkOffsetM[i] = 프레임 i 원점의 프레임 i−1 좌표 (Q[i]로 회전하지 않은 값).
	// LinkOffsetM[6] = 툴 프레임 원점의 프레임 5 좌표 = 링크 5의 span 끝.
	// 주의: CenterOfMassLocalM은 이미 m이므로 여기서 변환하지 않는다.
	FVector LinkOffsetM[NumJoints + 1];
	for (int32 i = 0; i < NumJoints; ++i)
	{
		LinkOffsetM[i] = Model.LinkOffsets[i] * RobotCmToM;
	}
	LinkOffsetM[NumJoints] = Model.ToolOffset.GetTranslation() * RobotCmToM;

	// 관절 축은 정규화해 둔다 (authoring 경계에서 정규화되지만 모델을 직접 조립한 경우를 대비).
	FVector JointAxisLocal[NumJoints];
	FQuat ChildToParentRot[NumJoints];
	for (int32 i = 0; i < NumJoints; ++i)
	{
		JointAxisLocal[i] = Model.JointAxes[i].GetSafeNormal();
		// R[i]: 프레임 i → 프레임 i−1. 기존 FK의 FQuat(JointAxes[i], Q[i])와 동일하다.
		ChildToParentRot[i] = FQuat(JointAxisLocal[i], Q.Q[i]);
	}

	const FVector GravityMPerSec2 = Options.bEnableGravity
		? Options.GravityOverrideMPerSec2.Get(Model.GravityMPerSec2)
		: FVector::ZeroVector;

	//~ 1. Forward pass: 베이스 → 말단으로 속도/가속도를 전파한다 ----------------------
	// 각 프레임 i 기준으로 표현한다. 베이스는 정지 상태이며, base acceleration을 −g로 두는
	// 고전적 트릭으로 중력 항이 관성력에 자동으로 섞여 나온다 (별도 중력 항이 없는 이유).
	FVector AngularVelRadPerSec[NumJoints];
	FVector AngularAccelRadPerSec2[NumJoints];
	FVector LinkForceN[NumJoints];
	FVector LinkMomentNm[NumJoints];

	FVector ParentAngularVel = FVector::ZeroVector;
	FVector ParentAngularAccel = FVector::ZeroVector;
	FVector ParentLinearAccel = -GravityMPerSec2;

	for (int32 i = 0; i < NumJoints; ++i)
	{
		const FQuat ParentToChildRot = ChildToParentRot[i].Inverse();
		const FVector AxisTimesQd = JointAxisLocal[i] * Qd.Qd[i];

		// 부모의 운동을 자식 프레임으로 끌어온 값.
		const FVector ParentAngularVelInChild = ParentToChildRot.RotateVector(ParentAngularVel);

		// ω[i] = R^T·ω[i−1] + z[i]·qd[i]
		AngularVelRadPerSec[i] = ParentAngularVelInChild + AxisTimesQd;

		// α[i] = R^T·α[i−1] + (R^T·ω[i−1]) × (z[i]·qd[i]) + z[i]·qdd[i]
		AngularAccelRadPerSec2[i] =
			ParentToChildRot.RotateVector(ParentAngularAccel)
			+ FVector::CrossProduct(ParentAngularVelInChild, AxisTimesQd)
			+ JointAxisLocal[i] * Qdd.Qdd[i];

		// a[i] = R^T·( a[i−1] + α[i−1] × p[i] + ω[i−1] × (ω[i−1] × p[i]) )
		// p[i]는 부모 프레임 기준이므로 회전을 적용하기 **전에** 부모 프레임에서 전부 계산한다.
		// i=0에서는 부모(베이스)가 정지라 두 교차항이 0이 되어 LinkOffsetM[0](베이스 기둥)이
		// 자동으로 탈락한다 — 고정 베이스의 일부라 관절 토크에 기여하지 않는다는 사실의 수식적 반영.
		const FVector OriginAccelInParent =
			ParentLinearAccel
			+ FVector::CrossProduct(ParentAngularAccel, LinkOffsetM[i])
			+ FVector::CrossProduct(ParentAngularVel, FVector::CrossProduct(ParentAngularVel, LinkOffsetM[i]));
		const FVector LinearAccelMPerSec2 = ParentToChildRot.RotateVector(OriginAccelInParent);

		// COM 가속도: ac = a + α × c + ω × (ω × c)
		const FRobotLinkDynamics& Dynamics = Model.LinkDynamics[i];
		const FVector CenterOfMassLocalM = Dynamics.CenterOfMassLocalM;
		const FVector ComAccelMPerSec2 =
			LinearAccelMPerSec2
			+ FVector::CrossProduct(AngularAccelRadPerSec2[i], CenterOfMassLocalM)
			+ FVector::CrossProduct(AngularVelRadPerSec[i], FVector::CrossProduct(AngularVelRadPerSec[i], CenterOfMassLocalM));

		// 링크에 작용해야 하는 합력/합모멘트 (Newton-Euler). 모멘트는 COM 기준이다.
		LinkForceN[i] = Dynamics.MassKg * ComAccelMPerSec2;
		LinkMomentNm[i] =
			MultiplyDiagonalInertia(Dynamics.InertiaDiagonalKgM2, AngularAccelRadPerSec2[i])
			+ FVector::CrossProduct(
				AngularVelRadPerSec[i],
				MultiplyDiagonalInertia(Dynamics.InertiaDiagonalKgM2, AngularVelRadPerSec[i]));

		ParentAngularVel = AngularVelRadPerSec[i];
		ParentAngularAccel = AngularAccelRadPerSec2[i];
		ParentLinearAccel = LinearAccelMPerSec2;
	}

	//~ 2. Backward pass: 말단 → 베이스로 힘/모멘트를 역누적한다 -----------------------
	// f/n은 프레임 i 기준이고, n[i]는 프레임 i **원점**에 대한 모멘트다.
	// 말단 외력은 0으로 둔다 (Step B-05 grasp에서 여기가 확장 지점이 된다).
	FRobot6DJointTorque Torque;
	FVector ChildForceN = FVector::ZeroVector;
	FVector ChildMomentNm = FVector::ZeroVector;

	for (int32 i = NumJoints - 1; i >= 0; --i)
	{
		// 자식(i+1)이 링크 i에 가하는 반력을 프레임 i로 끌어온다. i=5는 자식이 없어 0이다.
		const bool bHasChild = (i + 1 < NumJoints);
		const FVector ForceFromChildN = bHasChild
			? ChildToParentRot[i + 1].RotateVector(ChildForceN)
			: FVector::ZeroVector;
		const FVector MomentFromChildNm = bHasChild
			? ChildToParentRot[i + 1].RotateVector(ChildMomentNm)
			: FVector::ZeroVector;

		// f[i] = R·f[i+1] + F[i]
		const FVector JointForceN = ForceFromChildN + LinkForceN[i];

		// n[i] = R·n[i+1] + c[i] × F[i] + p[i+1] × (R·f[i+1]) + N[i]
		// p[i+1] = 자식 프레임 원점의 프레임 i 좌표 = LinkOffsetM[i+1].
		const FVector JointMomentNm =
			MomentFromChildNm
			+ FVector::CrossProduct(Model.LinkDynamics[i].CenterOfMassLocalM, LinkForceN[i])
			+ FVector::CrossProduct(LinkOffsetM[i + 1], ForceFromChildN)
			+ LinkMomentNm[i];

		// 관절 축 성분만이 액추에이터가 내야 하는 토크다 (나머지는 베어링이 받는다).
		Torque.TauNm[i] = FVector::DotProduct(JointMomentNm, JointAxisLocal[i]);

		ChildForceN = JointForceN;
		ChildMomentNm = JointMomentNm;
	}

	//~ 3. 옵션 항: 강체 토크에 별도로 가산한다 ---------------------------------------
	for (int32 i = 0; i < NumJoints; ++i)
	{
		const FRobotLinkDynamics& Dynamics = Model.LinkDynamics[i];

		if (Options.bIncludeRotorInertia)
		{
			// 감속기 반영 로터 관성은 관절 축에 대한 스칼라 관성이므로 그대로 더한다.
			Torque.TauNm[i] += Dynamics.RotorInertiaKgM2 * Qdd.Qdd[i];
		}

		if (Options.bIncludeFriction)
		{
			// 마찰은 운동을 방해하는 방향(τ_f = −b·qd − c·sign(qd))으로 작용하므로,
			// **모터가 이를 추가로 이겨내야 한다** → 역동역학 토크에는 +로 가산한다.
			// qd=0이면 Coulomb 항은 0이다 (정지 마찰은 모델링하지 않는다 — STEP_B-02.md "한계").
			Torque.TauNm[i] += Dynamics.ViscousFrictionNmsPerRad * Qd.Qd[i];
			Torque.TauNm[i] += Dynamics.CoulombFrictionNm * FMath::Sign(Qd.Qd[i]);
		}
	}

	return Torque;
}

FRobot6DJointTorque ComputeGravityTorque(
	const FSerial6DoFModel& Model,
	const FRobot6DJointState& Q,
	const FRobotRNEAOptions& Options)
{
	// qd = qdd = 0 → 관성/원심/코리올리 항이 전부 사라지고 중력 항만 남는다.
	const FRobot6DJointVelocity ZeroVelocity;
	const FRobot6DJointAcceleration ZeroAcceleration;
	return SolveInverseDynamicsRNEA(Model, Q, ZeroVelocity, ZeroAcceleration, Options);
}
