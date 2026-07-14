// Fill out your copyright notice in the Description page of Project Settings.

#include "Robot/RobotJacobian.h"

#include "Robot/RobotPoseError.h"
#include "Robot/Serial6DoFModel.h"

#pragma region FRobotJacobian6x6

void FRobotJacobian6x6::SetZero()
{
	for (int32 Row = 0; Row < 6; ++Row)
	{
		for (int32 Col = 0; Col < 6; ++Col)
		{
			M[Row][Col] = 0.0;
		}
	}
}

double& FRobotJacobian6x6::At(int32 Row, int32 Col)
{
	check(Row >= 0 && Row < 6 && Col >= 0 && Col < 6);
	return M[Row][Col];
}

double FRobotJacobian6x6::At(int32 Row, int32 Col) const
{
	check(Row >= 0 && Row < 6 && Col >= 0 && Col < 6);
	return M[Row][Col];
}

#pragma endregion

#pragma region ComputeNumericalJacobian

FRobotJacobian6x6 FRobotJacobian::ComputeNumericalJacobian(
	const FSerial6DoFModel& Model,
	const FRobot6DJointState& State,
	double EpsilonRad)
{
	// Epsilon 안전장치: 0 이하 또는 지나치게 작은 값(반올림 잡음 우세)은 기본값으로 대체한다.
	double Epsilon = EpsilonRad;
	if (!(Epsilon > 1e-9))
	{
		Epsilon = DefaultEpsilonRad;
	}

	FRobotJacobian6x6 Jacobian;
	Jacobian.SetZero();

	// 기준 자세의 EE 변환 (source of truth FK).
	const FTransform Current = Model.ComputeEndEffectorTransform(State);

	for (int32 JointIndex = 0; JointIndex < FSerial6DoFModel::NumJoints; ++JointIndex)
	{
		// 입력 State는 불변: 로컬 복사본만 섭동한다.
		FRobot6DJointState StatePlus = State;
		StatePlus.Q[JointIndex] += Epsilon;

		const FTransform Plus = Model.ComputeEndEffectorTransform(StatePlus);

		// delta = plus − current (ComputePoseError는 Target − Current 규약).
		// 회전 성분은 A-02 quaternion-log rotation vector로 계산된다.
		const FRobot6DPoseError Delta = FRobotPoseError::ComputePoseError(Current, Plus);

		double DeltaArray[6];
		Delta.ToArray6(DeltaArray);

		// column JointIndex = delta / epsilon.
		for (int32 Row = 0; Row < 6; ++Row)
		{
			Jacobian.M[Row][JointIndex] = DeltaArray[Row] / Epsilon;
		}
	}

	return Jacobian;
}

#pragma endregion
