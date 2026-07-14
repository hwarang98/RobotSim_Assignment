// Fill out your copyright notice in the Description page of Project Settings.

#include "Robot/Serial6DoFModel.h"

FSerial6DoFModel FSerial6DoFModel::CreateDefault()
{
	FSerial6DoFModel Model;

	// 회전축: J0 yaw(+Z), J1/J2/J4 pitch(+Y), J3/J5 roll(+X)
	Model.JointAxes[0] = FVector::ZAxisVector;
	Model.JointAxes[1] = FVector::YAxisVector;
	Model.JointAxes[2] = FVector::YAxisVector;
	Model.JointAxes[3] = FVector::XAxisVector;
	Model.JointAxes[4] = FVector::YAxisVector;
	Model.JointAxes[5] = FVector::XAxisVector;

	// 링크 오프셋 (cm): 베이스~팔꿈치는 +Z 수직, 팔꿈치 이후는 +X 수평 전개.
	// Q=0에서 EE = (60+20+15+10, 0, 40+20+60) = (105, 0, 120)
	Model.LinkOffsets[0] = FVector(0.0, 0.0, 40.0);	// 베이스 기둥
	Model.LinkOffsets[1] = FVector(0.0, 0.0, 20.0);	// 어깨
	Model.LinkOffsets[2] = FVector(0.0, 0.0, 60.0);	// 상완 (수직)
	Model.LinkOffsets[3] = FVector(60.0, 0.0, 0.0);	// 전완 (수평)
	Model.LinkOffsets[4] = FVector(20.0, 0.0, 0.0);	// 손목
	Model.LinkOffsets[5] = FVector(15.0, 0.0, 0.0);	// 플랜지

	// 관절 한계 (KUKA류 스펙을 단순화한 값, 내부 저장은 radian)
	const double LimitsDeg[NumJoints][2] = {
		{ -185.0, 185.0 },	// J0
		{ -120.0, 120.0 },	// J1
		{ -150.0, 150.0 },	// J2
		{ -350.0, 350.0 },	// J3
		{ -120.0, 120.0 },	// J4
		{ -350.0, 350.0 }	// J5
	};
	const double MaxVelsDeg[NumJoints] = { 120.0, 115.0, 120.0, 190.0, 180.0, 260.0 };

	for (int32 i = 0; i < NumJoints; ++i)
	{
		Model.JointLimits[i].MinRad = FMath::DegreesToRadians(LimitsDeg[i][0]);
		Model.JointLimits[i].MaxRad = FMath::DegreesToRadians(LimitsDeg[i][1]);
		Model.JointLimits[i].MaxVelRadPerSec = FMath::DegreesToRadians(MaxVelsDeg[i]);
	}

	// 툴 오프셋: 플랜지에서 +X 방향 10cm. J5 회전축(+X)과 동일선상이므로
	// J5 회전은 EE 위치를 바꾸지 않고 자세만 바꾼다 (단위테스트의 불변량으로 사용).
	Model.ToolOffset = FTransform(FQuat::Identity, FVector(10.0, 0.0, 0.0));

	return Model;
}

FTransform FSerial6DoFModel::ComputeJointWorldTransform(int32 JointIndex, const FRobot6DJointState& State) const
{
	check(JointIndex >= 0 && JointIndex < NumJoints);

	// 각 관절의 로컬 변환 = 링크 오프셋 평행이동 + 관절 축 기준 회전.
	// UE FTransform 합성은 World = Local * Parent (child-first) 순서.
	FTransform Accumulated = BaseTransform;
	for (int32 i = 0; i <= JointIndex; ++i)
	{
		const FTransform JointLocal(FQuat(JointAxes[i], State.Q[i]), LinkOffsets[i]);
		Accumulated = JointLocal * Accumulated;
	}
	return Accumulated;
}

FTransform FSerial6DoFModel::ComputeEndEffectorTransform(const FRobot6DJointState& State) const
{
	return ToolOffset * ComputeJointWorldTransform(NumJoints - 1, State);
}

FRobot6DPose FSerial6DoFModel::ComputeEndEffectorPose(const FRobot6DJointState& State) const
{
	const FTransform EETransform = ComputeEndEffectorTransform(State);

	FRobot6DPose Pose;
	Pose.PositionCm = EETransform.GetLocation();
	Pose.Orientation = EETransform.GetRotation();
	return Pose;
}

FRobot6DJointState FSerial6DoFModel::ClampToLimits(const FRobot6DJointState& State) const
{
	FRobot6DJointState Clamped;
	for (int32 i = 0; i < NumJoints; ++i)
	{
		Clamped.Q[i] = JointLimits[i].Clamp(State.Q[i]);
	}
	return Clamped;
}

bool FSerial6DoFModel::IsWithinLimits(const FRobot6DJointState& State) const
{
	for (int32 i = 0; i < NumJoints; ++i)
	{
		if (!JointLimits[i].Contains(State.Q[i]))
		{
			return false;
		}
	}
	return true;
}
