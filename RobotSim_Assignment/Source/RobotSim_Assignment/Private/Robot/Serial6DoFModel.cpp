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

	// 관절 최대 토크 (N·m). Step B-01 시점의 **잠정값**이다: 아직 중력 토크를 계산할 수 없으므로
	// 근위 관절일수록 크게 두는 정도의 근사다. Step B-02에서 RNEA로 최악 자세의 정지 중력 토크를
	// 산출한 뒤 여유 2~3배가 되도록 확정한다.
	const double MaxTorquesNm[NumJoints] = { 400.0, 600.0, 350.0, 120.0, 80.0, 40.0 };
	for (int32 i = 0; i < NumJoints; ++i)
	{
		Model.JointLimits[i].MaxTorqueNm = MaxTorquesNm[i];
	}

	Model.InitializeDefaultLinkDynamics();

	return Model;
}

void FSerial6DoFModel::InitializeDefaultLinkDynamics()
{
	// 링크 i의 span = 프레임 i 원점 → 프레임 i+1 원점 (i=5는 ToolOffset).
	// LinkOffsets[i]가 아님에 주의: 관절 i의 회전은 LinkOffsets[i]만큼 떨어진 지점에서 일어나므로,
	// 관절 i에 딸려 도는 강체는 그 지점에서 "다음" 오프셋 끝까지를 차지한다.
	// LinkOffsets[0](베이스 기둥)은 J0 피벗보다 아래라 회전하지 않으므로 동역학 대상이 아니다.
	//
	// 질량/반경은 도달거리 105cm급 산업용 암을 근사한 값이며 KUKA CAD 근거가 없는 추정치다
	// (근위 링크가 무겁고 원위로 갈수록 가벼워지는 단조성만 지킨다, 총합 67kg).
	const double MassesKg[NumJoints] = { 25.0, 20.0, 12.0, 5.0, 3.0, 2.0 };
	const double RadiiM[NumJoints] = { 0.060, 0.050, 0.045, 0.035, 0.030, 0.030 };

	// 액추에이터 특성도 동일하게 잠정값이다 (Step B-06 토크 제어에서 튜닝 예정).
	const double RotorInertiasKgM2[NumJoints] = { 0.010, 0.010, 0.008, 0.004, 0.003, 0.002 };
	const double ViscousFrictionsNmsPerRad[NumJoints] = { 2.0, 2.0, 1.5, 0.8, 0.5, 0.3 };
	const double CoulombFrictionsNm[NumJoints] = { 3.0, 3.0, 2.0, 1.0, 0.7, 0.5 };

	constexpr double CmToM = 0.01;

	for (int32 i = 0; i < NumJoints; ++i)
	{
		const FVector SpanCm = (i + 1 < NumJoints) ? LinkOffsets[i + 1] : ToolOffset.GetTranslation();
		const double LengthM = SpanCm.Size() * CmToM;
		const double MassKg = MassesKg[i];
		const double RadiusM = RadiiM[i];

		FRobotLinkDynamics& Dynamics = LinkDynamics[i];
		Dynamics.MassKg = MassKg;

		// 균일 밀도 근사 → COM은 span의 중점.
		Dynamics.CenterOfMassLocalM = SpanCm * (0.5 * CmToM);

		// 균일 밀도 실린더(축 = span 방향, 반지름 r, 길이 L)의 COM 기준 관성:
		//   축 방향  I = ½ m r²
		//   횡 방향  I = (1/12) m (3r² + L²)
		const double AxialKgM2 = 0.5 * MassKg * RadiusM * RadiusM;
		const double TransverseKgM2 = (1.0 / 12.0) * MassKg * (3.0 * RadiusM * RadiusM + LengthM * LengthM);

		// 기본 로봇의 span은 전부 좌표축에 정렬돼 있으므로(±X 또는 ±Z) 텐서가 프레임 축에서 대각이 된다.
		// span 축 성분만 axial로, 나머지 두 축은 transverse로 채운다.
		FVector DiagonalKgM2(TransverseKgM2, TransverseKgM2, TransverseKgM2);
		const FVector SpanDir = SpanCm.GetSafeNormal();
		const double AbsX = FMath::Abs(SpanDir.X);
		const double AbsY = FMath::Abs(SpanDir.Y);
		const double AbsZ = FMath::Abs(SpanDir.Z);
		if (AbsX >= AbsY && AbsX >= AbsZ)
		{
			DiagonalKgM2.X = AxialKgM2;
		}
		else if (AbsY >= AbsZ)
		{
			DiagonalKgM2.Y = AxialKgM2;
		}
		else
		{
			DiagonalKgM2.Z = AxialKgM2;
		}
		Dynamics.InertiaDiagonalKgM2 = DiagonalKgM2;

		Dynamics.RotorInertiaKgM2 = RotorInertiasKgM2[i];
		Dynamics.ViscousFrictionNmsPerRad = ViscousFrictionsNmsPerRad[i];
		Dynamics.CoulombFrictionNm = CoulombFrictionsNm[i];
	}
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
