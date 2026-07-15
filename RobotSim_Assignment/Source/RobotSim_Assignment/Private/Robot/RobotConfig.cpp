// Fill out your copyright notice in the Description page of Project Settings.

#include "Robot/RobotConfig.h"

URobotConfig::URobotConfig()
{
	// 기구학 기본값은 코드의 단일 정의처인 CreateDefault()에서 그대로 읽어 채운다.
	// 이렇게 하면 하드코딩 숫자를 두 곳에 두지 않고, 새 에셋이 곧 기본 로봇과 동일해진다.
	const FSerial6DoFModel Default = FSerial6DoFModel::CreateDefault();

	BaseTransform = Default.BaseTransform;
	ToolOffset = Default.ToolOffset;
	GravityMPerSec2 = Default.GravityMPerSec2;

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		Joints[i].Axis = Default.JointAxes[i];
		Joints[i].LinkOffsetCm = Default.LinkOffsets[i];
		Joints[i].MinDeg = FMath::RadiansToDegrees(Default.JointLimits[i].MinRad);
		Joints[i].MaxDeg = FMath::RadiansToDegrees(Default.JointLimits[i].MaxRad);
		Joints[i].MaxVelDegPerSec = FMath::RadiansToDegrees(Default.JointLimits[i].MaxVelRadPerSec);
		Joints[i].MaxTorqueNm = Default.JointLimits[i].MaxTorqueNm;

		// 동역학 값은 authoring 단위가 이미 SI라 변환 없이 그대로 옮긴다.
		const FRobotLinkDynamics& DefaultDynamics = Default.LinkDynamics[i];
		LinkDynamics[i].MassKg = DefaultDynamics.MassKg;
		LinkDynamics[i].CenterOfMassLocalM = DefaultDynamics.CenterOfMassLocalM;
		LinkDynamics[i].InertiaDiagonalKgM2 = DefaultDynamics.InertiaDiagonalKgM2;
		LinkDynamics[i].RotorInertiaKgM2 = DefaultDynamics.RotorInertiaKgM2;
		LinkDynamics[i].ViscousFrictionNmsPerRad = DefaultDynamics.ViscousFrictionNmsPerRad;
		LinkDynamics[i].CoulombFrictionNm = DefaultDynamics.CoulombFrictionNm;
	}

	// 확정된 KUKA 본 매핑 기본값 (Bone Probe로 확정, STEP_A-01.5b 참조).
	// J3(전완 roll)는 메시에 대응 자유도가 없어 None (시각화에서 skip, 수학 FK에는 그대로 존재).
	JointBoneNames[0] = TEXT("Bone_001"); // J0 베이스 yaw
	JointBoneNames[1] = TEXT("Bone_002"); // J1 어깨 pitch
	JointBoneNames[2] = TEXT("Bone_004"); // J2 팔꿈치 pitch
	JointBoneNames[3] = NAME_None;        // J3 전완 roll — visual-only unmapped
	JointBoneNames[4] = TEXT("Bone_006"); // J4 손목 pitch
	JointBoneNames[5] = TEXT("Bone_007"); // J5 툴 roll
}

namespace RobotConfigSanitize
{
	/** 유한하고 양수인 값만 통과시키고, 아니면 폴백으로 대체한다 (질량/관성/토크용). */
	static double PositiveOrFallback(double Value, double Fallback)
	{
		return (FMath::IsFinite(Value) && Value > 0.0) ? Value : Fallback;
	}

	/** 유한하고 0 이상인 값만 통과시킨다 (마찰/로터 관성용 — 0은 "미모델링"이라 유효). */
	static double NonNegativeOrFallback(double Value, double Fallback)
	{
		return (FMath::IsFinite(Value) && Value >= 0.0) ? Value : Fallback;
	}

	/** 유한한 벡터만 통과시킨다 (COM은 음수/0 성분도 물리적으로 유효). */
	static FVector FiniteOrFallback(const FVector& Value, const FVector& Fallback)
	{
		return (!Value.ContainsNaN() && FMath::IsFinite(Value.X) && FMath::IsFinite(Value.Y) && FMath::IsFinite(Value.Z))
			? Value
			: Fallback;
	}

	/** 세 성분이 모두 유한하고 양수인 벡터만 통과시킨다 (관성 대각 성분용). */
	static FVector PositiveComponentsOrFallback(const FVector& Value, const FVector& Fallback)
	{
		const bool bValid =
			FMath::IsFinite(Value.X) && Value.X > 0.0 &&
			FMath::IsFinite(Value.Y) && Value.Y > 0.0 &&
			FMath::IsFinite(Value.Z) && Value.Z > 0.0;
		return bValid ? Value : Fallback;
	}
}

FSerial6DoFModel URobotConfig::ToModel() const
{
	using namespace RobotConfigSanitize;

	// 비정상 축(0 벡터 등)과 물리적으로 불가능한 동역학 값은 기본 로봇의 대응 값으로 안전하게 대체한다.
	const FSerial6DoFModel Fallback = FSerial6DoFModel::CreateDefault();

	FSerial6DoFModel Model;
	Model.BaseTransform = BaseTransform;
	Model.ToolOffset = ToolOffset;
	Model.GravityMPerSec2 = FiniteOrFallback(GravityMPerSec2, Fallback.GravityMPerSec2);

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		const FVector NormalizedAxis = Joints[i].Axis.GetSafeNormal();
		Model.JointAxes[i] = NormalizedAxis.IsNearlyZero() ? Fallback.JointAxes[i] : NormalizedAxis;
		Model.LinkOffsets[i] = Joints[i].LinkOffsetCm;

		// 도→radian 변환은 이 경계에서만. Min/Max가 뒤집혀 있어도 방어적으로 정렬한다.
		const double MinRad = FMath::DegreesToRadians(FMath::Min(Joints[i].MinDeg, Joints[i].MaxDeg));
		const double MaxRad = FMath::DegreesToRadians(FMath::Max(Joints[i].MinDeg, Joints[i].MaxDeg));
		Model.JointLimits[i].MinRad = MinRad;
		Model.JointLimits[i].MaxRad = MaxRad;
		Model.JointLimits[i].MaxVelRadPerSec = FMath::DegreesToRadians(FMath::Abs(Joints[i].MaxVelDegPerSec));
		Model.JointLimits[i].MaxTorqueNm = PositiveOrFallback(Joints[i].MaxTorqueNm, Fallback.JointLimits[i].MaxTorqueNm);

		// 동역학 값은 authoring 단위가 이미 SI → 변환 없이 복사하고, 물리적으로 불가능한 값만 방어한다.
		const FRobotLinkDynamicsConfig& SourceDynamics = LinkDynamics[i];
		const FRobotLinkDynamics& FallbackDynamics = Fallback.LinkDynamics[i];
		FRobotLinkDynamics& TargetDynamics = Model.LinkDynamics[i];

		TargetDynamics.MassKg = PositiveOrFallback(SourceDynamics.MassKg, FallbackDynamics.MassKg);
		TargetDynamics.CenterOfMassLocalM = FiniteOrFallback(SourceDynamics.CenterOfMassLocalM, FallbackDynamics.CenterOfMassLocalM);
		TargetDynamics.InertiaDiagonalKgM2 = PositiveComponentsOrFallback(SourceDynamics.InertiaDiagonalKgM2, FallbackDynamics.InertiaDiagonalKgM2);
		TargetDynamics.RotorInertiaKgM2 = NonNegativeOrFallback(SourceDynamics.RotorInertiaKgM2, FallbackDynamics.RotorInertiaKgM2);
		TargetDynamics.ViscousFrictionNmsPerRad = NonNegativeOrFallback(SourceDynamics.ViscousFrictionNmsPerRad, FallbackDynamics.ViscousFrictionNmsPerRad);
		TargetDynamics.CoulombFrictionNm = NonNegativeOrFallback(SourceDynamics.CoulombFrictionNm, FallbackDynamics.CoulombFrictionNm);
	}

	return Model;
}
