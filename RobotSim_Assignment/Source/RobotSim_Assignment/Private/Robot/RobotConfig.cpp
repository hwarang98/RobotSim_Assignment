// Fill out your copyright notice in the Description page of Project Settings.

#include "Robot/RobotConfig.h"

URobotConfig::URobotConfig()
{
	// 기구학 기본값은 코드의 단일 정의처인 CreateDefault()에서 그대로 읽어 채운다.
	// 이렇게 하면 하드코딩 숫자를 두 곳에 두지 않고, 새 에셋이 곧 기본 로봇과 동일해진다.
	const FSerial6DoFModel Default = FSerial6DoFModel::CreateDefault();

	BaseTransform = Default.BaseTransform;
	ToolOffset = Default.ToolOffset;

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		Joints[i].Axis = Default.JointAxes[i];
		Joints[i].LinkOffsetCm = Default.LinkOffsets[i];
		Joints[i].MinDeg = FMath::RadiansToDegrees(Default.JointLimits[i].MinRad);
		Joints[i].MaxDeg = FMath::RadiansToDegrees(Default.JointLimits[i].MaxRad);
		Joints[i].MaxVelDegPerSec = FMath::RadiansToDegrees(Default.JointLimits[i].MaxVelRadPerSec);
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

FSerial6DoFModel URobotConfig::ToModel() const
{
	// 비정상 축(0 벡터 등)은 기본 로봇의 대응 축으로 안전하게 대체한다.
	const FSerial6DoFModel Fallback = FSerial6DoFModel::CreateDefault();

	FSerial6DoFModel Model;
	Model.BaseTransform = BaseTransform;
	Model.ToolOffset = ToolOffset;

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
	}

	return Model;
}
