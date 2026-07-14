// Fill out your copyright notice in the Description page of Project Settings.

#include "Robot/RobotPoseError.h"

FVector FRobotPoseError::ComputeRotationError(const FQuat& Current, const FQuat& Target)
{
	// 월드 프레임 상대 회전: current를 target으로 돌리는 회전.
	// (A-03 geometric Jacobian이 월드 프레임 각속도를 쓰는 것과 정합)
	FQuat QErr = Target * Current.Inverse();
	QErr.Normalize();

	// Shortest path: q와 -q는 같은 회전이지만 반구가 다르다.
	// W >= 0 반구를 고르면 회전각이 [0, π]로 제한되어 최단 경로가 된다.
	if (QErr.W < 0.0)
	{
		QErr = FQuat(-QErr.X, -QErr.Y, -QErr.Z, -QErr.W);
	}

	// vec(q_err) = axis * sin(θ/2), W = cos(θ/2).
	const FVector Vec(QErr.X, QErr.Y, QErr.Z);
	const double SinHalfAngle = Vec.Size();	// = |sin(θ/2)| >= 0
	const double CosHalfAngle = QErr.W;		// W >= 0 (위 반구 보정)

	// θ ≈ 0: 나눗셈(1/SinHalfAngle)이 불안정하므로 small-angle 근사.
	// rotvec = axis*θ = (Vec/|Vec|) * 2*asin(|Vec|) → |Vec|→0에서 2*Vec로 수렴.
	if (SinHalfAngle < UE_SMALL_NUMBER)
	{
		return Vec * 2.0;
	}

	// atan2를 쓰면 acos(W)의 정의역 초과(부동소수 W>1) NaN을 원천 차단한다.
	// 180° 근처: SinHalfAngle≈1, CosHalfAngle≈0 → HalfAngle≈π/2 → Angle≈π (안정).
	const double HalfAngle = FMath::Atan2(SinHalfAngle, CosHalfAngle);	// ∈ [0, π/2]
	const double Angle = 2.0 * HalfAngle;								// ∈ [0, π]

	return (Vec / SinHalfAngle) * Angle;
}

FRobot6DPoseError FRobotPoseError::ComputePoseError(const FTransform& Current, const FTransform& Target)
{
	FRobot6DPoseError Error;
	Error.PositionErrorCm = Target.GetLocation() - Current.GetLocation();
	Error.RotationErrorRad = ComputeRotationError(Current.GetRotation(), Target.GetRotation());
	return Error;
}

FRobot6DPoseError FRobotPoseError::ComputeWeightedPoseError(
	const FTransform& Current, const FTransform& Target,
	double PositionWeight, double RotationWeight)
{
	FRobot6DPoseError Error = ComputePoseError(Current, Target);
	Error.PositionErrorCm *= PositionWeight;
	Error.RotationErrorRad *= RotationWeight;
	return Error;
}
