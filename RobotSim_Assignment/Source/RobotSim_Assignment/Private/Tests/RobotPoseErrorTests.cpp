// Fill out your copyright notice in the Description page of Project Settings.

#include "Misc/AutomationTest.h"
#include "Robot/RobotPoseError.h"

#if WITH_DEV_AUTOMATION_TESTS

/**
 * FRobotPoseError 순수 수학 6D pose error 단위테스트.
 * 액터/월드를 전혀 사용하지 않으므로 Transform만 스택에 만들어 검증한다.
 * 회전 오차는 quaternion 기반 axis-angle(rotation vector)이며, Euler 각 차이가 아니다.
 */

namespace RobotPoseErrorTestUtils
{
	/** 위치 허용 오차 (cm) */
	constexpr double PosToleranceCm = 1e-4;

	/** 각도 허용 오차 (radian) */
	constexpr double AngToleranceRad = 1e-4;

	/** 목표를 각도 AngleRad, 축 Axis로 회전시킨 Transform (위치는 동일)을 만든다. */
	static FTransform MakeRotatedTarget(const FVector& Location, const FVector& Axis, double AngleRad)
	{
		return FTransform(FQuat(Axis, AngleRad), Location);
	}
}

//~============================================================================
// 1. 같은 Transform이면 position/rotation error가 둘 다 0이어야 한다.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotPoseErrorIdentityTest,
	"RobotSim.PoseError.Identity",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotPoseErrorIdentityTest::RunTest(const FString& Parameters)
{
	using namespace RobotPoseErrorTestUtils;

	// 회전과 위치가 모두 있는 임의 자세라도, 자기 자신과의 오차는 0이어야 한다.
	const FTransform Pose(FQuat(FVector(0.3, -0.7, 0.5).GetSafeNormal(), 1.1), FVector(42.0, -13.0, 7.0));

	const FRobot6DPoseError Error = FRobotPoseError::ComputePoseError(Pose, Pose);

	TestTrue(
		FString::Printf(TEXT("동일 자세: 위치 오차 크기 0 (실제 %.8fcm)"), Error.PositionErrorNorm()),
		Error.PositionErrorNorm() <= PosToleranceCm);
	TestTrue(
		FString::Printf(TEXT("동일 자세: 회전 오차 크기 0 (실제 %.8frad)"), Error.RotationErrorNorm()),
		Error.RotationErrorNorm() <= AngToleranceRad);

	return true;
}

//~============================================================================
// 2. 위치만 10cm 차이나면 position error만 발생하고 rotation error는 0이어야 한다.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotPoseErrorPositionOnlyTest,
	"RobotSim.PoseError.PositionOnly",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotPoseErrorPositionOnlyTest::RunTest(const FString& Parameters)
{
	using namespace RobotPoseErrorTestUtils;

	const FQuat SameRotation(FVector::YAxisVector, 0.5);	// 회전은 동일하게 유지
	const FTransform Current(SameRotation, FVector(10.0, 20.0, 30.0));
	const FTransform Target(SameRotation, FVector(20.0, 20.0, 30.0));	// +X로 10cm

	const FRobot6DPoseError Error = FRobotPoseError::ComputePoseError(Current, Target);

	TestTrue(
		FString::Printf(TEXT("위치 오차 = (+10, 0, 0) (실제 %.6f, %.6f, %.6f)"),
			Error.PositionErrorCm.X, Error.PositionErrorCm.Y, Error.PositionErrorCm.Z),
		Error.PositionErrorCm.Equals(FVector(10.0, 0.0, 0.0), PosToleranceCm));
	TestTrue(
		FString::Printf(TEXT("회전 오차 크기 0 (실제 %.8frad)"), Error.RotationErrorNorm()),
		Error.RotationErrorNorm() <= AngToleranceRad);

	return true;
}

//~============================================================================
// 3. 90도 회전 목표에서 rotation error 크기가 PI/2 근처이고 축이 맞아야 한다.
//    위치는 동일하므로 position error는 0.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotPoseErrorRotation90Test,
	"RobotSim.PoseError.Rotation90",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotPoseErrorRotation90Test::RunTest(const FString& Parameters)
{
	using namespace RobotPoseErrorTestUtils;

	const FVector Location(50.0, 0.0, 0.0);
	const FTransform Current(FQuat::Identity, Location);
	const FTransform Target = MakeRotatedTarget(Location, FVector::ZAxisVector, HALF_PI);	// +Z 90도

	const FRobot6DPoseError Error = FRobotPoseError::ComputePoseError(Current, Target);

	TestTrue(
		FString::Printf(TEXT("위치 오차 크기 0 (실제 %.8fcm)"), Error.PositionErrorNorm()),
		Error.PositionErrorNorm() <= PosToleranceCm);
	TestTrue(
		FString::Printf(TEXT("회전 오차 크기 ≈ PI/2 (실제 %.6frad)"), Error.RotationErrorNorm()),
		FMath::IsNearlyEqual(Error.RotationErrorNorm(), HALF_PI, AngToleranceRad));
	// 회전 벡터는 +Z축 × (PI/2) 이어야 한다.
	TestTrue(
		FString::Printf(TEXT("회전 오차 벡터 ≈ (0, 0, PI/2) (실제 %.6f, %.6f, %.6f)"),
			Error.RotationErrorRad.X, Error.RotationErrorRad.Y, Error.RotationErrorRad.Z),
		Error.RotationErrorRad.Equals(FVector(0.0, 0.0, HALF_PI), AngToleranceRad));

	return true;
}

//~============================================================================
// 4. 목표 회전이 -90도/270도처럼 표현되어도 shortest path 기준으로 계산되어야 한다.
//    270도(+Z)와 -90도(+Z)는 같은 회전 → 오차 크기는 3PI/2가 아니라 PI/2.
//    또한 270도(+Z) 목표는 -90도 회전과 같으므로 오차 벡터는 (0,0,-PI/2)이어야 한다.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotPoseErrorShortestPathTest,
	"RobotSim.PoseError.ShortestPath",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotPoseErrorShortestPathTest::RunTest(const FString& Parameters)
{
	using namespace RobotPoseErrorTestUtils;

	const FVector Location(50.0, 0.0, 0.0);
	const FTransform Current(FQuat::Identity, Location);

	// 두 표현이 동일한 오차를 주는지 확인: -90도와 270도(+Z).
	const FTransform TargetNeg90 = MakeRotatedTarget(Location, FVector::ZAxisVector, -HALF_PI);
	const FTransform Target270 = MakeRotatedTarget(Location, FVector::ZAxisVector, 3.0 * HALF_PI);

	const FRobot6DPoseError ErrorNeg90 = FRobotPoseError::ComputePoseError(Current, TargetNeg90);
	const FRobot6DPoseError Error270 = FRobotPoseError::ComputePoseError(Current, Target270);

	// 크기는 최단 경로 PI/2 (3PI/2 아님).
	TestTrue(
		FString::Printf(TEXT("270도 목표: 오차 크기 ≈ PI/2 (shortest, 실제 %.6frad)"), Error270.RotationErrorNorm()),
		FMath::IsNearlyEqual(Error270.RotationErrorNorm(), HALF_PI, AngToleranceRad));
	// 방향은 -Z (음의 90도).
	TestTrue(
		FString::Printf(TEXT("270도 목표: 오차 벡터 ≈ (0, 0, -PI/2) (실제 %.6f, %.6f, %.6f)"),
			Error270.RotationErrorRad.X, Error270.RotationErrorRad.Y, Error270.RotationErrorRad.Z),
		Error270.RotationErrorRad.Equals(FVector(0.0, 0.0, -HALF_PI), AngToleranceRad));
	// -90도 표현과 270도 표현이 동일한 오차 벡터를 줘야 한다.
	TestTrue(
		TEXT("-90도 표현과 270도 표현의 오차 벡터가 동일"),
		ErrorNeg90.RotationErrorRad.Equals(Error270.RotationErrorRad, AngToleranceRad));

	return true;
}

//~============================================================================
// 5. 180도 근처 회전에서 NaN/Inf가 나오지 않고 크기가 PI 근처여야 한다.
//    axis-angle 특이점 근방(sin(θ/2)≈1이지만 부호 반전 경계)에서의 안정성 검증.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotPoseErrorNearPiStableTest,
	"RobotSim.PoseError.NearPiStable",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotPoseErrorNearPiStableTest::RunTest(const FString& Parameters)
{
	using namespace RobotPoseErrorTestUtils;

	const FVector Location(50.0, 0.0, 0.0);
	const FTransform Current(FQuat::Identity, Location);
	const FVector Axis = FVector(1.0, 1.0, 0.0).GetSafeNormal();

	// 정확히 180도, 그리고 그 아래/위 미세 변위에서 모두 안정해야 한다.
	const double TestAnglesRad[] = { PI - 1e-5, PI, PI + 1e-5, PI - 1e-8 };

	for (const double AngleRad : TestAnglesRad)
	{
		const FTransform Target = MakeRotatedTarget(Location, Axis, AngleRad);
		const FRobot6DPoseError Error = FRobotPoseError::ComputePoseError(Current, Target);
		const FVector& R = Error.RotationErrorRad;

		// NaN/Inf가 절대 나오면 안 된다.
		const bool bFinite =
			FMath::IsFinite(R.X) && FMath::IsFinite(R.Y) && FMath::IsFinite(R.Z);
		TestTrue(
			FString::Printf(TEXT("θ=%.6frad: 회전 오차 성분이 유한 (실제 %.6f, %.6f, %.6f)"),
				AngleRad, R.X, R.Y, R.Z),
			bFinite);

		// 크기는 PI 근처여야 한다 (shortest path이므로 최대값이 PI).
		TestTrue(
			FString::Printf(TEXT("θ=%.6frad: 오차 크기 ≈ PI (실제 %.6frad)"), AngleRad, Error.RotationErrorNorm()),
			FMath::IsNearlyEqual(Error.RotationErrorNorm(), PI, 1e-3));
	}

	return true;
}

//~============================================================================
// 6. 가중치 적용: weighted 오차는 raw 오차 × 가중치여야 하고,
//    위치/회전 가중치가 독립적으로 적용되어야 한다.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotPoseErrorWeightedTest,
	"RobotSim.PoseError.Weighted",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotPoseErrorWeightedTest::RunTest(const FString& Parameters)
{
	using namespace RobotPoseErrorTestUtils;

	const FTransform Current(FQuat::Identity, FVector(0.0, 0.0, 0.0));
	const FTransform Target(FQuat(FVector::ZAxisVector, HALF_PI), FVector(10.0, 0.0, 0.0));

	const double PosWeight = 2.0;
	const double RotWeight = 0.5;

	const FRobot6DPoseError Raw = FRobotPoseError::ComputePoseError(Current, Target);
	const FRobot6DPoseError Weighted = FRobotPoseError::ComputeWeightedPoseError(Current, Target, PosWeight, RotWeight);

	TestTrue(
		FString::Printf(TEXT("가중 위치 오차 = raw × %.1f (실제 %.6f, %.6f, %.6f)"),
			PosWeight, Weighted.PositionErrorCm.X, Weighted.PositionErrorCm.Y, Weighted.PositionErrorCm.Z),
		Weighted.PositionErrorCm.Equals(Raw.PositionErrorCm * PosWeight, PosToleranceCm));
	TestTrue(
		FString::Printf(TEXT("가중 회전 오차 = raw × %.1f (실제 %.6f, %.6f, %.6f)"),
			RotWeight, Weighted.RotationErrorRad.X, Weighted.RotationErrorRad.Y, Weighted.RotationErrorRad.Z),
		Weighted.RotationErrorRad.Equals(Raw.RotationErrorRad * RotWeight, AngToleranceRad));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
