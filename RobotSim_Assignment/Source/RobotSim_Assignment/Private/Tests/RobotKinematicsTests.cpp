// Fill out your copyright notice in the Description page of Project Settings.

#include "Misc/AutomationTest.h"
#include "Robot/Serial6DoFModel.h"

#if WITH_DEV_AUTOMATION_TESTS

/**
 * FSerial6DoFModel 순수 수학 FK 단위테스트.
 * 액터/월드를 전혀 사용하지 않으므로 스폰 없이 모델만 스택에 생성해 검증한다.
 * 골든 값은 CreateDefault() 파라미터로부터 손으로 계산한 값이다 (계획 문서 참조).
 */

namespace RobotKinematicsTestUtils
{
	/** 위치 허용 오차 (cm). 해석적 double 연산이므로 실제 오차는 이보다 훨씬 작다. */
	constexpr double PosToleranceCm = 1e-3;

	/** 각도 허용 오차 (radian) */
	constexpr double AngToleranceRad = 1e-4;

	/** 두 위치 벡터가 허용 오차 내에서 일치하는지 검증한다. */
	static void TestPositionNear(FAutomationTestBase& Test, const FString& What, const FVector& Actual, const FVector& Expected)
	{
		const double Error = FVector::Distance(Actual, Expected);
		Test.TestTrue(
			FString::Printf(TEXT("%s: 기대 위치 (%.4f, %.4f, %.4f), 실제 위치 (%.4f, %.4f, %.4f), 오차 %.6fcm"),
				*What, Expected.X, Expected.Y, Expected.Z, Actual.X, Actual.Y, Actual.Z, Error),
			Error <= PosToleranceCm);
	}

	/** 두 쿼터니언이 허용 각도 오차 내에서 일치하는지 검증한다. */
	static void TestOrientationNear(FAutomationTestBase& Test, const FString& What, const FQuat& Actual, const FQuat& Expected)
	{
		const double AngularError = Actual.AngularDistance(Expected);
		Test.TestTrue(
			FString::Printf(TEXT("%s: 자세 각도 오차 %.8frad"), *What, AngularError),
			AngularError <= AngToleranceRad);
	}
}

//~============================================================================
// 1. Q=0 FK: End Effector가 기하 파라미터 합산 위치 (105, 0, 120)에 있어야 한다.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotFKZeroPoseTest,
	"RobotSim.Kinematics.ZeroPoseFK",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotFKZeroPoseTest::RunTest(const FString& Parameters)
{
	using namespace RobotKinematicsTestUtils;

	const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();
	const FRobot6DJointState ZeroState;

	const FRobot6DPose EEPose = Model.ComputeEndEffectorPose(ZeroState);

	// X = 60 + 20 + 15 + 10 = 105, Z = 40 + 20 + 60 = 120
	TestPositionNear(*this, TEXT("Q=0 EE 위치"), EEPose.PositionCm, FVector(105.0, 0.0, 120.0));
	TestOrientationNear(*this, TEXT("Q=0 EE 자세는 Identity"), EEPose.Orientation, FQuat::Identity);

	return true;
}

//~============================================================================
// 2. J0 yaw: +Z축 회전이므로 EE가 Z축 둘레를 돌아야 하며,
//    반경 sqrt(x^2+y^2)=105와 높이 z=120은 어떤 각도에서도 불변이어야 한다.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotFKJ0YawTest,
	"RobotSim.Kinematics.J0YawRotatesAboutZ",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotFKJ0YawTest::RunTest(const FString& Parameters)
{
	using namespace RobotKinematicsTestUtils;

	const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();

	// J0 = +90도: +X → +Y 이므로 (105, 0, 120) → (0, 105, 120)
	{
		FRobot6DJointState State;
		State.Q[0] = FMath::DegreesToRadians(90.0);

		const FRobot6DPose EEPose = Model.ComputeEndEffectorPose(State);
		TestPositionNear(*this, TEXT("J0=90도 EE 위치"), EEPose.PositionCm, FVector(0.0, 105.0, 120.0));
	}

	// J0 = 임의 각도(37도): base yaw만 변하므로 반경과 높이가 보존되어야 한다.
	{
		FRobot6DJointState State;
		State.Q[0] = FMath::DegreesToRadians(37.0);

		const FRobot6DPose EEPose = Model.ComputeEndEffectorPose(State);
		const double Radius = FMath::Sqrt(FMath::Square(EEPose.PositionCm.X) + FMath::Square(EEPose.PositionCm.Y));

		TestTrue(
			FString::Printf(TEXT("J0=37도: XY 반경 보존 (기대 105, 실제 %.6f)"), Radius),
			FMath::IsNearlyEqual(Radius, 105.0, PosToleranceCm));
		TestTrue(
			FString::Printf(TEXT("J0=37도: 높이 보존 (기대 120, 실제 %.6f)"), EEPose.PositionCm.Z),
			FMath::IsNearlyEqual(EEPose.PositionCm.Z, 120.0, PosToleranceCm));
	}

	return true;
}

//~============================================================================
// 3. J1 pitch: 어깨 피벗 (0,0,60) 기준 +Y축 회전.
//    +90도에서 원위 체인 (105,0,60)이 (60,0,-105)로 회전 → EE = (60, 0, -45).
//    피벗으로부터의 거리도 보존되어야 한다.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotFKJ1PitchTest,
	"RobotSim.Kinematics.J1PitchMovesEE",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotFKJ1PitchTest::RunTest(const FString& Parameters)
{
	using namespace RobotKinematicsTestUtils;

	const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();
	const FVector ShoulderPivot(0.0, 0.0, 60.0);
	const double ExpectedDistance = FVector(105.0, 0.0, 60.0).Length();

	// J1 = +90도: +Y축 회전은 +X → -Z 이므로 (105,0,60) → (60,0,-105)
	{
		FRobot6DJointState State;
		State.Q[1] = FMath::DegreesToRadians(90.0);

		const FRobot6DPose EEPose = Model.ComputeEndEffectorPose(State);
		TestPositionNear(*this, TEXT("J1=90도 EE 위치"), EEPose.PositionCm, FVector(60.0, 0.0, -45.0));

		const double Distance = FVector::Distance(EEPose.PositionCm, ShoulderPivot);
		TestTrue(
			FString::Printf(TEXT("J1=90도: 어깨 피벗 거리 보존 (기대 %.6f, 실제 %.6f)"), ExpectedDistance, Distance),
			FMath::IsNearlyEqual(Distance, ExpectedDistance, PosToleranceCm));
	}

	// J1 = 임의 각도(-50도)에서도 어깨 피벗 거리는 보존되어야 한다.
	{
		FRobot6DJointState State;
		State.Q[1] = FMath::DegreesToRadians(-50.0);

		const FRobot6DPose EEPose = Model.ComputeEndEffectorPose(State);
		const double Distance = FVector::Distance(EEPose.PositionCm, ShoulderPivot);
		TestTrue(
			FString::Printf(TEXT("J1=-50도: 어깨 피벗 거리 보존 (기대 %.6f, 실제 %.6f)"), ExpectedDistance, Distance),
			FMath::IsNearlyEqual(Distance, ExpectedDistance, PosToleranceCm));
	}

	return true;
}

//~============================================================================
// 4. J5 roll: 툴 오프셋 (10,0,0)이 J5 회전축 +X와 동일선상이므로
//    J5 회전은 EE 위치를 바꾸지 않고 자세만 FQuat(+X, theta)로 바꿔야 한다.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotFKToolRollTest,
	"RobotSim.Kinematics.ToolRollInvariance",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotFKToolRollTest::RunTest(const FString& Parameters)
{
	using namespace RobotKinematicsTestUtils;

	const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();
	const double TestAnglesDeg[] = { 45.0, 170.0 };

	for (const double AngleDeg : TestAnglesDeg)
	{
		FRobot6DJointState State;
		State.Q[5] = FMath::DegreesToRadians(AngleDeg);

		const FRobot6DPose EEPose = Model.ComputeEndEffectorPose(State);

		TestPositionNear(*this,
			FString::Printf(TEXT("J5=%.0f도 EE 위치 불변"), AngleDeg),
			EEPose.PositionCm, FVector(105.0, 0.0, 120.0));

		const FQuat ExpectedOrientation(FVector::XAxisVector, FMath::DegreesToRadians(AngleDeg));
		TestOrientationNear(*this,
			FString::Printf(TEXT("J5=%.0f도 EE 자세"), AngleDeg),
			EEPose.Orientation, ExpectedOrientation);
	}

	return true;
}

//~============================================================================
// 5. 관절 한계 클램프: 범위 초과 각도는 정확히 Min/Max로 잘리고,
//    범위 내 각도는 변하지 않아야 한다. IsWithinLimits도 함께 검증한다.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotJointLimitClampTest,
	"RobotSim.Kinematics.JointLimitClamp",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotJointLimitClampTest::RunTest(const FString& Parameters)
{
	using namespace RobotKinematicsTestUtils;

	const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();

	FRobot6DJointState OverLimitState;
	OverLimitState.Q[0] = FMath::DegreesToRadians(-400.0);	// J0 하한(-185도) 초과
	OverLimitState.Q[1] = FMath::DegreesToRadians(200.0);	// J1 상한(+120도) 초과
	OverLimitState.Q[2] = FMath::DegreesToRadians(10.0);	// 범위 내

	TestFalse(TEXT("클램프 전: 범위 초과 상태는 IsWithinLimits가 false여야 함"), Model.IsWithinLimits(OverLimitState));

	const FRobot6DJointState Clamped = Model.ClampToLimits(OverLimitState);

	TestTrue(
		FString::Printf(TEXT("J0: -400도 입력이 하한 -185도로 클램프 (실제 %.6f도)"), FMath::RadiansToDegrees(Clamped.Q[0])),
		FMath::IsNearlyEqual(Clamped.Q[0], Model.JointLimits[0].MinRad, AngToleranceRad));
	TestTrue(
		FString::Printf(TEXT("J1: 200도 입력이 상한 120도로 클램프 (실제 %.6f도)"), FMath::RadiansToDegrees(Clamped.Q[1])),
		FMath::IsNearlyEqual(Clamped.Q[1], Model.JointLimits[1].MaxRad, AngToleranceRad));
	TestTrue(
		FString::Printf(TEXT("J2: 범위 내 10도 입력은 변경 없음 (실제 %.6f도)"), FMath::RadiansToDegrees(Clamped.Q[2])),
		FMath::IsNearlyEqual(Clamped.Q[2], FMath::DegreesToRadians(10.0), AngToleranceRad));

	TestTrue(TEXT("클램프 후: IsWithinLimits가 true여야 함"), Model.IsWithinLimits(Clamped));

	// 나머지 관절(입력 0)은 0으로 유지되어야 한다.
	for (int32 i = 3; i < FSerial6DoFModel::NumJoints; ++i)
	{
		TestTrue(
			FString::Printf(TEXT("J%d: 0 입력은 변경 없음"), i),
			FMath::IsNearlyEqual(Clamped.Q[i], 0.0, AngToleranceRad));
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
