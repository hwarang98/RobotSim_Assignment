// Fill out your copyright notice in the Description page of Project Settings.

#include "Misc/AutomationTest.h"
#include "Robot/RobotDlsIK.h"
#include "Robot/RobotPoseError.h"
#include "Robot/Serial6DoFModel.h"

#if WITH_DEV_AUTOMATION_TESTS

/**
 * Nullspace joint-limit avoidance(STEP A-05) 순수 수학 단위테스트.
 * 액터/월드를 전혀 사용하지 않으므로 스폰 없이 모델만 스택에 생성해 검증한다.
 *
 * 방향성(중립 쪽으로 되돌리는가)과 finite 안정성 위주로 느슨하게 검증한다.
 * 주의: 6R 로봇 + 6D task는 본질적으로 비여유(non-redundant)라 정확 해 근처에서 nullspace가
 * 거의 비어 있다. 따라서 solver 레벨 테스트(e)는 "악화되지 않음(does not worsen)"만 요구한다.
 */

namespace RobotNullspaceTestUtils
{
	static FSerial6DoFModel MakeModel()
	{
		return FSerial6DoFModel::CreateDefault();
	}

	/** 관절 i를 그 가동 범위의 Ratio(예: +0.9) 지점에 두고 나머지는 midpoint(0)로 둔 상태를 만든다. */
	static FRobot6DJointState MakeStateAtJointRatio(const FSerial6DoFModel& Model, int32 JointIndex, double Ratio)
	{
		FRobot6DJointState State;
		const FRobotJointLimit& Limit = Model.JointLimits[JointIndex];
		const double Midpoint = 0.5 * (Limit.MinRad + Limit.MaxRad);
		const double HalfRange = 0.5 * (Limit.MaxRad - Limit.MinRad);
		State.Q[JointIndex] = Midpoint + Ratio * HalfRange;
		return State;
	}
}

//~============================================================================
// a. DisabledByDefault — 옵션 기본값에서 nullspace가 꺼져 있어야 함
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotNullspaceDisabledByDefaultTest,
	"RobotSim.IK.Nullspace.DisabledByDefault",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotNullspaceDisabledByDefaultTest::RunTest(const FString& Parameters)
{
	const FRobotDlsIKOptions Options;
	TestFalse(TEXT("bUseNullspaceJointLimitAvoidance 기본값은 false여야 함"),
		Options.bUseNullspaceJointLimitAvoidance);

	return true;
}

//~============================================================================
// b. GradientPointsTowardMidpoint — 한계 근처에서 회피 속도가 중립 방향을 향함
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotNullspaceGradientTowardMidpointTest,
	"RobotSim.IK.Nullspace.GradientPointsTowardMidpoint",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotNullspaceGradientTowardMidpointTest::RunTest(const FString& Parameters)
{
	using namespace RobotNullspaceTestUtils;

	const FSerial6DoFModel Model = MakeModel();
	FRobotDlsIKOptions Options;
	Options.bUseNullspaceJointLimitAvoidance = true;

	// 상한 근처(+0.9): normalized > 0 → 중립 방향은 음수.
	{
		const FRobot6DJointState State = MakeStateAtJointRatio(Model, /*Joint*/ 0, /*Ratio*/ 0.9);
		double Vel[6];
		FRobotDlsIK::ComputeJointLimitAvoidanceVelocity(Model, State, Options, Vel);
		TestTrue(
			FString::Printf(TEXT("상한 근처 J0 회피 속도는 음수여야 함, 값 %g"), Vel[0]),
			Vel[0] < 0.0);
	}

	// 하한 근처(-0.9): normalized < 0 → 중립 방향은 양수.
	{
		const FRobot6DJointState State = MakeStateAtJointRatio(Model, /*Joint*/ 0, /*Ratio*/ -0.9);
		double Vel[6];
		FRobotDlsIK::ComputeJointLimitAvoidanceVelocity(Model, State, Options, Vel);
		TestTrue(
			FString::Printf(TEXT("하한 근처 J0 회피 속도는 양수여야 함, 값 %g"), Vel[0]),
			Vel[0] > 0.0);
	}

	return true;
}

//~============================================================================
// c. NoActivationNearMidpoint — 중립 근처에서는 회피 속도가 거의 0
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotNullspaceNoActivationNearMidpointTest,
	"RobotSim.IK.Nullspace.NoActivationNearMidpoint",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotNullspaceNoActivationNearMidpointTest::RunTest(const FString& Parameters)
{
	using namespace RobotNullspaceTestUtils;

	const FSerial6DoFModel Model = MakeModel();
	FRobotDlsIKOptions Options;
	Options.bUseNullspaceJointLimitAvoidance = true;

	// 모든 관절이 midpoint(대칭 한계이므로 0). activation ratio(0.65) 미만이므로 전부 0이어야 함.
	const FRobot6DJointState MidpointState; // 기본 Q = 모두 0.
	double Vel[6];
	FRobotDlsIK::ComputeJointLimitAvoidanceVelocity(Model, MidpointState, Options, Vel);

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		TestTrue(
			FString::Printf(TEXT("중립 근처 J%d 회피 속도는 0이어야 함, 값 %g"), i, Vel[i]),
			FMath::IsNearlyZero(Vel[i], 1e-9));
	}

	return true;
}

//~============================================================================
// d. SolverKeepsFinite — nullspace on 상태로 어려운 target에서도 NaN/Inf 없음
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotNullspaceSolverKeepsFiniteTest,
	"RobotSim.IK.Nullspace.SolverKeepsFinite",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotNullspaceSolverKeepsFiniteTest::RunTest(const FString& Parameters)
{
	using namespace RobotNullspaceTestUtils;

	const FSerial6DoFModel Model = MakeModel();
	const FRobot6DJointState ZeroState;

	FRobotDlsIKOptions Options;
	Options.bUseNullspaceJointLimitAvoidance = true;

	// 명백히 도달 불가한 아주 먼 목표 + 임의 회전.
	const FTransform Target(FQuat(FVector(1.0, 1.0, 1.0).GetSafeNormal(), FMath::DegreesToRadians(150.0)),
		FVector(100000.0, -50000.0, 75000.0));

	const FRobotDlsIKResult Result = FRobotDlsIK::SolveDlsIK(Model, ZeroState, Target, Options);

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		TestTrue(
			FString::Printf(TEXT("Solution.Q[%d]가 finite여야 함, 값 %f"), i, Result.Solution.Q[i]),
			FMath::IsFinite(Result.Solution.Q[i]));
	}

	TestTrue(TEXT("최종 위치오차 크기가 finite여야 함"), FMath::IsFinite(Result.FinalPositionErrorCm));
	TestTrue(TEXT("최종 회전오차 크기가 finite여야 함"), FMath::IsFinite(Result.FinalRotationErrorRad));
	TestTrue(TEXT("nullspace step norm이 finite여야 함"), FMath::IsFinite(Result.NullspaceStepNorm));
	TestTrue(TEXT("max 관절 편차가 finite여야 함"), FMath::IsFinite(Result.MaxAbsNormalizedJointDistance));

	return true;
}

//~============================================================================
// e. JointLimitDistanceDoesNotWorsenOnRedundantOrWeakTask
//    한계 근처 초기 상태 + 약한 task에서 nullspace on이 관절 편차를 악화시키지 않아야 함
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotNullspaceDoesNotWorsenTest,
	"RobotSim.IK.Nullspace.JointLimitDistanceDoesNotWorsenOnRedundantOrWeakTask",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotNullspaceDoesNotWorsenTest::RunTest(const FString& Parameters)
{
	using namespace RobotNullspaceTestUtils;

	const FSerial6DoFModel Model = MakeModel();

	// J0를 상한의 90% 지점에 둔 초기 상태(나머지는 중립). max 관절 편차 ≈ 0.9.
	const FRobot6DJointState StartState = MakeStateAtJointRatio(Model, /*Joint*/ 0, /*Ratio*/ 0.9);

	// 약한 task: 현재 EE에서 아주 살짝(+2cm Y)만 이동한, 도달 가능한 근접 목표.
	FTransform Target = Model.ComputeEndEffectorTransform(StartState);
	Target.AddToTranslation(FVector(0.0, 2.0, 0.0));

	FRobotDlsIKOptions OffOptions;
	OffOptions.bUseNullspaceJointLimitAvoidance = false;
	const FRobotDlsIKResult OffResult = FRobotDlsIK::SolveDlsIK(Model, StartState, Target, OffOptions);

	FRobotDlsIKOptions OnOptions;
	OnOptions.bUseNullspaceJointLimitAvoidance = true;
	const FRobotDlsIKResult OnResult = FRobotDlsIK::SolveDlsIK(Model, StartState, Target, OnOptions);

	// nullspace는 중립 방향으로만 밀므로 관절 편차를 악화시키지 않아야 한다(느슨한 허용오차 eps).
	constexpr double Eps = 0.02;
	TestTrue(
		FString::Printf(TEXT("nullspace on의 max관절편차(%.4f)가 off(%.4f)보다 악화되지 않아야 함"),
			OnResult.MaxAbsNormalizedJointDistance, OffResult.MaxAbsNormalizedJointDistance),
		OnResult.MaxAbsNormalizedJointDistance <= OffResult.MaxAbsNormalizedJointDistance + Eps);

	// 두 경우 모두 task는 여전히 잘 추종해야 한다(nullspace가 task 수렴을 크게 해치지 않음).
	TestTrue(
		FString::Printf(TEXT("nullspace on에서도 위치오차가 작게 유지되어야 함, %.3fcm"),
			OnResult.FinalPositionErrorCm),
		OnResult.FinalPositionErrorCm <= OffResult.FinalPositionErrorCm + 1.0);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
