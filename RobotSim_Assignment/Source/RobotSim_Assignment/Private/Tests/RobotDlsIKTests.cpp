// Fill out your copyright notice in the Description page of Project Settings.

#include "Misc/AutomationTest.h"
#include "Robot/RobotDlsIK.h"
#include "Robot/RobotPoseError.h"
#include "Robot/Serial6DoFModel.h"

#if WITH_DEV_AUTOMATION_TESTS

/**
 * FRobotDlsIK(Damped Least Squares IK) 순수 수학 단위테스트.
 * 액터/월드를 전혀 사용하지 않으므로 스폰 없이 모델만 스택에 생성해 검증한다.
 *
 * 주의: numerical Jacobian + DLS는 초기값/weight에 민감하므로, "완전 수렴"보다
 * "오차 감소"를 검증하는 느슨한 테스트를 우선한다.
 */

namespace RobotIKTestUtils
{
	/** 오차 감소 판정: 최종 오차가 초기 오차의 이 비율 미만이면 "크게 줄었다"로 본다. */
	constexpr double SignificantReductionRatio = 0.5;

	/** Q=0 기준 모델과 그 EE 목표 Transform을 만든다. */
	static FSerial6DoFModel MakeModel()
	{
		return FSerial6DoFModel::CreateDefault();
	}

	/** 목표 Transform에 대해 Q=0에서 시작한 IK 결과를 반환한다(기본 옵션). */
	static FRobotDlsIKResult SolveFromZero(const FSerial6DoFModel& Model, const FTransform& Target)
	{
		const FRobot6DJointState ZeroState;
		const FRobotDlsIKOptions Options;
		return FRobotDlsIK::SolveDlsIK(Model, ZeroState, Target, Options);
	}
}

//~============================================================================
// a. AlreadyAtTarget — Q=0의 FK를 목표로 넣으면 0~1회 내에 수렴
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotIKAlreadyAtTargetTest,
	"RobotSim.IK.AlreadyAtTarget",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotIKAlreadyAtTargetTest::RunTest(const FString& Parameters)
{
	using namespace RobotIKTestUtils;

	const FSerial6DoFModel Model = MakeModel();
	const FRobot6DJointState ZeroState;
	const FTransform Target = Model.ComputeEndEffectorTransform(ZeroState);

	const FRobotDlsIKResult Result = SolveFromZero(Model, Target);

	TestTrue(TEXT("이미 목표에 있으므로 수렴해야 함"), Result.bConverged);
	TestTrue(
		FString::Printf(TEXT("0~1회 반복 내 수렴해야 함, 실제 %d회"), Result.Iterations),
		Result.Iterations <= 1);

	return true;
}

//~============================================================================
// b. SmallReachablePositionOffset — Y +10cm 이동 시 위치오차가 크게 줄어드는지
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotIKSmallReachablePositionOffsetTest,
	"RobotSim.IK.SmallReachablePositionOffset",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotIKSmallReachablePositionOffsetTest::RunTest(const FString& Parameters)
{
	using namespace RobotIKTestUtils;

	const FSerial6DoFModel Model = MakeModel();
	const FRobot6DJointState ZeroState;

	// Q=0의 EE에서 Y로 +10cm 이동시킨 목표.
	FTransform Target = Model.ComputeEndEffectorTransform(ZeroState);
	Target.AddToTranslation(FVector(0.0, 10.0, 0.0));

	// 초기 위치오차(= 10cm)를 명시적으로 계산.
	const FTransform StartTransform = Model.ComputeEndEffectorTransform(ZeroState);
	const double InitialPosErr =
		FRobotPoseError::ComputePoseError(StartTransform, Target).PositionErrorNorm();

	const FRobotDlsIKResult Result = SolveFromZero(Model, Target);

	// 수렴했거나, 최소한 위치오차가 초기의 절반 미만으로 줄어야 한다.
	const bool bReduced = Result.FinalPositionErrorCm < InitialPosErr * SignificantReductionRatio;
	TestTrue(
		FString::Printf(TEXT("수렴 또는 위치오차 감소 필요: 초기 %.3fcm → 최종 %.3fcm (수렴=%d)"),
			InitialPosErr, Result.FinalPositionErrorCm, Result.bConverged ? 1 : 0),
		Result.bConverged || bReduced);

	return true;
}

//~============================================================================
// c. SmallReachableOrientationOffset — tool 10도 회전 시 회전오차가 줄어드는지
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotIKSmallReachableOrientationOffsetTest,
	"RobotSim.IK.SmallReachableOrientationOffset",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotIKSmallReachableOrientationOffsetTest::RunTest(const FString& Parameters)
{
	using namespace RobotIKTestUtils;

	const FSerial6DoFModel Model = MakeModel();
	const FRobot6DJointState ZeroState;

	// Q=0의 EE 자세에 tool roll(+X축) 10도를 추가한 목표.
	const FTransform StartTransform = Model.ComputeEndEffectorTransform(ZeroState);
	const FQuat DeltaRot(FVector::XAxisVector, FMath::DegreesToRadians(10.0));
	FTransform Target = StartTransform;
	Target.SetRotation((StartTransform.GetRotation() * DeltaRot).GetNormalized());

	const double InitialRotErr =
		FRobotPoseError::ComputePoseError(StartTransform, Target).RotationErrorNorm();

	const FRobotDlsIKResult Result = SolveFromZero(Model, Target);

	// 회전오차가 초기보다 확실히 줄어야 한다(엄밀 수렴은 요구하지 않음).
	TestTrue(
		FString::Printf(TEXT("회전오차 감소 필요: 초기 %.4frad → 최종 %.4frad"),
			InitialRotErr, Result.FinalRotationErrorRad),
		Result.FinalRotationErrorRad < InitialRotErr);

	return true;
}

//~============================================================================
// d. JointLimitClamp — 결과 관절이 가동 범위를 벗어나지 않는지
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotIKJointLimitClampTest,
	"RobotSim.IK.JointLimitClamp",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotIKJointLimitClampTest::RunTest(const FString& Parameters)
{
	using namespace RobotIKTestUtils;

	const FSerial6DoFModel Model = MakeModel();

	// 큰 스텝을 유도하도록 멀리 있는(도달 불가에 가까운) 목표.
	FTransform Target(FQuat(FVector::ZAxisVector, FMath::DegreesToRadians(120.0)),
		FVector(300.0, 300.0, 40.0));

	const FRobotDlsIKResult Result = SolveFromZero(Model, Target);

	TestTrue(TEXT("bClampJointLimits 기본값이면 결과가 가동 범위 안에 있어야 함"),
		Model.IsWithinLimits(Result.Solution));

	return true;
}

//~============================================================================
// e. NoNaNOnDifficultTarget — 어려운 목표에서도 NaN/Inf가 없어야 함
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotIKNoNaNOnDifficultTargetTest,
	"RobotSim.IK.NoNaNOnDifficultTarget",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotIKNoNaNOnDifficultTargetTest::RunTest(const FString& Parameters)
{
	using namespace RobotIKTestUtils;

	const FSerial6DoFModel Model = MakeModel();

	// 명백히 도달 불가한 아주 먼 목표 + 임의 회전.
	FTransform Target(FQuat(FVector(1.0, 1.0, 1.0).GetSafeNormal(), FMath::DegreesToRadians(150.0)),
		FVector(100000.0, -50000.0, 75000.0));

	const FRobotDlsIKResult Result = SolveFromZero(Model, Target);

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		TestTrue(
			FString::Printf(TEXT("Solution.Q[%d]가 finite여야 함, 값 %f"), i, Result.Solution.Q[i]),
			FMath::IsFinite(Result.Solution.Q[i]));
	}

	double ErrArray[6];
	Result.FinalError.ToArray6(ErrArray);
	for (int32 i = 0; i < 6; ++i)
	{
		TestTrue(
			FString::Printf(TEXT("FinalError[%d]가 finite여야 함, 값 %f"), i, ErrArray[i]),
			FMath::IsFinite(ErrArray[i]));
	}

	TestTrue(TEXT("최종 위치오차 크기가 finite여야 함"), FMath::IsFinite(Result.FinalPositionErrorCm));
	TestTrue(TEXT("최종 회전오차 크기가 finite여야 함"), FMath::IsFinite(Result.FinalRotationErrorRad));

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
