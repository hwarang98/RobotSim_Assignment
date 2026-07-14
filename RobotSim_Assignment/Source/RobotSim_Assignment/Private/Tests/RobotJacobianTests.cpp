// Fill out your copyright notice in the Description page of Project Settings.

#include "Misc/AutomationTest.h"
#include "Robot/RobotJacobian.h"
#include "Robot/Serial6DoFModel.h"

#if WITH_DEV_AUTOMATION_TESTS

/**
 * FRobotJacobian 순수 수학 numerical Jacobian 단위테스트.
 * 액터/월드를 전혀 사용하지 않으므로 스폰 없이 모델만 스택에 생성해 검증한다.
 * 골든 값은 CreateDefault() 파라미터와 Q=0 EE 위치 (105,0,120)으로부터 손으로 계산한 값이다.
 */

namespace RobotJacobianTestUtils
{
	/** 위치 도함수 허용 오차 (cm/rad). 전진차분 절단오차를 고려해 여유를 둔다. */
	constexpr double PosDerivToleranceCmPerRad = 1e-2;

	/** 회전 도함수 허용 오차 (rad/rad). */
	constexpr double RotDerivTolerance = 1e-3;

	/** Q=0 기준 모델과 관절 자세를 만들어 Jacobian을 계산한다. */
	static FRobotJacobian6x6 ComputeZeroPoseJacobian()
	{
		const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();
		const FRobot6DJointState ZeroState;
		return FRobotJacobian::ComputeNumericalJacobian(Model, ZeroState);
	}

	/** Jacobian의 특정 열(관절)에서 지정 행 구간을 벡터로 뽑는다. */
	static FVector ColumnBlock(const FRobotJacobian6x6& J, int32 Col, int32 RowStart)
	{
		return FVector(J.At(RowStart, Col), J.At(RowStart + 1, Col), J.At(RowStart + 2, Col));
	}
}

//~============================================================================
// a. ShapeAndFinite — 6×6 모든 원소가 finite인지 확인
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotJacobianShapeAndFiniteTest,
	"RobotSim.Jacobian.ShapeAndFinite",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotJacobianShapeAndFiniteTest::RunTest(const FString& Parameters)
{
	using namespace RobotJacobianTestUtils;

	const FRobotJacobian6x6 J = ComputeZeroPoseJacobian();

	for (int32 Row = 0; Row < 6; ++Row)
	{
		for (int32 Col = 0; Col < 6; ++Col)
		{
			const double Value = J.At(Row, Col);
			TestTrue(
				FString::Printf(TEXT("J[%d][%d]가 finite여야 함, 값 %f"), Row, Col, Value),
				FMath::IsFinite(Value));
		}
	}

	return true;
}

//~============================================================================
// b. J0YawPositionDerivative — Q=0에서 J0 yaw의 position derivative ≈ (0, 105, 0)
//    이유: EE 위치 (105,0,120)을 Z축 회전 → 순간 속도 ẑ × r = (0, 105, 0)
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotJacobianJ0YawPositionDerivativeTest,
	"RobotSim.Jacobian.J0YawPositionDerivative",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotJacobianJ0YawPositionDerivativeTest::RunTest(const FString& Parameters)
{
	using namespace RobotJacobianTestUtils;

	const FRobotJacobian6x6 J = ComputeZeroPoseJacobian();

	const FVector Actual = ColumnBlock(J, /*Col=J0*/ 0, /*RowStart=position*/ 0);
	const FVector Expected(0.0, 105.0, 0.0);
	const double Error = FVector::Distance(Actual, Expected);

	TestTrue(
		FString::Printf(TEXT("J0 position derivative: 기대 (%.2f, %.2f, %.2f), 실제 (%.4f, %.4f, %.4f), 오차 %.6f cm/rad"),
			Expected.X, Expected.Y, Expected.Z, Actual.X, Actual.Y, Actual.Z, Error),
		Error <= PosDerivToleranceCmPerRad);

	return true;
}

//~============================================================================
// c. J5RollPositionInvariant — Q=0에서 J5 roll은 ToolOffset과 같은 X축이므로
//    position derivative가 거의 0이어야 함
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotJacobianJ5RollPositionInvariantTest,
	"RobotSim.Jacobian.J5RollPositionInvariant",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotJacobianJ5RollPositionInvariantTest::RunTest(const FString& Parameters)
{
	using namespace RobotJacobianTestUtils;

	const FRobotJacobian6x6 J = ComputeZeroPoseJacobian();

	const FVector Actual = ColumnBlock(J, /*Col=J5*/ 5, /*RowStart=position*/ 0);
	const double Magnitude = Actual.Size();

	TestTrue(
		FString::Printf(TEXT("J5 position derivative 크기 ≈ 0: 실제 (%.6f, %.6f, %.6f), 크기 %.6f cm/rad"),
			Actual.X, Actual.Y, Actual.Z, Magnitude),
		Magnitude <= PosDerivToleranceCmPerRad);

	return true;
}

//~============================================================================
// d. J5RollRotationDerivative — Q=0에서 J5 roll의 rotation derivative는 X축 방향
//    즉 row 3 ≈ 1, row 4/5 ≈ 0
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotJacobianJ5RollRotationDerivativeTest,
	"RobotSim.Jacobian.J5RollRotationDerivative",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotJacobianJ5RollRotationDerivativeTest::RunTest(const FString& Parameters)
{
	using namespace RobotJacobianTestUtils;

	const FRobotJacobian6x6 J = ComputeZeroPoseJacobian();

	const FVector Actual = ColumnBlock(J, /*Col=J5*/ 5, /*RowStart=rotation*/ 3);
	const FVector Expected(1.0, 0.0, 0.0);
	const double Error = FVector::Distance(Actual, Expected);

	TestTrue(
		FString::Printf(TEXT("J5 rotation derivative: 기대 (1, 0, 0), 실제 (%.6f, %.6f, %.6f), 오차 %.6f rad/rad"),
			Actual.X, Actual.Y, Actual.Z, Error),
		Error <= RotDerivTolerance);

	return true;
}

//~============================================================================
// e. NoMutation — ComputeNumericalJacobian 호출 후 입력 State가 변하지 않아야 함
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotJacobianNoMutationTest,
	"RobotSim.Jacobian.NoMutation",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotJacobianNoMutationTest::RunTest(const FString& Parameters)
{
	const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();

	// 0이 아닌 임의 자세로도 불변성을 확인한다.
	FRobot6DJointState State;
	const double Original[6] = { 0.1, -0.2, 0.3, -0.4, 0.5, -0.6 };
	for (int32 i = 0; i < 6; ++i)
	{
		State.Q[i] = Original[i];
	}

	FRobotJacobian::ComputeNumericalJacobian(Model, State);

	for (int32 i = 0; i < 6; ++i)
	{
		TestTrue(
			FString::Printf(TEXT("호출 후 State.Q[%d] 불변: 기대 %.6f, 실제 %.6f"), i, Original[i], State.Q[i]),
			State.Q[i] == Original[i]);
	}

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
