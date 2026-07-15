// Fill out your copyright notice in the Description page of Project Settings.

#include "Misc/AutomationTest.h"
#include "Robot/RobotDynamicsRNEA.h"
#include "Robot/RobotSimLog.h"
#include "Robot/Serial6DoFModel.h"

#if WITH_DEV_AUTOMATION_TESTS

/**
 * Step B-02 RNEA 역동역학 단위테스트.
 *
 * B-01이 데이터 불변성만 검증했다면, 여기서는 처음으로 동역학 **계산**을 검증한다.
 * 액터/월드를 쓰지 않는 순수 수학 테스트다 (기존 RobotKinematicsTests 패턴).
 *
 * 이 파일의 핵심은 GravityTorqueMatchesEnergyGradient다. 나머지 테스트는 sanity 수준이라
 * 부호나 링크 인덱스가 한 칸 밀려도 통과할 수 있지만, 위치에너지 기울기 대조는
 * **RNEA와 완전히 독립된 경로**(신뢰된 FK만 사용)로 같은 값을 구하므로 그런 실수를 잡아낸다.
 */

namespace RobotRNEATestUtils
{
	/** Step A FK 골든값 허용 오차 (cm). 기존 테스트와 동일. */
	constexpr double PosToleranceCm = 1e-3;

	/** "정확히 0이어야 하는" 토크의 허용 오차 (N·m). 부동소수 누적오차만 허용한다. */
	constexpr double ZeroTorqueToleranceNm = 1e-12;

	/** 옵션 항(로터 관성/마찰)의 분리 검증 허용 오차 (N·m). 같은 수식을 두 번 계산하는 셈이라 타이트하다. */
	constexpr double TermToleranceNm = 1e-9;

	/**
	 * 위치에너지 수치미분 대조 허용 오차.
	 *
	 * 중심차분의 절단오차 + 반올림오차가 섞이므로 타이트하게 잡으면 brittle해진다.
	 * 상대오차 1e-3에 절대 하한 1e-2 N·m을 둔다 — 기본 모델의 중력 토크는 O(100) N·m 규모라
	 * 이 정도로도 부호 오류·링크 인덱스 밀림·cm/m 혼동(100배 차이)은 전부 걸린다.
	 */
	constexpr double EnergyGradientRelTolerance = 1e-3;
	constexpr double EnergyGradientAbsToleranceNm = 1e-2;

	/** 중심차분 스텝 (radian). 절단오차와 반올림오차의 균형점. */
	constexpr double FiniteDiffStepRad = 1e-5;

	/** 테스트용 임의 자세 (기존 B-01 회귀 테스트와 같은 값). */
	static FRobot6DJointState MakeArbitraryState()
	{
		FRobot6DJointState State;
		State.Q[0] = 0.3; State.Q[1] = -0.4; State.Q[2] = 0.5;
		State.Q[3] = 0.6; State.Q[4] = -0.2; State.Q[5] = 0.1;
		return State;
	}

	/**
	 * 링크 i의 COM 월드 위치 (m). **RNEA를 전혀 쓰지 않고** 기존 FK만으로 구한다.
	 *
	 * CenterOfMassLocalM은 m이고 FK는 cm 세계이므로, cm로 내려서 변환한 뒤 결과를 다시 m로 올린다.
	 * (이 왕복이 RNEA의 cm↔m 경계와 독립적이라는 점이 교차검증의 핵심이다.)
	 */
	static FVector ComputeComWorldM(const FSerial6DoFModel& Model, int32 LinkIndex, const FRobot6DJointState& State)
	{
		const FVector ComLocalCm = Model.LinkDynamics[LinkIndex].CenterOfMassLocalM / RobotCmToM;
		const FTransform FrameWorld = Model.ComputeJointWorldTransform(LinkIndex, State);
		return FrameWorld.TransformPosition(ComLocalCm) * RobotCmToM;
	}

	/**
	 * 로봇 전체의 중력 위치에너지 U(q) = −Σ mᵢ · g · p_comᵢ (Joule).
	 *
	 * 정의상 τ_g = ∂U/∂q이므로, 이 함수를 수치미분하면 RNEA의 중력 토크와 같아야 한다.
	 */
	static double ComputePotentialEnergyJ(const FSerial6DoFModel& Model, const FRobot6DJointState& State)
	{
		double EnergyJ = 0.0;
		for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
		{
			const FVector ComWorldM = ComputeComWorldM(Model, i, State);
			EnergyJ -= Model.LinkDynamics[i].MassKg * FVector::DotProduct(Model.GravityMPerSec2, ComWorldM);
		}
		return EnergyJ;
	}

	/** Q=0에서 EE가 Step A 골든 포즈 (105, 0, 120)cm에 그대로 있는지 검증한다. */
	static void TestZeroPoseGolden(FAutomationTestBase& Test, const FString& What, const FSerial6DoFModel& Model)
	{
		const FRobot6DJointState ZeroState;
		const FRobot6DPose Pose = Model.ComputeEndEffectorPose(ZeroState);
		const double Error = FVector::Distance(Pose.PositionCm, FVector(105.0, 0.0, 120.0));

		Test.TestTrue(
			FString::Printf(TEXT("%s: Q=0 EE는 (105, 0, 120)이어야 한다. 실제 (%.4f, %.4f, %.4f), 오차 %.6fcm"),
				*What, Pose.PositionCm.X, Pose.PositionCm.Y, Pose.PositionCm.Z, Error),
			Error <= PosToleranceCm);
	}
}

//~============================================================================
// 1. 중력을 끄고 정지 상태면 토크가 0이어야 한다. 질량이 0일 때도 마찬가지다.
//    (강체 항만 있으므로 근사가 아니라 "정확히" 0이어야 한다)
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotRNEAZeroMassOrGravityOffTest,
	"RobotSim.Dynamics.RNEA.ZeroMassOrGravityOffProducesZero",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotRNEAZeroMassOrGravityOffTest::RunTest(const FString& Parameters)
{
	using namespace RobotRNEATestUtils;

	const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();
	const FRobot6DJointState ZeroState;
	const FRobot6DJointVelocity ZeroVelocity;
	const FRobot6DJointAcceleration ZeroAcceleration;

	// (a) 중력 off + 정지: 관성력도 중력도 없다.
	FRobotRNEAOptions GravityOff;
	GravityOff.bEnableGravity = false;

	const FRobot6DJointTorque TorqueNoGravity =
		SolveInverseDynamicsRNEA(Model, ZeroState, ZeroVelocity, ZeroAcceleration, GravityOff);

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		TestTrue(FString::Printf(TEXT("중력 off + 정지 상태에서 관절 %d 토크는 0이어야 한다 (실제 %g N·m)"),
				i, TorqueNoGravity.TauNm[i]),
			FMath::Abs(TorqueNoGravity.TauNm[i]) <= ZeroTorqueToleranceNm);
	}

	// (b) 질량 0 + 중력 on + 정지: 중력이 걸릴 질량이 없으므로 역시 0이다.
	//     (관성 텐서는 그대로 두지만 qd=qdd=0이라 모멘트도 0이 된다)
	FSerial6DoFModel MasslessModel = Model;
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		MasslessModel.LinkDynamics[i].MassKg = 0.0;
	}

	const FRobot6DJointTorque TorqueMassless = ComputeGravityTorque(MasslessModel, ZeroState);

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		TestTrue(FString::Printf(TEXT("질량 0 + 정지 상태에서 관절 %d 토크는 0이어야 한다 (실제 %g N·m)"),
				i, TorqueMassless.TauNm[i]),
			FMath::Abs(TorqueMassless.TauNm[i]) <= ZeroTorqueToleranceNm);
	}

	return true;
}

//~============================================================================
// 2. 기본 모델에서 토크가 유한한가 (NaN/Inf가 새어나오지 않는가).
//    B-01의 방어 처리 덕에 데이터는 이미 유한하므로, 여기서 NaN이 나오면 원인은 수식이다.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotRNEAFiniteTorquesTest,
	"RobotSim.Dynamics.RNEA.FiniteTorques",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotRNEAFiniteTorquesTest::RunTest(const FString& Parameters)
{
	using namespace RobotRNEATestUtils;

	const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();

	// (a) 기본 요구사항: q=qd=qdd=0, 중력 on.
	const FRobot6DJointState ZeroState;
	const FRobot6DJointTorque TorqueZeroPose = ComputeGravityTorque(Model, ZeroState);

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		TestTrue(FString::Printf(TEXT("Q=0 중력 토크는 유한해야 한다 (관절 %d, 실제 %g N·m)"), i, TorqueZeroPose.TauNm[i]),
			FMath::IsFinite(TorqueZeroPose.TauNm[i]));
	}

	// (b) 모든 항이 살아있는 조건에서도 유한해야 한다 (임의 자세 + 속도 + 가속도 + 옵션 전부 on).
	FRobot6DJointVelocity Velocity;
	FRobot6DJointAcceleration Acceleration;
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		Velocity.Qd[i] = 0.5 - 0.1 * i;
		Acceleration.Qdd[i] = 1.0 - 0.2 * i;
	}

	FRobotRNEAOptions AllTermsOn;
	AllTermsOn.bIncludeRotorInertia = true;
	AllTermsOn.bIncludeFriction = true;

	const FRobot6DJointTorque TorqueFullDynamics =
		SolveInverseDynamicsRNEA(Model, MakeArbitraryState(), Velocity, Acceleration, AllTermsOn);

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		TestTrue(FString::Printf(TEXT("전체 동역학 토크는 유한해야 한다 (관절 %d, 실제 %g N·m)"), i, TorqueFullDynamics.TauNm[i]),
			FMath::IsFinite(TorqueFullDynamics.TauNm[i]));
	}

	return true;
}

//~============================================================================
// 3. 자세가 바뀌면 중력 토크도 바뀌어야 한다.
//    (모델이 자세를 아예 안 읽는 치명적 실수를 잡는 최소 검증. 정량 검증은 테스트 6번.)
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotRNEAGravityTorqueChangesWithPoseTest,
	"RobotSim.Dynamics.RNEA.GravityTorqueChangesWithPose",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotRNEAGravityTorqueChangesWithPoseTest::RunTest(const FString& Parameters)
{
	const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();

	const FRobot6DJointState ZeroState;

	// J1은 pitch(+Y) 관절이라 30° 기울이면 팔 전체의 중력 모멘트 팔이 확 바뀐다.
	FRobot6DJointState TiltedState;
	TiltedState.Q[1] = FMath::DegreesToRadians(30.0);

	const FRobot6DJointTorque TorqueZero = ComputeGravityTorque(Model, ZeroState);
	const FRobot6DJointTorque TorqueTilted = ComputeGravityTorque(Model, TiltedState);

	const double DeltaJ1Nm = FMath::Abs(TorqueTilted.TauNm[1] - TorqueZero.TauNm[1]);

	TestTrue(
		FString::Printf(TEXT("J1을 30° 기울이면 J1 중력 토크가 달라져야 한다 (Q=0: %g, 30°: %g, 차이 %g N·m)"),
			TorqueZero.TauNm[1], TorqueTilted.TauNm[1], DeltaJ1Nm),
		DeltaJ1Nm > 1.0);

	return true;
}

//~============================================================================
// 4. 관절 j만 +가속시키면 그 관절 토크도 +여야 한다.
//    질량행렬의 대각 성분 M_jj(q)는 항상 양수이므로(양정부호) τ_j = M_jj·qdd_j > 0이 보장된다.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotRNEAAccelerationTorqueSignTest,
	"RobotSim.Dynamics.RNEA.AccelerationTorqueSign",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotRNEAAccelerationTorqueSignTest::RunTest(const FString& Parameters)
{
	using namespace RobotRNEATestUtils;

	const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();
	const FRobot6DJointVelocity ZeroVelocity;

	// 중력을 끄면 남는 건 순수 관성 항뿐이라 부호 판정이 명확해진다.
	FRobotRNEAOptions GravityOff;
	GravityOff.bEnableGravity = false;

	// Q=0과 임의 자세 양쪽에서 확인한다 (M(q)의 양정부호성은 자세와 무관하다).
	const FRobot6DJointState States[2] = { FRobot6DJointState(), MakeArbitraryState() };
	const TCHAR* StateNames[2] = { TEXT("Q=0"), TEXT("임의 자세") };

	for (int32 StateIndex = 0; StateIndex < 2; ++StateIndex)
	{
		for (int32 j = 0; j < FSerial6DoFModel::NumJoints; ++j)
		{
			FRobot6DJointAcceleration Acceleration;
			Acceleration.Qdd[j] = 1.0;

			const FRobot6DJointTorque Torque =
				SolveInverseDynamicsRNEA(Model, States[StateIndex], ZeroVelocity, Acceleration, GravityOff);

			TestTrue(
				FString::Printf(TEXT("%s에서 관절 %d만 +1 rad/s²로 가속하면 관절 %d 토크는 양수여야 한다 (실제 %g N·m)"),
					StateNames[StateIndex], j, j, Torque.TauNm[j]),
				Torque.TauNm[j] > 0.0);
		}
	}

	return true;
}

//~============================================================================
// 5. 회귀 가드: RNEA를 추가/호출해도 Step A의 FK 골든 포즈는 그대로여야 한다.
//    (RNEA가 모델을 const로만 읽는다는 것의 실증)
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotRNEAFKRegressionTest,
	"RobotSim.Dynamics.RNEA.FKRegressionUnaffected",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotRNEAFKRegressionTest::RunTest(const FString& Parameters)
{
	using namespace RobotRNEATestUtils;

	const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();

	TestZeroPoseGolden(*this, TEXT("RNEA 호출 전"), Model);

	// 온갖 조건으로 RNEA를 돌린다.
	const FRobot6DJointState ArbitraryState = MakeArbitraryState();
	FRobot6DJointVelocity Velocity;
	FRobot6DJointAcceleration Acceleration;
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		Velocity.Qd[i] = 1.0;
		Acceleration.Qdd[i] = 2.0;
	}

	FRobotRNEAOptions AllTermsOn;
	AllTermsOn.bIncludeRotorInertia = true;
	AllTermsOn.bIncludeFriction = true;

	SolveInverseDynamicsRNEA(Model, ArbitraryState, Velocity, Acceleration, AllTermsOn);
	ComputeGravityTorque(Model, ArbitraryState);

	TestZeroPoseGolden(*this, TEXT("RNEA 호출 후"), Model);

	// 임의 자세의 EE도 Step A 그대로여야 한다.
	const FSerial6DoFModel Untouched = FSerial6DoFModel::CreateDefault();
	const FRobot6DPose PoseAfterRNEA = Model.ComputeEndEffectorPose(ArbitraryState);
	const FRobot6DPose PoseReference = Untouched.ComputeEndEffectorPose(ArbitraryState);

	const double PositionDelta = FVector::Distance(PoseAfterRNEA.PositionCm, PoseReference.PositionCm);
	const double AngularDelta = PoseAfterRNEA.Orientation.AngularDistance(PoseReference.Orientation);

	TestTrue(FString::Printf(TEXT("임의 자세 EE 위치가 RNEA 호출과 무관해야 한다 (차이 %g cm)"), PositionDelta),
		PositionDelta <= PosToleranceCm);
	TestTrue(FString::Printf(TEXT("임의 자세 EE 자세가 RNEA 호출과 무관해야 한다 (차이 %g rad)"), AngularDelta),
		AngularDelta <= 1e-9);

	return true;
}

//~============================================================================
// 6. **핵심 교차검증**: 중력 토크가 위치에너지의 기울기와 일치하는가.
//
//    τ_g,j = ∂U/∂q_j,  U(q) = −Σ mᵢ · g · p_comᵢ(q)
//
//    우변은 기존 ComputeJointWorldTransform(신뢰된 Step A FK)만으로 계산하므로 RNEA의
//    forward/backward pass와 완전히 독립적이다. 프레임 규약(R이 i→i−1 사상인가),
//    링크 인덱스(span이 LinkOffsets[i+1]인가), cm→m 경계, 중력 부호가 전부 맞아야만 통과한다.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotRNEAGravityMatchesEnergyGradientTest,
	"RobotSim.Dynamics.RNEA.GravityTorqueMatchesEnergyGradient",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotRNEAGravityMatchesEnergyGradientTest::RunTest(const FString& Parameters)
{
	using namespace RobotRNEATestUtils;

	const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();

	// 여러 자세에서 확인한다. Q=0은 특이하게 대칭이라(팔이 XZ 평면에 눕는다) 통과하기 쉬우므로
	// 모든 관절이 비대칭으로 꺾인 자세를 반드시 포함한다.
	FRobot6DJointState TiltedState;
	TiltedState.Q[1] = FMath::DegreesToRadians(30.0);

	const FRobot6DJointState States[3] = { FRobot6DJointState(), TiltedState, MakeArbitraryState() };
	const TCHAR* StateNames[3] = { TEXT("Q=0"), TEXT("J1=30°"), TEXT("임의 자세") };

	for (int32 StateIndex = 0; StateIndex < 3; ++StateIndex)
	{
		const FRobot6DJointState& State = States[StateIndex];
		const FRobot6DJointTorque Torque = ComputeGravityTorque(Model, State);

		for (int32 j = 0; j < FSerial6DoFModel::NumJoints; ++j)
		{
			// 중심차분으로 ∂U/∂q_j를 구한다.
			FRobot6DJointState Forward = State;
			FRobot6DJointState Backward = State;
			Forward.Q[j] += FiniteDiffStepRad;
			Backward.Q[j] -= FiniteDiffStepRad;

			const double ExpectedNm =
				(ComputePotentialEnergyJ(Model, Forward) - ComputePotentialEnergyJ(Model, Backward))
				/ (2.0 * FiniteDiffStepRad);

			const double ErrorNm = FMath::Abs(Torque.TauNm[j] - ExpectedNm);
			const double ToleranceNm = FMath::Max(
				EnergyGradientAbsToleranceNm,
				EnergyGradientRelTolerance * FMath::Abs(ExpectedNm));

			TestTrue(
				FString::Printf(
					TEXT("%s: 관절 %d 중력 토크는 위치에너지 기울기와 같아야 한다. RNEA %g, ∂U/∂q %g, 오차 %g N·m (허용 %g)"),
					StateNames[StateIndex], j, Torque.TauNm[j], ExpectedNm, ErrorNm, ToleranceNm),
				ErrorNm <= ToleranceNm);
		}
	}

	return true;
}

//~============================================================================
// 7. 로터 관성/마찰이 강체 항과 분리된 별도 항인가.
//    옵션을 켠 결과에서 끈 결과를 빼면 정확히 해당 항의 수식이 나와야 한다.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotRNEASeparableTermsTest,
	"RobotSim.Dynamics.RNEA.FrictionAndRotorInertiaAreSeparableTerms",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotRNEASeparableTermsTest::RunTest(const FString& Parameters)
{
	using namespace RobotRNEATestUtils;

	const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();
	const FRobot6DJointState State = MakeArbitraryState();

	// 부호가 섞이도록 속도에 음수를 포함시킨다 (Coulomb 항의 sign(qd) 검증).
	FRobot6DJointVelocity Velocity;
	FRobot6DJointAcceleration Acceleration;
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		Velocity.Qd[i] = (i % 2 == 0) ? 0.7 : -0.7;
		Acceleration.Qdd[i] = 1.5;
	}

	const FRobotRNEAOptions RigidOnly;
	const FRobot6DJointTorque TorqueRigid =
		SolveInverseDynamicsRNEA(Model, State, Velocity, Acceleration, RigidOnly);

	// (a) 로터 관성 항 = Ir · qdd
	FRobotRNEAOptions WithRotor;
	WithRotor.bIncludeRotorInertia = true;
	const FRobot6DJointTorque TorqueWithRotor =
		SolveInverseDynamicsRNEA(Model, State, Velocity, Acceleration, WithRotor);

	// (b) 마찰 항 = b · qd + c · sign(qd)
	FRobotRNEAOptions WithFriction;
	WithFriction.bIncludeFriction = true;
	const FRobot6DJointTorque TorqueWithFriction =
		SolveInverseDynamicsRNEA(Model, State, Velocity, Acceleration, WithFriction);

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		const FRobotLinkDynamics& Dynamics = Model.LinkDynamics[i];

		const double ExpectedRotorNm = Dynamics.RotorInertiaKgM2 * Acceleration.Qdd[i];
		const double ActualRotorNm = TorqueWithRotor.TauNm[i] - TorqueRigid.TauNm[i];
		TestTrue(
			FString::Printf(TEXT("관절 %d 로터 관성 항은 Ir·qdd여야 한다 (기대 %g, 실제 %g N·m)"),
				i, ExpectedRotorNm, ActualRotorNm),
			FMath::Abs(ActualRotorNm - ExpectedRotorNm) <= TermToleranceNm);

		// 마찰은 운동을 방해하므로 모터가 더 내야 한다 → qd와 같은 부호로 가산된다.
		const double ExpectedFrictionNm =
			Dynamics.ViscousFrictionNmsPerRad * Velocity.Qd[i]
			+ Dynamics.CoulombFrictionNm * FMath::Sign(Velocity.Qd[i]);
		const double ActualFrictionNm = TorqueWithFriction.TauNm[i] - TorqueRigid.TauNm[i];
		TestTrue(
			FString::Printf(TEXT("관절 %d 마찰 항은 b·qd + c·sign(qd)여야 한다 (기대 %g, 실제 %g N·m)"),
				i, ExpectedFrictionNm, ActualFrictionNm),
			FMath::Abs(ActualFrictionNm - ExpectedFrictionNm) <= TermToleranceNm);

		TestTrue(
			FString::Printf(TEXT("관절 %d 마찰 항은 qd와 같은 부호여야 한다 (qd %g, 마찰 %g N·m)"),
				i, Velocity.Qd[i], ActualFrictionNm),
			ActualFrictionNm * Velocity.Qd[i] > 0.0);
	}

	// 마찰은 정지 시 0이어야 한다 (정지 마찰은 모델링하지 않는다).
	const FRobot6DJointVelocity ZeroVelocity;
	const FRobot6DJointTorque TorqueStaticRigid =
		SolveInverseDynamicsRNEA(Model, State, ZeroVelocity, Acceleration, RigidOnly);
	const FRobot6DJointTorque TorqueStaticFriction =
		SolveInverseDynamicsRNEA(Model, State, ZeroVelocity, Acceleration, WithFriction);

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		const double DeltaNm = TorqueStaticFriction.TauNm[i] - TorqueStaticRigid.TauNm[i];
		TestTrue(FString::Printf(TEXT("관절 %d: qd=0이면 마찰 항이 0이어야 한다 (실제 %g N·m)"), i, DeltaNm),
			FMath::Abs(DeltaNm) <= TermToleranceNm);
	}

	return true;
}

//~============================================================================
// 8. MaxTorqueNm 산정용 진단 로그 (비단정).
//
//    각 자세의 중력 토크를 LogRobotSim에 출력한다. **MaxTorqueNm과 비교 단정은 하지 않는다** —
//    B-01의 값(400/600/350/120/80/40)은 잠정값이고, RNEA 검증 전에 토크 한계를 자동 갱신하면
//    회귀 원인이 수식인지 한계값인지 분리할 수 없기 때문이다(STEP_B-02.md 참조).
//    값 확정은 이 로그를 사람이 검토한 뒤 별도 판단으로 남긴다.
//
//    **최악 자세는 관절마다 다르다.** 팔을 수평 전개한 자세는 J1/J2/J4의 모멘트 팔을 최대화하지만,
//    그 자세에서 J3(팔 축 방향 roll)는 하류 COM이 전부 자기 회전축 위에 놓여 토크가 0이 된다.
//    J3를 부하시키려면 손목을 꺾어 COM을 축에서 떼어내야 하므로 자세를 하나 더 둔다.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotRNEAGravityTorqueReportTest,
	"RobotSim.Dynamics.RNEA.GravityTorqueReportForMaxTorqueSizing",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotRNEAGravityTorqueReportTest::RunTest(const FString& Parameters)
{
	const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();

	// 자세 A: 팔을 +X로 완전히 수평 전개해 중력 모멘트 팔을 최대화한다. J1/J2/J4의 최악 자세다.
	// J1=+90°가 상완을 +Z에서 +X로 눕히고(+Z→+X), J2=−90°가 그 뒤를 다시 +X로 편다.
	FRobot6DJointState ExtendedState;
	ExtendedState.Q[1] = FMath::DegreesToRadians(90.0);
	ExtendedState.Q[2] = FMath::DegreesToRadians(-90.0);

	// 자세 B: 수평 전개에 손목 roll(J3) + pitch(J4)를 더한다. J3 부하용이다.
	// J3=90°가 프레임 3의 y축을 수직으로 세워 중력이 y성분을 갖게 하고,
	// J4=90°가 하류 COM을 J3 축에서 떼어내 모멘트 팔을 만든다. 둘 다 있어야 J3에 토크가 걸린다.
	FRobot6DJointState WristBentState = ExtendedState;
	WristBentState.Q[3] = FMath::DegreesToRadians(90.0);
	WristBentState.Q[4] = FMath::DegreesToRadians(90.0);

	const FRobot6DJointState States[2] = { ExtendedState, WristBentState };
	const TCHAR* StateNames[2] = {
		TEXT("A: 수평 전개 Q=(0, +90, -90, 0, 0, 0) — J1/J2/J4 최악"),
		TEXT("B: 수평 전개 + 손목 꺾음 Q=(0, +90, -90, +90, +90, 0) — J3 부하")
	};

	for (int32 StateIndex = 0; StateIndex < 2; ++StateIndex)
	{
		const FRobot6DJointState& State = States[StateIndex];

		TestTrue(FString::Printf(TEXT("자세 %d는 관절 한계 안이어야 한다"), StateIndex),
			Model.IsWithinLimits(State));

		const FRobot6DPose Pose = Model.ComputeEndEffectorPose(State);
		const FRobot6DJointTorque GravityTorque = ComputeGravityTorque(Model, State);

		UE_LOG(LogRobotSim, Log, TEXT("[RNEA] MaxTorqueNm 산정 리포트 — 자세 %s"), StateNames[StateIndex]);
		UE_LOG(LogRobotSim, Log, TEXT("[RNEA]   EE = (%.1f, %.1f, %.1f)cm"),
			Pose.PositionCm.X, Pose.PositionCm.Y, Pose.PositionCm.Z);
		UE_LOG(LogRobotSim, Log,
			TEXT("[RNEA]   관절 | 중력토크(N·m) | 현재 MaxTorqueNm(잠정) | 여유배수 | 권장(|τ|×2.5)"));

		for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
		{
			const double GravityTorqueNm = GravityTorque.TauNm[i];
			const double AbsGravityTorqueNm = FMath::Abs(GravityTorqueNm);
			const double CurrentMaxTorqueNm = Model.JointLimits[i].MaxTorqueNm;

			// 중력 토크가 0인 관절은 여유배수가 무한대라 숫자로 찍으면 의미가 없다.
			// 이 관절들의 한계는 중력이 아니라 순수 가속 요구(B-03의 M(q) 대각)에서 산정해야 한다.
			const bool bGravityLoaded = AbsGravityTorqueNm > KINDA_SMALL_NUMBER;
			const FString MarginText = bGravityLoaded
				? FString::Printf(TEXT("%8.2f"), CurrentMaxTorqueNm / AbsGravityTorqueNm)
				: FString(TEXT("       -"));
			const FString SuggestionText = bGravityLoaded
				? FString::Printf(TEXT("%10.1f"), AbsGravityTorqueNm * 2.5)
				: FString(TEXT("M(q)에서 산정"));

			UE_LOG(LogRobotSim, Log,
				TEXT("[RNEA]     J%d  |  %10.3f  |  %10.1f  |  %s  |  %s"),
				i, GravityTorqueNm, CurrentMaxTorqueNm, *MarginText, *SuggestionText);

			// 단정은 유한성까지만. MaxTorqueNm과의 비교는 의도적으로 하지 않는다.
			TestTrue(FString::Printf(TEXT("자세 %d 관절 %d 중력 토크는 유한해야 한다 (실제 %g N·m)"),
					StateIndex, i, GravityTorqueNm),
				FMath::IsFinite(GravityTorqueNm));
		}

		// J0(yaw)의 중력 토크는 **모든 자세에서** 구조적으로 0이다: 축이 +Z(수직)이고 중력도 수직이라
		// r × F의 Z 성분 = r_x·F_y − r_y·F_x = 0이 된다 (수직력은 수직축 둘레 모멘트를 만들지 못한다).
		TestTrue(
			FString::Printf(TEXT("자세 %d: J0(yaw)는 중력과 축이 평행하므로 중력 토크가 0이어야 한다 (실제 %g N·m)"),
				StateIndex, GravityTorque.TauNm[0]),
			FMath::Abs(GravityTorque.TauNm[0]) <= 1e-9);

		// J5(툴 roll)도 마찬가지로 구조적 0이다: ToolOffset(10,0,0)이 J5 축(+X)과 동일선상이라
		// 툴 COM이 항상 회전축 위에 놓인다 (CreateDefault가 의도한 불변량).
		TestTrue(
			FString::Printf(TEXT("자세 %d: J5는 툴 COM이 회전축 위에 있으므로 중력 토크가 0이어야 한다 (실제 %g N·m)"),
				StateIndex, GravityTorque.TauNm[5]),
			FMath::Abs(GravityTorque.TauNm[5]) <= 1e-9);
	}

	// J3는 구조적 0이 아니다 — 자세 A에서 0인 것은 팔이 y=0 평면에 눕는 그 자세의 성질일 뿐이다.
	// 자세 B에서는 실제로 토크가 걸려야 하며, 이게 0이면 최악 자세 선정이 틀린 것이다.
	const FRobot6DJointTorque TorqueWristBent = ComputeGravityTorque(Model, WristBentState);
	TestTrue(
		FString::Printf(TEXT("손목을 꺾으면 J3에 중력 토크가 걸려야 한다 (실제 %g N·m)"), TorqueWristBent.TauNm[3]),
		FMath::Abs(TorqueWristBent.TauNm[3]) > 1.0);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
