// Fill out your copyright notice in the Description page of Project Settings.

#include "Misc/AutomationTest.h"
#include "Robot/RobotConfig.h"
#include "Robot/Serial6DoFModel.h"
#include "UObject/StrongObjectPtr.h"

#include <limits>

#if WITH_DEV_AUTOMATION_TESTS

/**
 * Step B-01 동역학 파라미터 단위테스트.
 *
 * B-01은 데이터만 추가하는 단계이므로 여기서도 **데이터 불변성**만 검증한다:
 * 기본값이 물리적으로 타당한가, 그리고 동역학 값이 Step A의 FK 결과를 건드리지 않는가.
 * RNEA/질량행렬/토크 제어는 아직 없으므로 동역학 "계산"을 검증하는 테스트는 없다 (B-02부터).
 *
 * 액터/월드를 쓰지 않는 기존 RobotKinematicsTests의 패턴을 따르되, URobotConfig 왕복 검증만
 * UObject가 필요해 NewObject + TStrongObjectPtr(GC 보호)로 생성한다.
 */

namespace RobotDynamicsParamsTestUtils
{
	/** Step A FK 골든값 허용 오차 (cm). */
	constexpr double PosToleranceCm = 1e-3;

	/** 동역학 값 비교 허용 오차 (상대적으로 작은 SI 값들이라 타이트하게 둔다). */
	constexpr double DynamicsTolerance = 1e-9;

	/** 링크 i의 span 벡터(cm) = 프레임 i 원점 → 프레임 i+1 원점. i=5는 ToolOffset. */
	static FVector GetLinkSpanCm(const FSerial6DoFModel& Model, int32 LinkIndex)
	{
		return (LinkIndex + 1 < FSerial6DoFModel::NumJoints)
			? Model.LinkOffsets[LinkIndex + 1]
			: Model.ToolOffset.GetTranslation();
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
// 1. 기본값이 전부 유한하고 물리적으로 타당한 부호를 갖는가.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotDynamicsDefaultsPositiveFiniteTest,
	"RobotSim.Dynamics.Params.DefaultsArePositiveFinite",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotDynamicsDefaultsPositiveFiniteTest::RunTest(const FString& Parameters)
{
	const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		const FRobotLinkDynamics& Dynamics = Model.LinkDynamics[i];

		TestTrue(FString::Printf(TEXT("링크 %d 질량은 유한한 양수여야 한다 (실제 %g)"), i, Dynamics.MassKg),
			FMath::IsFinite(Dynamics.MassKg) && Dynamics.MassKg > 0.0);

		TestFalse(FString::Printf(TEXT("링크 %d COM에 NaN이 없어야 한다"), i),
			Dynamics.CenterOfMassLocalM.ContainsNaN());

		TestTrue(FString::Printf(TEXT("링크 %d 관성 대각은 세 성분 모두 유한한 양수여야 한다 (실제 %s)"),
				i, *Dynamics.InertiaDiagonalKgM2.ToString()),
			FMath::IsFinite(Dynamics.InertiaDiagonalKgM2.X) && Dynamics.InertiaDiagonalKgM2.X > 0.0 &&
			FMath::IsFinite(Dynamics.InertiaDiagonalKgM2.Y) && Dynamics.InertiaDiagonalKgM2.Y > 0.0 &&
			FMath::IsFinite(Dynamics.InertiaDiagonalKgM2.Z) && Dynamics.InertiaDiagonalKgM2.Z > 0.0);

		// 로터 관성/마찰은 0(미모델링)이 유효하므로 "음수가 아님"까지만 요구한다.
		TestTrue(FString::Printf(TEXT("링크 %d 로터 관성은 유한하고 0 이상이어야 한다 (실제 %g)"), i, Dynamics.RotorInertiaKgM2),
			FMath::IsFinite(Dynamics.RotorInertiaKgM2) && Dynamics.RotorInertiaKgM2 >= 0.0);
		TestTrue(FString::Printf(TEXT("링크 %d 점성 마찰은 유한하고 0 이상이어야 한다 (실제 %g)"), i, Dynamics.ViscousFrictionNmsPerRad),
			FMath::IsFinite(Dynamics.ViscousFrictionNmsPerRad) && Dynamics.ViscousFrictionNmsPerRad >= 0.0);
		TestTrue(FString::Printf(TEXT("링크 %d Coulomb 마찰은 유한하고 0 이상이어야 한다 (실제 %g)"), i, Dynamics.CoulombFrictionNm),
			FMath::IsFinite(Dynamics.CoulombFrictionNm) && Dynamics.CoulombFrictionNm >= 0.0);

		TestTrue(FString::Printf(TEXT("관절 %d 최대 토크는 유한한 양수여야 한다 (실제 %g)"), i, Model.JointLimits[i].MaxTorqueNm),
			FMath::IsFinite(Model.JointLimits[i].MaxTorqueNm) && Model.JointLimits[i].MaxTorqueNm > 0.0);
	}

	TestFalse(TEXT("중력 벡터에 NaN이 없어야 한다"), Model.GravityMPerSec2.ContainsNaN());
	TestTrue(TEXT("중력은 −Z 방향이어야 한다 (Unreal Z-up)"), Model.GravityMPerSec2.Z < 0.0);

	return true;
}

//~============================================================================
// 2. 관성 텐서가 실현 가능한가 (삼각 부등식: 어떤 두 주관성의 합도 나머지 하나 이상).
//    실제 강체라면 반드시 성립하므로, 근사 공식을 잘못 쓰면 여기서 걸린다.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotDynamicsInertiaTriangleInequalityTest,
	"RobotSim.Dynamics.Params.InertiaSatisfiesTriangleInequality",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotDynamicsInertiaTriangleInequalityTest::RunTest(const FString& Parameters)
{
	const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		const FVector I = Model.LinkDynamics[i].InertiaDiagonalKgM2;

		TestTrue(FString::Printf(TEXT("링크 %d: Ixx + Iyy >= Izz (%g + %g vs %g)"), i, I.X, I.Y, I.Z), I.X + I.Y >= I.Z);
		TestTrue(FString::Printf(TEXT("링크 %d: Iyy + Izz >= Ixx (%g + %g vs %g)"), i, I.Y, I.Z, I.X), I.Y + I.Z >= I.X);
		TestTrue(FString::Printf(TEXT("링크 %d: Izz + Ixx >= Iyy (%g + %g vs %g)"), i, I.Z, I.X, I.Y), I.Z + I.X >= I.Y);
	}

	return true;
}

//~============================================================================
// 3. COM이 링크 span의 중점인가 (균일 밀도 근사의 정의).
//    링크 i의 span을 LinkOffsets[i]로 잘못 잡는 한 칸 밀림 실수를 잡아낸다.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotDynamicsComAtLinkSpanMidpointTest,
	"RobotSim.Dynamics.Params.ComLiesAtLinkSpanMidpoint",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotDynamicsComAtLinkSpanMidpointTest::RunTest(const FString& Parameters)
{
	using namespace RobotDynamicsParamsTestUtils;

	const FSerial6DoFModel Model = FSerial6DoFModel::CreateDefault();

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		// span(cm)의 중점을 m로 환산한 값이 기대 COM이다.
		const FVector ExpectedComM = GetLinkSpanCm(Model, i) * 0.5 * 0.01;
		const FVector ActualComM = Model.LinkDynamics[i].CenterOfMassLocalM;
		const double Error = FVector::Distance(ActualComM, ExpectedComM);

		TestTrue(
			FString::Printf(TEXT("링크 %d COM은 span 중점 %s여야 한다. 실제 %s, 오차 %g m"),
				i, *ExpectedComM.ToString(), *ActualComM.ToString(), Error),
			Error <= DynamicsTolerance);
	}

	return true;
}

//~============================================================================
// 4. 회귀 가드: 동역학 값을 극단적으로 바꿔도 Step A의 FK 골든 포즈가 변하지 않아야 한다.
//    (FK 함수가 LinkDynamics/Gravity를 읽지 않는다는 것의 실증)
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotDynamicsDoNotAffectFKTest,
	"RobotSim.Dynamics.Params.ZeroPoseFKUnaffectedByDynamics",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotDynamicsDoNotAffectFKTest::RunTest(const FString& Parameters)
{
	using namespace RobotDynamicsParamsTestUtils;

	const FSerial6DoFModel Default = FSerial6DoFModel::CreateDefault();
	TestZeroPoseGolden(*this, TEXT("기본 모델"), Default);

	// 동역학 값만 말도 안 되게 바꾼 모델.
	FSerial6DoFModel Mutated = Default;
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		Mutated.LinkDynamics[i].MassKg = 12345.0;
		Mutated.LinkDynamics[i].CenterOfMassLocalM = FVector(9.0, -9.0, 9.0);
		Mutated.LinkDynamics[i].InertiaDiagonalKgM2 = FVector(500.0, 500.0, 500.0);
		Mutated.LinkDynamics[i].RotorInertiaKgM2 = 77.0;
		Mutated.LinkDynamics[i].ViscousFrictionNmsPerRad = 77.0;
		Mutated.LinkDynamics[i].CoulombFrictionNm = 77.0;
		Mutated.JointLimits[i].MaxTorqueNm = 99999.0;
	}
	Mutated.GravityMPerSec2 = FVector(100.0, 100.0, 100.0);

	TestZeroPoseGolden(*this, TEXT("동역학 값을 극단적으로 바꾼 모델"), Mutated);

	// 임의 자세에서도 두 모델의 EE가 완전히 동일해야 한다.
	FRobot6DJointState State;
	State.Q[0] = 0.3; State.Q[1] = -0.4; State.Q[2] = 0.5;
	State.Q[3] = 0.6; State.Q[4] = -0.2; State.Q[5] = 0.1;

	const FRobot6DPose DefaultPose = Default.ComputeEndEffectorPose(State);
	const FRobot6DPose MutatedPose = Mutated.ComputeEndEffectorPose(State);
	const double PositionDelta = FVector::Distance(DefaultPose.PositionCm, MutatedPose.PositionCm);
	const double AngularDelta = DefaultPose.Orientation.AngularDistance(MutatedPose.Orientation);

	TestTrue(FString::Printf(TEXT("임의 자세 EE 위치가 동역학 값과 무관해야 한다 (차이 %g cm)"), PositionDelta),
		PositionDelta <= PosToleranceCm);
	TestTrue(FString::Printf(TEXT("임의 자세 EE 자세가 동역학 값과 무관해야 한다 (차이 %g rad)"), AngularDelta),
		AngularDelta <= 1e-9);

	return true;
}

//~============================================================================
// 5. URobotConfig::ToModel()이 동역학 값을 그대로 복사하면서 FK 결과는 바꾸지 않는가.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotDynamicsConfigRoundTripTest,
	"RobotSim.Dynamics.Params.ConfigToModelCopiesDynamicsAndKeepsFK",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotDynamicsConfigRoundTripTest::RunTest(const FString& Parameters)
{
	using namespace RobotDynamicsParamsTestUtils;

	// 생성자가 CreateDefault()로 채우므로, 갓 만든 에셋의 ToModel()은 기본 모델과 같아야 한다.
	TStrongObjectPtr<URobotConfig> Config(NewObject<URobotConfig>());
	if (!TestNotNull(TEXT("URobotConfig 생성"), Config.Get()))
	{
		return false;
	}

	const FSerial6DoFModel Default = FSerial6DoFModel::CreateDefault();
	const FSerial6DoFModel Model = Config->ToModel();

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		const FRobotLinkDynamics& Expected = Default.LinkDynamics[i];
		const FRobotLinkDynamics& Actual = Model.LinkDynamics[i];

		TestTrue(FString::Printf(TEXT("링크 %d 질량 왕복 (기대 %g, 실제 %g)"), i, Expected.MassKg, Actual.MassKg),
			FMath::Abs(Expected.MassKg - Actual.MassKg) <= DynamicsTolerance);
		TestTrue(FString::Printf(TEXT("링크 %d COM 왕복 (기대 %s, 실제 %s)"), i, *Expected.CenterOfMassLocalM.ToString(), *Actual.CenterOfMassLocalM.ToString()),
			FVector::Distance(Expected.CenterOfMassLocalM, Actual.CenterOfMassLocalM) <= DynamicsTolerance);
		TestTrue(FString::Printf(TEXT("링크 %d 관성 왕복 (기대 %s, 실제 %s)"), i, *Expected.InertiaDiagonalKgM2.ToString(), *Actual.InertiaDiagonalKgM2.ToString()),
			FVector::Distance(Expected.InertiaDiagonalKgM2, Actual.InertiaDiagonalKgM2) <= DynamicsTolerance);
		TestTrue(FString::Printf(TEXT("링크 %d 로터 관성 왕복"), i),
			FMath::Abs(Expected.RotorInertiaKgM2 - Actual.RotorInertiaKgM2) <= DynamicsTolerance);
		TestTrue(FString::Printf(TEXT("링크 %d 점성 마찰 왕복"), i),
			FMath::Abs(Expected.ViscousFrictionNmsPerRad - Actual.ViscousFrictionNmsPerRad) <= DynamicsTolerance);
		TestTrue(FString::Printf(TEXT("링크 %d Coulomb 마찰 왕복"), i),
			FMath::Abs(Expected.CoulombFrictionNm - Actual.CoulombFrictionNm) <= DynamicsTolerance);
		TestTrue(FString::Printf(TEXT("관절 %d 최대 토크 왕복 (기대 %g, 실제 %g)"), i, Default.JointLimits[i].MaxTorqueNm, Model.JointLimits[i].MaxTorqueNm),
			FMath::Abs(Default.JointLimits[i].MaxTorqueNm - Model.JointLimits[i].MaxTorqueNm) <= DynamicsTolerance);
	}

	TestTrue(TEXT("중력 벡터 왕복"), FVector::Distance(Default.GravityMPerSec2, Model.GravityMPerSec2) <= DynamicsTolerance);

	// 동역학 필드가 붙은 뒤에도 기본 에셋의 FK는 Step A 골든값 그대로다.
	TestZeroPoseGolden(*this, TEXT("URobotConfig::ToModel()"), Model);

	return true;
}

//~============================================================================
// 6. 잘못된 authoring 값(NaN/Inf/0 이하)이 들어와도 모델이 유효하게 나오는가.
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobotDynamicsConfigSanitizeTest,
	"RobotSim.Dynamics.Params.ConfigToModelSanitizesInvalidValues",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobotDynamicsConfigSanitizeTest::RunTest(const FString& Parameters)
{
	using namespace RobotDynamicsParamsTestUtils;

	TStrongObjectPtr<URobotConfig> Config(NewObject<URobotConfig>());
	if (!TestNotNull(TEXT("URobotConfig 생성"), Config.Get()))
	{
		return false;
	}

	// 물리적으로 불가능하거나 수치적으로 오염된 값을 authoring 필드에 심는다.
	// (FMath::Sqrt(-1.0) 대신 quiet_NaN을 쓴다 — 컴파일러 최적화에 접히지 않게)
	const double NaN = std::numeric_limits<double>::quiet_NaN();
	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		Config->LinkDynamics[i].MassKg = (i % 2 == 0) ? 0.0 : -5.0;
		Config->LinkDynamics[i].CenterOfMassLocalM = FVector(NaN, 0.0, 0.0);
		Config->LinkDynamics[i].InertiaDiagonalKgM2 = FVector(0.0, -1.0, NaN);
		Config->LinkDynamics[i].RotorInertiaKgM2 = -1.0;
		Config->LinkDynamics[i].ViscousFrictionNmsPerRad = NaN;
		Config->LinkDynamics[i].CoulombFrictionNm = -2.0;
		Config->Joints[i].MaxTorqueNm = 0.0;
	}
	Config->GravityMPerSec2 = FVector(NaN, NaN, NaN);

	const FSerial6DoFModel Model = Config->ToModel();
	const FSerial6DoFModel Default = FSerial6DoFModel::CreateDefault();

	for (int32 i = 0; i < FSerial6DoFModel::NumJoints; ++i)
	{
		const FRobotLinkDynamics& Actual = Model.LinkDynamics[i];

		TestTrue(FString::Printf(TEXT("링크 %d: 잘못된 질량은 기본값으로 대체되어야 한다 (실제 %g)"), i, Actual.MassKg),
			FMath::IsFinite(Actual.MassKg) && Actual.MassKg > 0.0);
		TestFalse(FString::Printf(TEXT("링크 %d: NaN COM이 모델로 새어나오면 안 된다"), i),
			Actual.CenterOfMassLocalM.ContainsNaN());
		TestTrue(FString::Printf(TEXT("링크 %d: 잘못된 관성은 유한한 양수로 대체되어야 한다 (실제 %s)"), i, *Actual.InertiaDiagonalKgM2.ToString()),
			FMath::IsFinite(Actual.InertiaDiagonalKgM2.X) && Actual.InertiaDiagonalKgM2.X > 0.0 &&
			FMath::IsFinite(Actual.InertiaDiagonalKgM2.Y) && Actual.InertiaDiagonalKgM2.Y > 0.0 &&
			FMath::IsFinite(Actual.InertiaDiagonalKgM2.Z) && Actual.InertiaDiagonalKgM2.Z > 0.0);
		TestTrue(FString::Printf(TEXT("링크 %d: 음수 로터 관성은 대체되어야 한다 (실제 %g)"), i, Actual.RotorInertiaKgM2),
			FMath::IsFinite(Actual.RotorInertiaKgM2) && Actual.RotorInertiaKgM2 >= 0.0);
		TestTrue(FString::Printf(TEXT("링크 %d: NaN 점성 마찰은 대체되어야 한다 (실제 %g)"), i, Actual.ViscousFrictionNmsPerRad),
			FMath::IsFinite(Actual.ViscousFrictionNmsPerRad) && Actual.ViscousFrictionNmsPerRad >= 0.0);
		TestTrue(FString::Printf(TEXT("링크 %d: 음수 Coulomb 마찰은 대체되어야 한다 (실제 %g)"), i, Actual.CoulombFrictionNm),
			FMath::IsFinite(Actual.CoulombFrictionNm) && Actual.CoulombFrictionNm >= 0.0);
		TestTrue(FString::Printf(TEXT("관절 %d: 0 토크는 기본값으로 대체되어야 한다 (실제 %g)"), i, Model.JointLimits[i].MaxTorqueNm),
			FMath::Abs(Model.JointLimits[i].MaxTorqueNm - Default.JointLimits[i].MaxTorqueNm) <= DynamicsTolerance);
	}

	TestFalse(TEXT("NaN 중력이 모델로 새어나오면 안 된다"), Model.GravityMPerSec2.ContainsNaN());

	// 오염된 동역학 값을 넣어도 기구학은 멀쩡해야 한다.
	TestZeroPoseGolden(*this, TEXT("동역학 값이 오염된 config"), Model);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
