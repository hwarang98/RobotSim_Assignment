// Fill out your copyright notice in the Description page of Project Settings.

#include "Robot/RobotDlsIK.h"

#include "Robot/Serial6DoFModel.h"
#include "Robot/RobotJacobian.h"
#include "Robot/RobotPoseError.h"
#include "Robot/RobotSimLog.h"

#pragma region LinearAlgebraHelpers
namespace
{
	/** 6x6 행/열 크기 (Jacobian이 square이므로 위치3 + 회전3 = 6 고정). */
	constexpr int32 IKDim = 6;

	/**
	 * 부분 피벗을 사용하는 6×6 Gaussian elimination으로 A x = b를 푼다.
	 *
	 * DLS에서 A = J Jᵀ + λ² I는 대칭 양의(준)정부호이지만, damping이 작고
	 * singularity에 가까우면 피벗이 매우 작아질 수 있다. 피벗 절댓값이 임계값
	 * 미만이거나 결과에 NaN/Inf가 생기면 false를 반환해 호출부에서 안전 종료한다.
	 *
	 * @return 성공 시 true(OutX 채움), 특이/비정상 시 false.
	 */
	bool SolveLinearSystem6(const double InA[IKDim][IKDim], const double InB[IKDim], double OutX[IKDim])
	{
		// 파괴적 소거를 위해 로컬 복사본을 만든다.
		double A[IKDim][IKDim];
		double B[IKDim];
		for (int32 r = 0; r < IKDim; ++r)
		{
			B[r] = InB[r];
			for (int32 c = 0; c < IKDim; ++c)
			{
				A[r][c] = InA[r][c];
			}
		}

		// 피벗이 이보다 작으면 특이 행렬로 간주한다.
		constexpr double PivotThreshold = 1e-12;

		for (int32 Col = 0; Col < IKDim; ++Col)
		{
			// 부분 피벗: Col 열에서 절댓값이 가장 큰 행을 찾아 교환한다.
			int32 PivotRow = Col;
			double PivotMax = FMath::Abs(A[Col][Col]);
			for (int32 r = Col + 1; r < IKDim; ++r)
			{
				const double Candidate = FMath::Abs(A[r][Col]);
				if (Candidate > PivotMax)
				{
					PivotMax = Candidate;
					PivotRow = r;
				}
			}

			if (!(PivotMax > PivotThreshold))
			{
				return false; // 특이(singular) 또는 damping 부족.
			}

			if (PivotRow != Col)
			{
				for (int32 c = 0; c < IKDim; ++c)
				{
					Swap(A[PivotRow][c], A[Col][c]);
				}
				Swap(B[PivotRow], B[Col]);
			}

			// 아래 행들에서 Col 성분을 소거한다.
			const double Pivot = A[Col][Col];
			for (int32 r = Col + 1; r < IKDim; ++r)
			{
				const double Factor = A[r][Col] / Pivot;
				if (Factor == 0.0)
				{
					continue;
				}
				for (int32 c = Col; c < IKDim; ++c)
				{
					A[r][c] -= Factor * A[Col][c];
				}
				B[r] -= Factor * B[Col];
			}
		}

		// 후진 대입(back substitution).
		for (int32 r = IKDim - 1; r >= 0; --r)
		{
			double Sum = B[r];
			for (int32 c = r + 1; c < IKDim; ++c)
			{
				Sum -= A[r][c] * OutX[c];
			}
			const double Diag = A[r][r];
			if (!(FMath::Abs(Diag) > PivotThreshold))
			{
				return false;
			}
			OutX[r] = Sum / Diag;
		}

		// 결과 유효성 검사.
		for (int32 r = 0; r < IKDim; ++r)
		{
			if (!FMath::IsFinite(OutX[r]))
			{
				return false;
			}
		}
		return true;
	}
} // namespace
#pragma endregion

#pragma region SolveDlsIK
FRobotDlsIKResult FRobotDlsIK::SolveDlsIK(
	const FSerial6DoFModel& Model,
	const FRobot6DJointState& InitialState,
	const FTransform& TargetTransform,
	const FRobotDlsIKOptions& Options)
{
	FRobotDlsIKResult Result;

	// 반복 대상 상태(로컬 복사본). 입력 InitialState는 변경하지 않는다.
	FRobot6DJointState State = InitialState;
	if (Options.bClampJointLimits)
	{
		State = Model.ClampToLimits(State);
	}

	const double Lambda2 = Options.DampingLambda * Options.DampingLambda;

	// 실패 시에도 항상 마지막 유효 상태/오차를 채워 반환할 수 있도록 헬퍼로 마무리한다.
	auto FinalizeResult = [&](bool bConverged, int32 Iterations)
	{
		Result.bConverged = bConverged;
		Result.Iterations = Iterations;
		Result.Solution = State;

		const FTransform FinalTransform = Model.ComputeEndEffectorTransform(State);
		Result.FinalError = FRobotPoseError::ComputePoseError(FinalTransform, TargetTransform);
		Result.FinalPositionErrorCm = Result.FinalError.PositionErrorNorm();
		Result.FinalRotationErrorRad = Result.FinalError.RotationErrorNorm();
	};

	int32 Iteration = 0;
	for (; Iteration < Options.MaxIterations; ++Iteration)
	{
		// 1) 현재 EE 변환과 오차(target − current).
		const FTransform CurrentTransform = Model.ComputeEndEffectorTransform(State);
		const FRobot6DPoseError Error = FRobotPoseError::ComputePoseError(CurrentTransform, TargetTransform);

		// 2) 수렴 검사(가중치 미적용 원 단위로 판정).
		if (Error.PositionErrorNorm() <= Options.PositionToleranceCm &&
			Error.RotationErrorNorm() <= Options.RotationToleranceRad)
		{
			FinalizeResult(/*bConverged*/ true, Iteration);
			UE_LOG(LogRobotSim, Verbose,
				TEXT("[FRobotDlsIK] 수렴: %d회 반복, 위치오차 %.4fcm, 회전오차 %.4frad"),
				Result.Iterations, Result.FinalPositionErrorCm, Result.FinalRotationErrorRad);
			return Result;
		}

		// 3) 가중 오차 6-벡터 e = [Wp·pos, Wr·rot].
		double e[IKDim];
		Error.ToArray6(e);
		for (int32 Row = 0; Row < 3; ++Row)
		{
			e[Row] *= Options.PositionWeight;
			e[Row + 3] *= Options.RotationWeight;
		}

		// 4) numerical Jacobian을 계산하고 오차와 동일하게 row scaling.
		const FRobotJacobian6x6 J = FRobotJacobian::ComputeNumericalJacobian(Model, State, Options.JacobianEpsilonRad);
		double Jw[IKDim][IKDim];
		for (int32 Col = 0; Col < IKDim; ++Col)
		{
			for (int32 Row = 0; Row < 3; ++Row)
			{
				Jw[Row][Col]     = J.At(Row, Col)     * Options.PositionWeight;
				Jw[Row + 3][Col] = J.At(Row + 3, Col) * Options.RotationWeight;
			}
		}

		// 5) DLS: A = J Jᵀ + λ² I (6×6), A y = e 를 풀고 dq = Jᵀ y.
		double A[IKDim][IKDim];
		for (int32 r = 0; r < IKDim; ++r)
		{
			for (int32 c = 0; c < IKDim; ++c)
			{
				double Acc = 0.0;
				for (int32 k = 0; k < IKDim; ++k)
				{
					Acc += Jw[r][k] * Jw[c][k]; // (J Jᵀ)[r][c]
				}
				A[r][c] = Acc + ((r == c) ? Lambda2 : 0.0);
			}
		}

		double y[IKDim];
		if (!SolveLinearSystem6(A, e, y))
		{
			// 특이/비정상: 실패로 안전 종료(마지막 유효 State 유지).
			FinalizeResult(/*bConverged*/ false, Iteration);
			UE_LOG(LogRobotSim, Warning,
				TEXT("[FRobotDlsIK] 선형계 풀이 실패(특이 행렬 추정): %d회 반복 후 종료. λ를 키우거나 target을 조정하세요. ")
				TEXT("위치오차 %.4fcm, 회전오차 %.4frad"),
				Result.Iterations, Result.FinalPositionErrorCm, Result.FinalRotationErrorRad);
			return Result;
		}

		// dq = Jᵀ y  (dq[j] = Σ_i Jw[i][j] · y[i])
		double dq[IKDim];
		for (int32 j = 0; j < IKDim; ++j)
		{
			double Acc = 0.0;
			for (int32 i = 0; i < IKDim; ++i)
			{
				Acc += Jw[i][j] * y[i];
			}
			dq[j] = Acc;
		}

		// 6) dq NaN/Inf 검사 후 MaxStepRad로 clamp.
		bool bStepFinite = true;
		for (int32 j = 0; j < IKDim; ++j)
		{
			if (!FMath::IsFinite(dq[j]))
			{
				bStepFinite = false;
				break;
			}
			dq[j] = FMath::Clamp(dq[j], -Options.MaxStepRad, Options.MaxStepRad);
		}

		if (!bStepFinite)
		{
			FinalizeResult(/*bConverged*/ false, Iteration);
			UE_LOG(LogRobotSim, Warning,
				TEXT("[FRobotDlsIK] Δq에 NaN/Inf 발생: %d회 반복 후 종료. 마지막 유효 상태를 반환합니다."),
				Result.Iterations);
			return Result;
		}

		// 7) 상태 갱신 + (옵션) 한계 clamp.
		for (int32 j = 0; j < IKDim; ++j)
		{
			State.Q[j] += dq[j];
		}
		if (Options.bClampJointLimits)
		{
			State = Model.ClampToLimits(State);
		}
	}

	// 반복 소진: 미수렴이지만 오차는 줄어든 상태일 수 있다.
	FinalizeResult(/*bConverged*/ false, Iteration);
	UE_LOG(LogRobotSim, Verbose,
		TEXT("[FRobotDlsIK] 미수렴: 최대 반복(%d) 소진. 위치오차 %.4fcm, 회전오차 %.4frad (도달 불가/local minimum 가능)"),
		Result.Iterations, Result.FinalPositionErrorCm, Result.FinalRotationErrorRad);
	return Result;
}
#pragma endregion
