// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "RobotTargetInputConfig.generated.h"

class UInputMappingContext;
class UInputAction;

/**
 * @class URobotTargetInputConfig
 * @brief End Effector Target 조작용 Enhanced Input 배선과 튜닝값을 한곳에 모은 DataAsset.
 *
 * Enhanced Input의 InputAction / InputMappingContext는 코드에서 생성하지 않고
 * 에디터에서 .uasset으로 authoring한다. 이 DataAsset은 그 에셋들의 참조와
 * target 이동/회전 속도 같은 입력 튜닝값만 소유한다. AEndEffectorPlayerController는
 * 이 config 하나만 참조하면 되므로, 컨트롤러에 입력 설정을 개별 UPROPERTY로 흩뿌리지 않는다.
 *
 * 배선 구조:
 *   InputAction 에셋들 + InputMappingContext 에셋
 *       ↓ 참조
 *   URobotTargetInputConfig (이 에셋)
 *       ↓ 할당
 *   AEndEffectorPlayerController
 *
 * MappingContext 또는 개별 Action이 None이면 컨트롤러는 Warning을 남기고 해당 기능만 비활성화한다
 * (나머지 조작은 그대로 동작). 에셋 생성 절차는 Docs/Steps/STEP_A-06.md 참조.
 */
UCLASS(BlueprintType)
class ROBOTSIM_ASSIGNMENT_API URobotTargetInputConfig : public UDataAsset
{
	GENERATED_BODY()

public:
	//~ 입력 배선 (에디터에서 만든 Enhanced Input 에셋 참조) -----------------------

	/** target 조작 키/마우스를 담은 InputMappingContext 에셋 (예: IMC_RobotTargetControl). */
	UPROPERTY(EditAnywhere, Category = "Robot|Input")
	TObjectPtr<UInputMappingContext> MappingContext;

	/** MappingContext 우선순위. 다른 IMC와 겹칠 때만 조정하면 된다. */
	UPROPERTY(EditAnywhere, Category = "Robot|Input")
	int32 MappingPriority = 0;

	//~ InputAction 에셋들 --------------------------------------------------------

	/** target XY 평면 이동 (Axis2D: X=+X/−X, Y=+Y/−Y). 예: W/S, A/D. */
	UPROPERTY(EditAnywhere, Category = "Robot|Input|Actions")
	TObjectPtr<UInputAction> MoveXYAction;

	/** target Z 이동 (Axis1D: +Z/−Z). 예: Q/E. */
	UPROPERTY(EditAnywhere, Category = "Robot|Input|Actions")
	TObjectPtr<UInputAction> MoveZAction;

	/** target 회전 (Axis3D: X=pitch, Y=yaw, Z=roll). 예: I/K, J/L, U/O. */
	UPROPERTY(EditAnywhere, Category = "Robot|Input|Actions")
	TObjectPtr<UInputAction> RotateAction;

	/** DLS IK solve 실행 (Bool). 예: Space/Enter. */
	UPROPERTY(EditAnywhere, Category = "Robot|Input|Actions")
	TObjectPtr<UInputAction> SolveIKAction;

	/** target을 현재 End Effector로 리셋 (Bool). 예: R. */
	UPROPERTY(EditAnywhere, Category = "Robot|Input|Actions")
	TObjectPtr<UInputAction> ResetTargetAction;

	/** 마우스로 target 선택 (Bool). 예: 왼쪽 마우스 버튼(누름). */
	UPROPERTY(EditAnywhere, Category = "Robot|Input|Actions")
	TObjectPtr<UInputAction> SelectTargetAction;

	/** 마우스로 target 드래그 (Bool, 누르는 동안 유지). 예: 왼쪽 마우스 버튼(홀드). */
	UPROPERTY(EditAnywhere, Category = "Robot|Input|Actions")
	TObjectPtr<UInputAction> DragTargetAction;

	//~ 조작 튜닝값 ---------------------------------------------------------------

	/** 키보드 이동 속도 (cm/초). 키를 누르는 동안 매 프레임 속도×Δt만큼 이동한다. */
	UPROPERTY(EditAnywhere, Category = "Robot|Input|Tuning", meta = (ClampMin = "0.0"))
	float MoveSpeedCmPerSec = 100.0f;

	/** 키보드 회전 속도 (도/초). 누르는 동안 매 프레임 속도×Δt만큼 회전한다. */
	UPROPERTY(EditAnywhere, Category = "Robot|Input|Tuning", meta = (ClampMin = "0.0"))
	float RotateSpeedDegPerSec = 60.0f;

	/** 드래그 시 커서 ray를 평면과 교차시킬 때 허용하는 최대 거리 (cm). 이보다 멀면 무시. */
	UPROPERTY(EditAnywhere, Category = "Robot|Input|Tuning", meta = (ClampMin = "0.0"))
	float DragTraceDistanceCm = 100000.0f;

	/** true면 키보드/드래그로 target이 바뀌는 즉시 매번 IK를 푼다 (기본 false: 명시적 solve만). */
	UPROPERTY(EditAnywhere, Category = "Robot|Input|Tuning")
	bool bAutoSolveOnTargetMove = false;

	/** bAutoSolveOnTargetMove가 false일 때, 드래그를 놓는 순간 1회 IK를 풀지 여부 (기본 true). */
	UPROPERTY(EditAnywhere, Category = "Robot|Input|Tuning")
	bool bSolveOnDragRelease = true;
};
