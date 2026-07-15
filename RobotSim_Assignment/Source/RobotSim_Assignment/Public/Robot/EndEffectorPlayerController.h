// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"
#include "EndEffectorPlayerController.generated.h"

class ASerial6DoFRobotActor;
class AEndEffectorTargetActor;
class URobotTargetInputConfig;
struct FInputActionValue;

/**
 * @class AEndEffectorPlayerController
 * @brief End Effector Target을 키보드/마우스로 조작하는 입력 소유 컨트롤러.
 *
 * 책임 분리 원칙: 이 컨트롤러는 입력만 담당한다. 수학(IK)은 직접 호출하지 않고
 * 로봇 액터에 명령을 위임한다(Robot->SolveIKToTarget / CopyCurrentEndEffectorToTarget).
 * 조작 대상 target은 로봇이 소유(Robot->GetEndEffectorTargetActor())하므로 단일 소스를 공유한다.
 *
 * Enhanced Input 배선(IMC/IA)과 튜닝값은 URobotTargetInputConfig DataAsset이 소유한다.
 * IMC/IA는 코드에서 만들지 않고 에디터에서 authoring한 .uasset을 참조한다.
 *
 * 마우스 드래그는 XY 수평면(target 현재 Z 높이 유지)에 커서 ray를 교차시켜 위치만 옮긴다.
 * 회전은 키보드(I/K/J/L/U/O)가 담당한다 — full transform gizmo가 아니다.
 */
UCLASS()
class ROBOTSIM_ASSIGNMENT_API AEndEffectorPlayerController : public APlayerController
{
	GENERATED_BODY()

public:
	AEndEffectorPlayerController();

protected:
	virtual void BeginPlay() override;
	virtual void SetupInputComponent() override;

	/** 조작할 로봇. 비어 있으면 BeginPlay에서 월드의 첫 로봇을 자동으로 찾는다. */
	UPROPERTY(EditAnywhere, Category = "Robot|Input")
	TObjectPtr<ASerial6DoFRobotActor> Robot;

	/** Enhanced Input 배선/튜닝을 담은 DataAsset (에디터에서 할당). */
	UPROPERTY(EditDefaultsOnly, Category = "Robot|Input")
	TObjectPtr<URobotTargetInputConfig> InputConfig;

private:
	#pragma region InputHandlers

	void OnMoveXY(const FInputActionValue& Value);
	void OnMoveZ(const FInputActionValue& Value);
	void OnRotate(const FInputActionValue& Value);
	void OnSolveIK(const FInputActionValue& Value);
	void OnResetTarget(const FInputActionValue& Value);
	void OnSelectPressed(const FInputActionValue& Value);
	void OnDragHeld(const FInputActionValue& Value);
	void OnDragReleased(const FInputActionValue& Value);

	#pragma endregion

	/** 로봇이 소유한 현재 target 액터 (없으면 nullptr). */
	AEndEffectorTargetActor* GetTarget() const;

	/** 로봇 액터 프레임 기준 델타(cm)를 월드로 변환해 target을 이동한다. */
	void MoveTargetLocal(const FVector& LocalDeltaCm);

	/** 로봇에 IK solve를 위임한다. */
	void SolveNow();

	/** 커서 ray를 드래그 평면(Z=DragPlaneZ)과 교차시켜 월드 좌표를 구한다. */
	bool ComputeCursorOnDragPlane(FVector& OutWorld) const;

	/** 현재 프레임 델타초 (0 안전). */
	float GetDeltaSecondsSafe() const;

	/** 드래그 진행 중 여부. */
	bool bDragging = false;

	/** 드래그 시작 시 고정한 target Z 높이 (XY 평면 정의). */
	double DragPlaneZ = 0.0;
};
