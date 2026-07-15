// Fill out your copyright notice in the Description page of Project Settings.

#include "Robot/EndEffectorPlayerController.h"

#include "EngineUtils.h"
#include "EnhancedInputComponent.h"
#include "EnhancedInputSubsystems.h"
#include "Engine/LocalPlayer.h"
#include "Engine/World.h"
#include "InputAction.h"
#include "InputActionValue.h"
#include "InputMappingContext.h"
#include "Robot/EndEffectorTargetActor.h"
#include "Robot/RobotSimLog.h"
#include "Robot/RobotTargetInputConfig.h"
#include "Robot/Serial6DoFRobotActor.h"

AEndEffectorPlayerController::AEndEffectorPlayerController()
{
	// 마우스 클릭/드래그로 target을 조작하므로 커서와 클릭 이벤트를 켠다.
	bShowMouseCursor = true;
	bEnableClickEvents = true;
	bEnableMouseOverEvents = true;
}

void AEndEffectorPlayerController::BeginPlay()
{
	Super::BeginPlay();

	// 마우스 커서를 유지하면서 월드 클릭도 받도록 GameAndUI 모드로 설정한다.
	FInputModeGameAndUI InputMode;
	InputMode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
	InputMode.SetHideCursorDuringCapture(false);
	SetInputMode(InputMode);

	// 로봇 참조가 비어 있으면 월드의 첫 로봇을 자동으로 찾는다.
	if (!Robot)
	{
		if (UWorld* World = GetWorld())
		{
			for (TActorIterator<ASerial6DoFRobotActor> It(World); It; ++It)
			{
				Robot = *It;
				break;
			}
		}
		if (Robot)
		{
			UE_LOG(LogRobotSim, Log, TEXT("[AEndEffectorPlayerController] 월드에서 로봇을 자동으로 찾았습니다."));
		}
		else
		{
			UE_LOG(LogRobotSim, Warning, TEXT("[AEndEffectorPlayerController] 조작할 ASerial6DoFRobotActor를 찾지 못했습니다."));
		}
	}

	if (!GetTarget())
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[AEndEffectorPlayerController] Target Actor가 없습니다. 로봇 Details에서 SpawnOrAlignTargetToCurrentEndEffector를 먼저 실행하세요."));
	}

	// Enhanced Input MappingContext 등록 (config/에셋이 유효할 때만).
	if (InputConfig && InputConfig->MappingContext)
	{
		if (ULocalPlayer* LocalPlayer = GetLocalPlayer())
		{
			if (UEnhancedInputLocalPlayerSubsystem* Subsystem =
					LocalPlayer->GetSubsystem<UEnhancedInputLocalPlayerSubsystem>())
			{
				Subsystem->AddMappingContext(InputConfig->MappingContext, InputConfig->MappingPriority);
			}
		}
	}
	else
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[AEndEffectorPlayerController] InputConfig 또는 MappingContext가 비어 있어 키보드/마우스 조작이 비활성화됩니다."));
	}
}

void AEndEffectorPlayerController::SetupInputComponent()
{
	Super::SetupInputComponent();

	if (!InputConfig)
	{
		return; // BeginPlay에서 이미 경고. 액션 바인딩 없이 종료.
	}

	UEnhancedInputComponent* EIC = Cast<UEnhancedInputComponent>(InputComponent);
	if (!EIC)
	{
		UE_LOG(LogRobotSim, Warning,
			TEXT("[AEndEffectorPlayerController] EnhancedInputComponent가 아닙니다. 프로젝트 입력 설정을 확인하세요."));
		return;
	}

	// 개별 Action이 None이면 그 기능만 skip하고 Warning을 남긴다 (나머지는 정상 동작).
	auto BindOrWarn = [&](UInputAction* Action, ETriggerEvent Event, auto Handler, const TCHAR* Name)
	{
		if (Action)
		{
			EIC->BindAction(Action, Event, this, Handler);
		}
		else
		{
			UE_LOG(LogRobotSim, Warning, TEXT("[AEndEffectorPlayerController] InputAction '%s'가 비어 있어 해당 조작이 비활성화됩니다."), Name);
		}
	};

	// 연속 이동/회전: 누르는 동안 매 프레임 Triggered.
	BindOrWarn(InputConfig->MoveXYAction, ETriggerEvent::Triggered, &AEndEffectorPlayerController::OnMoveXY, TEXT("MoveXY"));
	BindOrWarn(InputConfig->MoveZAction, ETriggerEvent::Triggered, &AEndEffectorPlayerController::OnMoveZ, TEXT("MoveZ"));
	BindOrWarn(InputConfig->RotateAction, ETriggerEvent::Triggered, &AEndEffectorPlayerController::OnRotate, TEXT("Rotate"));

	// 이산 명령: 누르는 순간 1회(Started).
	BindOrWarn(InputConfig->SolveIKAction, ETriggerEvent::Started, &AEndEffectorPlayerController::OnSolveIK, TEXT("SolveIK"));
	BindOrWarn(InputConfig->ResetTargetAction, ETriggerEvent::Started, &AEndEffectorPlayerController::OnResetTarget, TEXT("ResetTarget"));
	BindOrWarn(InputConfig->SelectTargetAction, ETriggerEvent::Started, &AEndEffectorPlayerController::OnSelectPressed, TEXT("SelectTarget"));

	// 드래그: 누르는 동안 Triggered로 이동, 놓는 순간 Completed로 종료.
	if (InputConfig->DragTargetAction)
	{
		EIC->BindAction(InputConfig->DragTargetAction, ETriggerEvent::Triggered, this, &AEndEffectorPlayerController::OnDragHeld);
		EIC->BindAction(InputConfig->DragTargetAction, ETriggerEvent::Completed, this, &AEndEffectorPlayerController::OnDragReleased);
		EIC->BindAction(InputConfig->DragTargetAction, ETriggerEvent::Canceled, this, &AEndEffectorPlayerController::OnDragReleased);
	}
	else
	{
		UE_LOG(LogRobotSim, Warning, TEXT("[AEndEffectorPlayerController] InputAction 'DragTarget'이 비어 있어 드래그 이동이 비활성화됩니다."));
	}
}

#pragma region Helpers

AEndEffectorTargetActor* AEndEffectorPlayerController::GetTarget() const
{
	return Robot ? Robot->GetEndEffectorTargetActor() : nullptr;
}

float AEndEffectorPlayerController::GetDeltaSecondsSafe() const
{
	const UWorld* World = GetWorld();
	return World ? World->GetDeltaSeconds() : 0.0f;
}

void AEndEffectorPlayerController::SolveNow()
{
	if (Robot)
	{
		Robot->SolveIKToTarget();
	}
}

void AEndEffectorPlayerController::MoveTargetLocal(const FVector& LocalDeltaCm)
{
	AEndEffectorTargetActor* Target = GetTarget();
	if (!Target || LocalDeltaCm.IsNearlyZero())
	{
		return;
	}

	// 델타는 로봇 액터(베이스) 프레임 기준이다 — 로봇이 회전돼 있어도 "target +X"가 로봇 X를 향한다.
	const FVector WorldDelta = Robot
		? Robot->GetActorTransform().TransformVectorNoScale(LocalDeltaCm)
		: LocalDeltaCm;

	Target->AddActorWorldOffset(WorldDelta);

	if (InputConfig && InputConfig->bAutoSolveOnTargetMove)
	{
		SolveNow();
	}
}

bool AEndEffectorPlayerController::ComputeCursorOnDragPlane(FVector& OutWorld) const
{
	FVector WorldOrigin;
	FVector WorldDirection;
	if (!DeprojectMousePositionToWorld(WorldOrigin, WorldDirection))
	{
		return false;
	}

	// XY 수평면(법선 = +Z, 높이 = DragPlaneZ)과 커서 ray를 교차한다.
	if (FMath::IsNearlyZero(WorldDirection.Z))
	{
		return false; // ray가 평면과 평행 → 교차점 불안정, 무시.
	}

	const FPlane DragPlane(FVector(0.0, 0.0, DragPlaneZ), FVector::UpVector);
	OutWorld = FMath::RayPlaneIntersection(WorldOrigin, WorldDirection, DragPlane);

	const float MaxDist = InputConfig ? InputConfig->DragTraceDistanceCm : 100000.0f;
	if (FVector::Dist(WorldOrigin, OutWorld) > MaxDist)
	{
		return false;
	}
	return true;
}

#pragma endregion

#pragma region InputHandlers

void AEndEffectorPlayerController::OnMoveXY(const FInputActionValue& Value)
{
	const FVector2D Axis = Value.Get<FVector2D>();
	const float Speed = InputConfig ? InputConfig->MoveSpeedCmPerSec : 100.0f;
	MoveTargetLocal(FVector(Axis.X, Axis.Y, 0.0) * Speed * GetDeltaSecondsSafe());
}

void AEndEffectorPlayerController::OnMoveZ(const FInputActionValue& Value)
{
	const float Axis = Value.Get<float>();
	const float Speed = InputConfig ? InputConfig->MoveSpeedCmPerSec : 100.0f;
	MoveTargetLocal(FVector(0.0, 0.0, Axis) * Speed * GetDeltaSecondsSafe());
}

void AEndEffectorPlayerController::OnRotate(const FInputActionValue& Value)
{
	AEndEffectorTargetActor* Target = GetTarget();
	if (!Target)
	{
		return;
	}

	const FVector Axis = Value.Get<FVector>(); // X=pitch, Y=yaw, Z=roll
	const float Speed = InputConfig ? InputConfig->RotateSpeedDegPerSec : 60.0f;
	const float Step = Speed * GetDeltaSecondsSafe();

	// Euler는 입력 경계에서만 사용하고, 실제 합성은 FQuat로 한다 (winding 손실 방지).
	const FRotator DeltaRot(Axis.X * Step, Axis.Y * Step, Axis.Z * Step);
	if (DeltaRot.IsNearlyZero())
	{
		return;
	}

	Target->AddActorLocalRotation(DeltaRot.Quaternion());

	if (InputConfig && InputConfig->bAutoSolveOnTargetMove)
	{
		SolveNow();
	}
}

void AEndEffectorPlayerController::OnSolveIK(const FInputActionValue& /*Value*/)
{
	SolveNow();
}

void AEndEffectorPlayerController::OnResetTarget(const FInputActionValue& /*Value*/)
{
	if (Robot)
	{
		Robot->CopyCurrentEndEffectorToTarget();
	}
}

void AEndEffectorPlayerController::OnSelectPressed(const FInputActionValue& /*Value*/)
{
	AEndEffectorTargetActor* Target = GetTarget();

	FHitResult Hit;
	if (GetHitResultUnderCursor(ECC_Visibility, false, Hit) && Target && Hit.GetActor() == Target)
	{
		Target->SetSelected(true);
		bDragging = true;
		DragPlaneZ = Target->GetActorLocation().Z; // 드래그 평면 높이를 클릭 순간 고정.
		return;
	}

	// target을 못 맞혔으면 선택 해제하고 드래그도 시작하지 않는다.
	if (Target)
	{
		Target->SetSelected(false);
	}
	bDragging = false;
}

void AEndEffectorPlayerController::OnDragHeld(const FInputActionValue& /*Value*/)
{
	if (!bDragging)
	{
		return;
	}

	AEndEffectorTargetActor* Target = GetTarget();
	FVector PlanePoint;
	if (Target && ComputeCursorOnDragPlane(PlanePoint))
	{
		// XY만 갱신하고 Z는 유지한다 (Z 이동은 Q/E 키가 담당).
		FVector NewLocation = Target->GetActorLocation();
		NewLocation.X = PlanePoint.X;
		NewLocation.Y = PlanePoint.Y;
		Target->SetActorLocation(NewLocation);

		if (InputConfig && InputConfig->bAutoSolveOnTargetMove)
		{
			SolveNow();
		}
	}
}

void AEndEffectorPlayerController::OnDragReleased(const FInputActionValue& /*Value*/)
{
	if (!bDragging)
	{
		return;
	}
	bDragging = false;

	// auto-solve가 아니면 드래그를 놓는 순간 1회 solve (설정 시).
	if (InputConfig && !InputConfig->bAutoSolveOnTargetMove && InputConfig->bSolveOnDragRelease)
	{
		SolveNow();
	}
}

#pragma endregion
