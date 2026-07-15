// Fill out your copyright notice in the Description page of Project Settings.

#include "Robot/EndEffectorGameMode.h"

#include "Robot/EndEffectorPlayerController.h"
#include "Robot/EndEffectorViewPawn.h"

AEndEffectorGameMode::AEndEffectorGameMode()
{
	PlayerControllerClass = AEndEffectorPlayerController::StaticClass();

	// 이동 바인딩을 끈 고정 뷰 폰. 기본 ADefaultPawn은 W/A/S/D·Q/E·마우스를 자유비행 카메라 이동에
	// 소비해 target 조작 키와 충돌하므로, 그 바인딩을 비운 전용 폰을 쓴다 (뷰는 PlayerStart에 고정).
	DefaultPawnClass = AEndEffectorViewPawn::StaticClass();
}
