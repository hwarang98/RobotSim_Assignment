// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/DefaultPawn.h"
#include "EndEffectorViewPawn.generated.h"

/**
 * @class AEndEffectorViewPawn
 * @brief target 조작 전용 고정 뷰 폰.
 *
 * ADefaultPawn을 상속하되 기본 이동 바인딩(bAddDefaultMovementBindings)을 끈다.
 * DefaultPawn은 기본적으로 W/A/S/D·Q/E·마우스를 자유비행 카메라 이동에 소비하는데,
 * 그 키들은 이 프로젝트에서 End Effector Target 조작에 써야 하므로 충돌을 막기 위해 비운다.
 * 결과적으로 카메라는 PlayerStart 위치/방향에 고정되고, 모든 조작 키는
 * AEndEffectorPlayerController의 Enhanced Input으로만 전달된다.
 */
UCLASS()
class ROBOTSIM_ASSIGNMENT_API AEndEffectorViewPawn : public ADefaultPawn
{
	GENERATED_BODY()

public:
	AEndEffectorViewPawn();
};
