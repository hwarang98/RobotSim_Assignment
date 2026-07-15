// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "EndEffectorGameMode.generated.h"

/**
 * @class AEndEffectorGameMode
 * @brief End Effector Target 조작용 기본 GameMode.
 *
 * PlayerControllerClass를 AEndEffectorPlayerController로 지정해 PIE 진입 시 target 조작 입력이
 * 자동으로 배선되게 한다. 관전용 DefaultPawn을 스폰해 뷰포트 카메라를 제공한다.
 * 맵의 World Settings → GameMode Override에 이 클래스를 지정하면 된다.
 */
UCLASS()
class ROBOTSIM_ASSIGNMENT_API AEndEffectorGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AEndEffectorGameMode();
};
