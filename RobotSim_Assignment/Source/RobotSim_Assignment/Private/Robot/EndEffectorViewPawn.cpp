// Fill out your copyright notice in the Description page of Project Settings.

#include "Robot/EndEffectorViewPawn.h"

AEndEffectorViewPawn::AEndEffectorViewPawn()
{
	// DefaultPawn의 자유비행 이동/회전 바인딩을 끈다.
	// 이렇게 하면 W/A/S/D·Q/E·마우스가 카메라 이동에 소비되지 않고
	// target 조작 입력(Enhanced Input)으로 온전히 전달된다.
	bAddDefaultMovementBindings = false;
}
