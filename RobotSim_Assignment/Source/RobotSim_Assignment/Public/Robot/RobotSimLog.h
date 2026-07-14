// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

/**
 * 로봇 시뮬레이션 전용 로그 카테고리.
 * FK/IK 계산 결과, 관절 상태, 수학-비주얼 일관성 검증 등 로봇 관련 로그는 모두 이 카테고리로 출력한다.
 * Output Log에서 "LogRobotSim" 필터로 로봇 로그만 모아볼 수 있음.
 */
DECLARE_LOG_CATEGORY_EXTERN(LogRobotSim, Log, All);
