// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "EndEffectorTargetActor.generated.h"

class UStaticMeshComponent;

/**
 * @class AEndEffectorTargetActor
 * @brief 월드에 표시되는 End Effector IK 목표 Transform.
 *
 * 이 액터의 GetActorTransform()이 곧 IK target이다. 로봇 수학(FSerial6DoFModel/FRobotDlsIK)에는
 * 어떤 영향도 주지 않는다 — 순수 interaction/시각 레이어다. 로봇의 SolveIKToTarget이 이 액터의
 * 월드 트랜스폼을 읽어 로봇 모델 공간으로 변환한 뒤 solver에 넘긴다.
 *
 * 클릭 선택을 위해 마커 메시에 QueryOnly 컬리전(Visibility 채널 Block)을 준다. 로봇 링크 메시는
 * NoCollision이므로 커서 트레이스에는 이 target만 잡힌다. 선택/호버 상태와 좌표축은 DrawDebug로
 * 표시하므로 머티리얼 파라미터에 의존하지 않는다.
 */
UCLASS()
class ROBOTSIM_ASSIGNMENT_API AEndEffectorTargetActor : public AActor
{
	GENERATED_BODY()

public:
	AEndEffectorTargetActor();

	virtual void Tick(float DeltaSeconds) override;

	/** 에디터 뷰포트에서도 Tick을 돌려 디버그 축/색을 PIE 밖에서도 볼 수 있게 한다. */
	virtual bool ShouldTickIfViewportsOnly() const override { return true; }

	/** 선택 상태를 설정한다 (색 피드백에만 사용, 수학 무관). */
	void SetSelected(bool bInSelected) { bSelected = bInSelected; }

	/** 호버 상태를 설정한다 (색 피드백에만 사용, 수학 무관). */
	void SetHovered(bool bInHovered) { bHovered = bInHovered; }

	/** 현재 선택 상태. */
	bool IsSelected() const { return bSelected; }

protected:
	/** 액터 루트 (이 프레임이 IK target). */
	UPROPERTY(VisibleAnywhere, Category = "Target")
	TObjectPtr<USceneComponent> SceneRoot;

	/** 클릭 감지 + 위치 표시용 작은 sphere 마커 (QueryOnly 컬리전). */
	UPROPERTY(VisibleAnywhere, Category = "Target")
	TObjectPtr<UStaticMeshComponent> MarkerMesh;

	/** target 좌표축을 DrawDebug로 그릴지 여부. */
	UPROPERTY(EditAnywhere, Category = "Target")
	bool bDrawDebugAxes = true;

	/** 디버그 좌표축 길이 (cm). */
	UPROPERTY(EditAnywhere, Category = "Target")
	float AxisLengthCm = 20.0f;

	/** 비선택 상태 표시 색. */
	UPROPERTY(EditAnywhere, Category = "Target")
	FColor TargetColor = FColor(60, 200, 255);

	/** 선택 상태 표시 색. */
	UPROPERTY(EditAnywhere, Category = "Target")
	FColor SelectedColor = FColor(255, 200, 40);

	/** 현재 선택 여부 (컨트롤러가 클릭으로 설정). */
	UPROPERTY(VisibleAnywhere, Category = "Target")
	bool bSelected = false;

	/** 현재 호버 여부. */
	UPROPERTY(VisibleAnywhere, Category = "Target")
	bool bHovered = false;
};
