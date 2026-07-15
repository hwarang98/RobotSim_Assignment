// Fill out your copyright notice in the Description page of Project Settings.

#include "Robot/EndEffectorTargetActor.h"

#include "Components/StaticMeshComponent.h"
#include "DrawDebugHelpers.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "UObject/ConstructorHelpers.h"

AEndEffectorTargetActor::AEndEffectorTargetActor()
{
	PrimaryActorTick.bCanEverTick = true;

	SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
	SetRootComponent(SceneRoot);

	// 위치 표시 + 클릭 감지용 작은 sphere. 엔진 기본 도형(지름 100cm)을 작게 스케일한다.
	MarkerMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MarkerMesh"));
	MarkerMesh->SetupAttachment(SceneRoot);
	MarkerMesh->SetRelativeScale3D(FVector(0.08));

	static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereFinder(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	if (SphereFinder.Succeeded())
	{
		MarkerMesh->SetStaticMesh(SphereFinder.Object);
	}

	// 커서 트레이스(GetHitResultUnderCursor: Visibility 채널)에 잡히도록 QueryOnly 컬리전을 준다.
	// 물리/오버랩에는 관여하지 않으며, 로봇 링크(NoCollision)와 달리 이 target만 클릭된다.
	MarkerMesh->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
	MarkerMesh->SetCollisionObjectType(ECC_WorldDynamic);
	MarkerMesh->SetCollisionResponseToAllChannels(ECR_Ignore);
	MarkerMesh->SetCollisionResponseToChannel(ECC_Visibility, ECR_Block);
	MarkerMesh->SetGenerateOverlapEvents(false);
}

void AEndEffectorTargetActor::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	UWorld* World = GetWorld();
	if (!World)
	{
		return;
	}

	const FTransform TargetTransform = GetActorTransform();
	const FColor DrawColor = bSelected ? SelectedColor : TargetColor;

	// 좌표축: target의 방향(회전)을 그대로 보여준다 (키보드 회전 결과 확인용).
	if (bDrawDebugAxes)
	{
		DrawDebugCoordinateSystem(World, TargetTransform.GetLocation(), TargetTransform.Rotator(),
			AxisLengthCm, false, -1.0f, 0, bSelected ? 1.5f : 0.8f);
	}

	// 선택/호버 색 피드백: 마커 위에 반투명 sphere를 덧그린다 (머티리얼 파라미터 비의존).
	const float Radius = bHovered ? 6.0f : 5.0f;
	DrawDebugSphere(World, TargetTransform.GetLocation(), Radius, 12, DrawColor, false, -1.0f, 0, bSelected ? 1.2f : 0.6f);
}
