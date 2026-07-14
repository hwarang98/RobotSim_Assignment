// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Robot/RobotTypes.h"
#include "Robot/Serial6DoFModel.h"
#include "Serial6DoFRobotActor.generated.h"

class UStaticMesh;
class UStaticMeshComponent;

/**
 * @class ASerial6DoFRobotActor
 * @brief 6R serial 로봇의 비주얼 표현 액터.
 *
 * J0~J5 6개 revolute 관절을 SceneComponent 부모-자식 체인으로 구성하고,
 * 링크는 엔진 기본 도형(Cube/Cylinder) 디버그 메시로 표시한다.
 *
 * @details
 * 관절 축과 링크 오프셋의 유일한 정의처는 FSerial6DoFModel이며, 생성자에서
 * 모델 값을 그대로 컴포넌트 계층에 미러링한다 (이중 정의 금지).
 * 따라서 컴포넌트의 월드 트랜스폼과 수학 FK 결과는 항상 일치해야 하고,
 * 그 일치 여부를 CheckVisualMatchesMath()로 매 적용 시 검증한다.
 *
 * 각도 단위 규약: 내부 상태(CurrentState)는 radian, 에디터 노출 프로퍼티
 * (JointAnglesDeg)만 도(degree)를 사용한다.
 */
UCLASS()
class ROBOTSIM_ASSIGNMENT_API ASerial6DoFRobotActor : public AActor
{
	GENERATED_BODY()

public:
	ASerial6DoFRobotActor();

	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void Tick(float DeltaSeconds) override;

	/** 에디터 뷰포트에서도 Tick을 돌려 디버그 축을 PIE 밖에서도 볼 수 있게 한다. */
	virtual bool ShouldTickIfViewportsOnly() const override { return true; }

#if WITH_EDITOR
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

	#pragma region RobotAPI

	/**
	 * @brief 관절 각도를 설정한다 (radian).
	 *
	 * 입력을 모델의 관절 한계로 클램프한 뒤 저장하고 즉시 컴포넌트에 반영한다.
	 */
	void SetJointAngles(const FRobot6DJointState& NewState);

	/** End Effector의 월드 공간 6D 자세를 수학 FK로 계산해 반환한다. */
	FRobot6DPose GetEndEffectorPose() const;

	/** 현재 관절 상태(CurrentState)를 SceneComponent 계층에 반영한다. */
	void ApplyJointState();

	FORCEINLINE const FSerial6DoFModel& GetModel() const { return Model; }
	FORCEINLINE const FRobot6DJointState& GetJointState() const { return CurrentState; }

	#pragma endregion

	#pragma region EditorUtility

	/** 현재 EE 자세를 로그로 출력하고 수학-비주얼 일관성을 검증한다. */
	UFUNCTION(CallInEditor, Category = "Robot")
	void LogEndEffectorPose();

	/** 모든 관절 각도를 0으로 초기화한다. */
	UFUNCTION(CallInEditor, Category = "Robot")
	void ResetJointAngles();

	#pragma endregion

protected:
	#pragma region EditorProperties

	/**
	 * J0~J5 관절 각도 (도 단위, 에디터 조작용).
	 * 관절 한계를 벗어난 입력은 적용 시 한계값으로 클램프되어 되돌아 써진다.
	 */
	UPROPERTY(EditAnywhere, Category = "Robot|Joints", meta = (UIMin = "-350.0", UIMax = "350.0"))
	double JointAnglesDeg[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

	/** 각 관절 프레임과 EE 프레임에 디버그 좌표축을 그릴지 여부 */
	UPROPERTY(EditAnywhere, Category = "Robot|Debug")
	bool bDrawDebugFrames = true;

	/** EE 자세 주기 로그 간격 (초). 0 이하면 주기 로그를 끈다. */
	UPROPERTY(EditAnywhere, Category = "Robot|Debug", meta = (ClampMin = "0.0"))
	float PoseLogIntervalSeconds = 1.0f;

	#pragma endregion

	#pragma region Components

	/** J0~J5 관절 프레임. 부모-자식 체인으로 연결되며 상대 위치는 모델의 LinkOffsets를 따른다. */
	UPROPERTY(VisibleAnywhere, Category = "Robot")
	TObjectPtr<USceneComponent> JointComponents[6];

	/** End Effector(툴 끝단) 프레임. J5의 자식이며 상대 변환은 모델의 ToolOffset을 따른다. */
	UPROPERTY(VisibleAnywhere, Category = "Robot")
	TObjectPtr<USceneComponent> ToolTipComponent;

	/** 링크 디버그 메시 (비주얼 전용, 충돌 없음, FK에 영향 없음) */
	UPROPERTY(VisibleAnywhere, Category = "Robot")
	TArray<TObjectPtr<UStaticMeshComponent>> LinkMeshComponents;

	#pragma endregion

private:
	/** 기구학 파라미터의 유일한 정의처. 생성자에서 CreateDefault()로 초기화한다. */
	FSerial6DoFModel Model;

	/** 현재 관절 상태 (radian) */
	FRobot6DJointState CurrentState;

	/** 주기 로그 타이머 */
	double TimeSinceLastPoseLog = 0.0;

	/** JointAnglesDeg(도)를 radian으로 변환해 SetJointAngles를 호출하고, 클램프 결과를 도로 되돌려 쓴다. */
	void ApplyAnglesFromEditor();

	/** ToolTip 컴포넌트의 월드 트랜스폼과 수학 FK 결과를 비교해 불일치 시 Warning을 남긴다. */
	void CheckVisualMatchesMath() const;

	/** 관절 프레임/EE에 디버그 좌표축을 그린다. */
	void DrawDebugJointFrames() const;

	/** 링크 디버그 메시 컴포넌트를 생성하는 생성자 전용 헬퍼 */
	UStaticMeshComponent* CreateLinkMesh(
		USceneComponent* Parent, const FName& Name, UStaticMesh* Mesh,
		const FVector& RelativeLocation, const FRotator& RelativeRotation, const FVector& RelativeScale);
};
