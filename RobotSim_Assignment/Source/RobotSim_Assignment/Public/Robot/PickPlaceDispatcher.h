// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Robot/PickPlaceTaskActor.h"
#include "Templates/SubclassOf.h"
#include "PickPlaceDispatcher.generated.h"

/**
 * @class APickPlaceDispatcher
 * @brief 여러 로봇에게 pick&place 작업을 배급하는 중재자 — 멀티로봇 **작업 할당**의 주체.
 *
 * @details
 * **왜 필요한가**: `APickPlaceTaskActor`만으로 로봇을 2대 놓으면 각자 에디터에서 미리 지정된 자기
 * `Boxes` 배열을 순서대로 처리한다. 그건 독립 실행 2개지 할당이 아니다 — 배분하는 주체가 없다.
 * 이 액터가 박스와 슬롯의 **소유권을 가져와** 어떤 로봇이 어떤 박스를 맡을지 정한다.
 *
 * ## 요청이 아니라 배급 (결정론의 근거)
 *
 * 태스크 액터가 자기 Tick에서 "일 주세요"라고 요청하는 방식은 쓰지 않는다. 그러면 누가 먼저
 * 요청하느냐가 **액터 틱 순서**에 달리는데, UE는 태스크 액터 A와 B 사이의 틱 순서를 보장하지 않는다
 * (`AddTickPrerequisiteActor(Robot)`은 로봇→태스크만 잡는다). 겹친 작업 영역의 박스를 누가 가져갈지가
 * 실행마다 달라지면, 고정 타임스텝으로 확보한 결정론이 무의미해진다 —
 * `APickPlaceTaskActor` 헤더가 이미 주장하고 있는 성질이다.
 *
 * 대신 dispatcher가 자기 고정 스텝마다 **정렬된 순서로** 순회하며 배급한다:
 * 정렬 키는 `RobotPriority` → 액터 `FName`(동률 tie-break)이고, 후보 선택의 동률은 박스 인덱스로
 * 깬다. 이 두 개가 재실행 시 같은 할당 순서를 보장한다.
 *
 * ## 도달 가능성 캐시
 *
 * "이 로봇이 이 박스를 집을 수 있는가"는 `APickPlaceTaskActor::CanReachGraspPointWorld()`
 * (= 사이클과 같은 `SolveForVisualGraspPoint`)로 판정한다. 정직하지만 **비싸다**: 내부가 고정점
 * 반복이고 반복마다 DLS가 최대 80회 돈다. 로봇 2대 × 박스 6개 × 2자세(박스/슬롯)면 DLS 288회가
 * 매 배급마다 터진다 — PIE 시작 직후, 즉 녹화가 시작되는 지점에서. 스톨이 나면 태스크 액터의
 * 시간 누적기가 튀어 `MaxFixedStepsPerFrame` 클램프가 걸리고, 그건 곧 결정론 경고다.
 *
 * 그래서 사이클 시작 시점에 (로봇 × 박스) / (로봇 × 슬롯) 도달성 행렬을 **한 번만** 계산해
 * 캐시하고, 이후 배급은 O(1) 조회로 한다. 박스는 집히기 전까지 움직이지 않으므로 캐시가 유효하다.
 * 스톨도 사이클 시작 전으로 밀려나 녹화에 잡히지 않는다.
 *
 * ## 범위 밖
 *
 * 팔 링크 간 정밀 충돌 검사와 경로계획은 하지 않는다. 로봇 링크 메시가 `NoCollision`이라 두 팔은
 * 서로 통과한다. 충돌 회피는 **도착지 슬롯 상호배제**(MinSlotSeparationCm)로 실제 위험 지점만
 * 겨냥하며, 나머지는 로봇을 작업 영역이 심하게 겹치지 않게 **배치**해서 회피한다.
 */
UCLASS()
class ROBOTSIM_ASSIGNMENT_API APickPlaceDispatcher : public AActor
{
	GENERATED_BODY()

public:
	APickPlaceDispatcher();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	#pragma region TaskActorAPI

	/**
	 * 태스크 액터가 자기 BeginPlay에서 등록한다. 등록 순서는 결정론에 영향을 주지 않는다 —
	 * 배급 직전에 RobotPriority/FName으로 다시 정렬하기 때문이다.
	 */
	void RegisterTaskActor(APickPlaceTaskActor* TaskActor);

	/**
	 * 배급된 작업을 완료 처리한다 (박스는 도착지에 놓였고 슬롯은 점유 상태로 남는다).
	 * 태스크 액터가 ToRetreat를 마칠 때 호출한다.
	 */
	void CompleteTask(APickPlaceTaskActor* TaskActor, const FPickPlaceTask& Task);

	/**
	 * 배급된 작업을 반납한다 — 중단(Aborted)/리셋/EndPlay에서 호출한다.
	 *
	 * **박스와 슬롯을 함께 반납한다.** 박스만 되돌리고 슬롯을 빠뜨리면 그 슬롯이 영구 점유로 새어
	 * 뒤쪽 박스들이 놓을 곳을 잃는다.
	 *
	 * @param bBlacklist true면 (로봇, 박스) 조합을 실패 목록에 올려 **같은 로봇이 다시 못 가져가게**
	 *        한다. 안 하면 즉시 재배급 → 또 abort → 재배급 무한 루프가 되어 화면에서 팔이 같은 자리를
	 *        영원히 버벅인다. 다른 로봇은 여전히 그 박스를 시도할 수 있다.
	 */
	void ReturnTask(APickPlaceTaskActor* TaskActor, const FPickPlaceTask& Task, bool bBlacklist, const FString& Reason);

	/** 도착지 슬롯의 월드 위치 (태스크 액터가 IK 목표를 만들 때 읽는다). */
	bool GetDestinationSlotWorld(int32 SlotIndex, FVector& OutWorld) const;

	/** dispatcher가 스폰한 박스 전량 (읽기 전용). */
	FORCEINLINE const TArray<TObjectPtr<APickPlaceBoxActor>>& GetBoxes() const { return Boxes; }

	#pragma endregion

protected:
	#pragma region Setup

	/** 스폰할 박스 클래스. 비우면 C++ 기본 APickPlaceBoxActor. */
	UPROPERTY(EditAnywhere, Category = "Dispatcher|Spawning")
	TSubclassOf<APickPlaceBoxActor> BoxClass;

	/** 스폰할 박스 개수 = 총 작업 수. */
	UPROPERTY(EditAnywhere, Category = "Dispatcher|Spawning", meta = (ClampMin = "1"))
	int32 NumBoxesToSpawn = 6;

	/** **출발지** — 박스가 이 액터의 상판 위에 스폰된다 (팔레트). 레벨에 배치한 액터를 할당한다. */
	UPROPERTY(EditAnywhere, Category = "Dispatcher|Layout")
	TObjectPtr<AActor> SourceSurfaceActor;

	/** 출발지 슬롯 간격 (cm, 출발지 액터 로컬 공간). */
	UPROPERTY(EditAnywhere, Category = "Dispatcher|Layout")
	FVector SourceSlotStrideCm = FVector(0.0, 80.0, 0.0);

	/** 출발지 슬롯 행 미세 조정 (cm, 출발지 액터 로컬). */
	UPROPERTY(EditAnywhere, Category = "Dispatcher|Layout")
	FVector SourceSlotOffsetCm = FVector::ZeroVector;

	/** **도착지** — 박스를 이 액터의 상판 위에 놓는다 (레일/컨베이어). */
	UPROPERTY(EditAnywhere, Category = "Dispatcher|Layout")
	TObjectPtr<AActor> DestinationSurfaceActor;

	/** 도착지 슬롯 간격 (cm, 도착지 액터 로컬 공간). */
	UPROPERTY(EditAnywhere, Category = "Dispatcher|Layout")
	FVector DestinationSlotStrideCm = FVector(0.0, 80.0, 0.0);

	/** 도착지 슬롯 행 미세 조정 (cm, 도착지 액터 로컬). */
	UPROPERTY(EditAnywhere, Category = "Dispatcher|Layout")
	FVector DestinationSlotOffsetCm = FVector::ZeroVector;

	#pragma endregion

	#pragma region Assignment

	/**
	 * 배급을 전진시키는 고정 타임스텝 (초). 태스크 액터와 **같은 값**을 쓸 것.
	 *
	 * 배급이 dispatcher 틱에서 일어나므로 시간 기준도 dispatcher가 갖는다 — Dispatch.csv의
	 * sim_time_sec은 이 누적기에서 나온다.
	 */
	UPROPERTY(EditAnywhere, Category = "Dispatcher|Assignment", meta = (ClampMin = "0.0005"))
	double FixedTimeStepSec = 1.0 / 120.0;

	/**
	 * 다른 로봇이 작업 중인 슬롯에서 이 거리 이내인 슬롯은 배급하지 않는다 (cm).
	 *
	 * @details
	 * 이것이 이 프로젝트의 **충돌 회피**다. 두 팔이 실제로 부딪칠 위험이 가장 큰 지점은 "둘 다 같은
	 * 레일 근처에 놓으러 가는 순간"이고, dispatcher가 이미 중재자이므로 그 지점만 싸게 막을 수 있다.
	 *
	 * **한계**: 팔 링크 간 정밀 충돌 검사가 아니다. 로봇 링크 메시는 `NoCollision`이라 두 팔이 같은
	 * 공간을 지나면 그냥 통과한다. 경로 중간의 간섭은 로봇을 작업 영역이 심하게 겹치지 않게
	 * **배치**해서 회피해야 한다. 0으로 두면 상호배제를 끈다.
	 */
	UPROPERTY(EditAnywhere, Category = "Dispatcher|Assignment", meta = (ClampMin = "0.0"))
	double MinSlotSeparationCm = 60.0;

	#pragma endregion

	#pragma region Logging

	/** 배급 이벤트 CSV 로깅 여부. 평가자에게 "할당이 실제로 일어났다"는 증거가 된다. */
	UPROPERTY(EditAnywhere, Category = "Dispatcher|Logging")
	bool bEnableDispatchCsv = true;

	/** 배급 CSV 파일명. 저장 경로는 <Project>/Saved/RobotSim/<DispatchCsvFileName>. */
	UPROPERTY(EditAnywhere, Category = "Dispatcher|Logging")
	FString DispatchCsvFileName = TEXT("Dispatch.csv");

	/**
	 * 슬롯 행/박스/표면 바운드를 뷰포트에 도달성 색으로 그린다 (배치 조정용).
	 *
	 * 로그의 좌표를 읽어 머리로 배치를 그리는 것보다 훨씬 빠르다 — 빨간 슬롯이 어느 쪽 끝에 몰렸는지가
	 * 곧 "행이 긴가/치우쳤나"의 답이다. 데모 녹화 시엔 끄면 된다.
	 *
	 * 색: 🔴 아무도 도달 불가 / 🟡 로봇 1대만 / 🟢 2대 이상(겹침 — 배급 경쟁과 슬롯 상호배제가 일어나는 구간)
	 */
	UPROPERTY(EditAnywhere, Category = "Dispatcher|Logging")
	bool bDrawDebugLayout = true;

	#pragma endregion

private:
	#pragma region State

	/** 등록된 태스크 액터. 배급 직전에 RobotPriority/FName으로 정렬한다. */
	UPROPERTY()
	TArray<TObjectPtr<APickPlaceTaskActor>> RegisteredTasks;

	/** 스폰한 박스 전량 (= 작업 풀). 인덱스가 곧 박스 ID다. */
	UPROPERTY()
	TArray<TObjectPtr<APickPlaceBoxActor>> Boxes;

	/** 도착지 슬롯 월드 좌표 (툴 목표 높이 = 상판 + 박스 높이). */
	TArray<FVector> DestinationSlotsWorld;

	/** 박스 i가 배급됐거나 완료됐는가 (재배급 방지). */
	TArray<bool> BoxTaken;

	/** 슬롯 j가 점유됐는가 (배급 중이거나 이미 박스가 놓임). */
	TArray<bool> SlotTaken;

	/** 슬롯 j를 현재 작업 중인 태스크 인덱스 (없으면 INDEX_NONE). 슬롯 상호배제 판정에 쓴다. */
	TArray<int32> SlotActiveTask;

	/** 도달성 캐시: [TaskIndex * Boxes.Num() + BoxIndex]. 사이클 시작 시 한 번 계산한다. */
	TArray<bool> BoxReachCache;

	/** 도달성 캐시: [TaskIndex * DestinationSlotsWorld.Num() + SlotIndex]. */
	TArray<bool> SlotReachCache;

	/** 실패 블랙리스트: [TaskIndex * Boxes.Num() + BoxIndex]. true면 그 로봇은 그 박스를 다시 안 가져간다. */
	TArray<bool> Blacklist;

	/** 배급 순서 정렬에 쓴 태스크 순서 (캐시 인덱스의 기준). 정렬 후 고정된다. */
	bool bInitialized = false;

	/** 첫 Tick에 초기화하기 위한 플래그 (BeginPlay 순서 문제 — Tick 주석 참조). */
	bool bPendingInit = false;

	/** 정지/완료 보고를 이미 했는지 (매 스텝 로그 도배 방지). 새 배급이 일어나면 다시 열린다. */
	bool bStallReported = false;

	/** dispatcher 고정 스텝 누적기. */
	double TimeAccumulatorSec = 0.0;

	/** 배급 기준 시뮬레이션 시간 (초). Dispatch.csv의 sim_time_sec. */
	double SimTimeSec = 0.0;

	/** Dispatch CSV 행 버퍼. */
	TArray<FString> DispatchCsvRows;

	#pragma endregion

	#pragma region Internal

	/** 박스 스폰 + 슬롯 풀 구성 + 도달성 캐시 계산. 첫 Tick에서 한 번만 실행한다. */
	void InitializeWorkPool();

	/**
	 * 등록된 태스크를 RobotPriority → FName 순으로 정렬한다.
	 *
	 * **정렬이 결정론의 근거다.** 배급 순회 순서가 실행마다 달라지면 겹친 영역의 박스를 누가
	 * 가져갈지가 매번 바뀐다. UE의 액터 틱 순서와 TActorIterator 순서는 둘 다 보장이 없으므로,
	 * 명시적 키로 매번 정렬한다.
	 */
	void SortTaskActorsDeterministically();

	/** 도달성 행렬을 한 번 계산한다 (로봇 × 박스, 로봇 × 슬롯). 비싸므로 사이클 시작 시점에만. */
	void BuildReachCaches();

	/** 유휴 로봇들에게 작업을 배급한다. dispatcher 고정 스텝마다 호출된다. */
	void AssignTasksToIdleRobots();

	/**
	 * 로봇이 전부 유휴인데 박스가 남아 있으면 — 즉 **작업이 미완인 채 정지했으면** — 이유를 한 번 설명한다.
	 *
	 * 이게 없으면 시스템이 조용히 멈춘다. 화면에는 로봇 두 대가 한 개씩 옮기고 서 있는 그림만 남고,
	 * 로그 어디에도 "왜 멈췄는지"가 없다. 실제로 슬롯이 병목이었는데 그걸 도달성 캐시 숫자에서
	 * 역산해야 했다 — 코드가 말했어야 하는 것이다.
	 *
	 * 정지는 실패다. 성공(전량 처리)과 구분해서 찍는다.
	 */
	void ReportStallOrCompletion();

	/** 태스크 t가 쓸 수 있는 가장 낮은 빈 슬롯 (도달 가능 + 상호배제 통과). 없으면 INDEX_NONE. */
	int32 FindFreeSlotFor(int32 TaskIndex) const;

	/** 슬롯 s가 다른 로봇의 활성 슬롯과 MinSlotSeparationCm 이내인가 (충돌 회피 상호배제). */
	bool IsSlotTooCloseToActive(int32 SlotIndex, int32 RequestingTaskIndex) const;

	/** 박스/슬롯 인덱스 조회 (로그와 CSV에 ID를 남기기 위해). */
	int32 IndexOfBox(const APickPlaceBoxActor* Box) const;

	/** 슬롯 행/박스/표면 바운드를 도달성 색으로 뷰포트에 그린다 (bDrawDebugLayout일 때만). */
	void DrawDebugLayout() const;

	/** 도달 로봇 수에 따른 색 (0=빨강, 1=노랑, 2+=초록). 슬롯/박스 공용. */
	static FColor ReachColor(int32 ReachableRobotCount);

	/** Dispatch CSV에 이벤트 한 줄 추가. */
	void RecordDispatchEvent(const TCHAR* Event, const APickPlaceTaskActor* TaskActor, int32 BoxIndex,
		int32 SlotIndex, const FString& Reason);

	/** Dispatch CSV를 디스크에 기록하고 버퍼를 비운다. */
	void WriteDispatchCsvToDisk();

	#pragma endregion
};
