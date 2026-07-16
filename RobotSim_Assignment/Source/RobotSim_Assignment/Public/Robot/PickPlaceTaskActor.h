// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Robot/RobotTypes.h"
#include "Templates/SubclassOf.h"
#include "PickPlaceTaskActor.generated.h"

class APickPlaceBoxActor;
class APickPlaceDispatcher;
class ASerial6DoFRobotActor;
class UStaticMesh;
class UStaticMeshComponent;

/**
 * @struct FPickPlaceTask
 * @brief 로봇 한 대에게 배급되는 작업 단위 — "이 박스를 저 슬롯에 놓아라".
 *
 * 박스와 슬롯을 **한 몸으로** 들고 다니는 것이 핵심이다. 중단 시 박스만 반납하고 슬롯을 빠뜨리면
 * 그 슬롯이 영구 점유 상태로 새어, 뒤쪽 박스들이 놓을 곳을 잃는다.
 *
 * 리플렉션이 필요 없는 순수 데이터라 USTRUCT로 만들지 않는다 (FRobot6DJointState 등과 같은 규약).
 * Box 포인터의 수명은 dispatcher의 UPROPERTY 배열이 보장한다.
 *
 * PickPlaceDispatcher.h가 아니라 여기 있는 이유: 태스크 액터의 AssignTask()가 이 타입을 값으로
 * 받고 dispatcher는 태스크 액터를 알아야 하므로, 반대로 두면 순환 include가 된다.
 */
struct FPickPlaceTask
{
	/** 집을 박스 (nullptr이면 미배급 상태). */
	APickPlaceBoxActor* Box = nullptr;

	/** 놓을 도착지 슬롯 인덱스 (dispatcher의 슬롯 풀 기준). */
	int32 DestinationSlotIndex = INDEX_NONE;

	/** 실제 작업이 담긴 배급인지. */
	bool IsValid() const { return Box != nullptr && DestinationSlotIndex != INDEX_NONE; }
};

/**
 * @struct FPickPlaceLayout
 * @brief 표면 위 슬롯 배치와 박스 스폰을 수행하는 순수 헬퍼.
 *
 * **dispatcher와 태스크 액터가 공유한다.** 태스크 액터의 멤버 함수로 두면 dispatcher가 같은 로직을
 * 복제해야 하고, 그러면 두 경로의 배치 규약이 조용히 갈라진다 — 슬롯 Z 계산 하나만 어긋나도
 * 박스가 상판을 뚫거나 뜬다. 멤버 상태에 의존하지 않는 순수 계산이므로 static으로 승격했다.
 */
struct FPickPlaceLayout
{
	/**
	 * 배치된 액터의 **상판 위**에 슬롯 SlotCount개를 자동 배치해 월드 좌표로 반환한다.
	 *
	 * - 행은 액터 바운드 중심에 정렬된다 → 개수를 바꿔도 액터를 다시 옮길 필요가 없다
	 * - StrideCm/OffsetCm은 **액터 로컬 공간**이라 액터를 회전시키면 행도 같이 돈다
	 * - 상판 높이는 **바운드**에서 읽으므로 메시 피벗 위치와 무관하다
	 * - Z는 상판 기준으로 덮어쓴다 (액터가 기울어져 있어도 중력 기준으로 쌓아야 하므로)
	 *
	 * @param HeightAboveSurfaceCm 상판에서 띄울 높이 (박스 스폰=0, 툴 목표=박스 높이)
	 * @return 유효한 상판을 얻어 슬롯을 만들었으면 true
	 */
	static bool BuildSlotsOnSurface(
		const AActor* SurfaceActor, int32 SlotCount, const FVector& StrideCm, const FVector& OffsetCm,
		double HeightAboveSurfaceCm, TArray<FVector>& OutSlotWorld);

	/**
	 * 슬롯 월드 좌표마다 박스를 하나씩 스폰하고, 각 박스의 **바운드 바닥면**을 그 슬롯 Z에 맞춘다.
	 *
	 * 액터 위치가 아니라 바운드로 앉히는 이유: 프롭 메시는 피벗이 바닥/구석에 있는 경우가 흔해
	 * "액터 위치 = 기하학적 중심"이 성립하지 않는다. 뜬 채로 스폰되면 낙하 중에 파지 지점이
	 * 스냅샷돼 엉뚱한 곳을 집고, 옆 박스를 밀어내기도 한다.
	 */
	static void SpawnBoxesOnSlots(
		UWorld* World, UClass* BoxClass, const TArray<FVector>& SlotWorld, const FRotator& SpawnRotation,
		AActor* Owner, TArray<APickPlaceBoxActor*>& OutBoxes);
};

/**
 * @enum EPickPlacePhase
 * @brief pick&place 한 사이클의 FSM 단계.
 *
 * 전이는 선형이며 분기는 단 하나다 — ToRetreat 끝에서 남은 박스가 있으면 ToPickApproach로 돌아가고,
 * 없으면 Done으로 끝난다. 각 단계는 "관절공간 궤적 추종"(To*)이거나 "정지 대기"(Grasp/Release)
 * 둘 중 하나이며, 이 두 종류만으로 사이클 전체가 표현된다.
 */
UENUM(BlueprintType)
enum class EPickPlacePhase : uint8
{
	/** 시작 전. StartCycle()이 호출되기 전이거나 ResetCycle() 직후. */
	Idle,

	/** 집을 박스 바로 위(ApproachOffsetCm)까지 이동. 박스 옆면 충돌 없이 수직 진입하기 위한 준비 자세. */
	ToPickApproach,

	/** 접근 자세에서 박스 파지 위치까지 수직 하강. */
	ToPick,

	/** 정지 상태로 DwellSec 대기 (실제 그리퍼 닫힘 시간의 대역). 진입 시 박스를 툴에 부착한다. */
	Grasp,

	/** 박스를 든 채로 다시 접근 자세(박스 위)까지 수직 상승. */
	ToLift,

	/** 팔레트 적재 슬롯 바로 위까지 이동. 사이클에서 가장 큰 관절 이동이 일어나는 구간. */
	ToPlaceApproach,

	/** 적재 슬롯까지 수직 하강. */
	ToPlace,

	/** 정지 상태로 DwellSec 대기 (그리퍼 열림 시간). 진입 시 박스를 놓고 물리를 켠다. */
	Release,

	/** 놓은 박스 위로 수직 상승해 다음 사이클/종료 자세로 빠진다. */
	ToRetreat,

	/** 모든 박스 처리 완료. CSV가 있으면 이 시점에 기록된다. */
	Done,

	/**
	 * 도달 불가 목표 등으로 사이클을 중단했다.
	 *
	 * Done과 마찬가지로 관절 상태를 계속 로봇에 밀어넣지만(멈추면 팔이 홈 자세로 튕긴다) 성공과는
	 * 구분해 로그에 남긴다. 중단 없이 "최선해로 진행"하면 툴이 목표에서 1m 떨어진 채 박스를 든 것처럼
	 * 행동해 원인이 눈에 보이지 않는다 — 실패는 조용히 진행하는 것보다 멈추고 말하는 편이 낫다.
	 */
	Aborted
};

/**
 * @class APickPlaceBoxActor
 * @brief pick&place로 옮길 박스 하나.
 *
 * 파지 중에는 물리를 끄고(SetSimulatePhysics(false)) 툴 팁의 자식처럼 따라다니고, 놓을 때 물리를 켜서
 * 이후 거동을 Chaos에 넘긴다. "파지 중 kinematic, 해제 후 dynamic"은 STEP C 수직 슬라이스의 의도적 단순화다 —
 * 마찰 원뿔 기반 grasp 판정(STEP B 범위)이 들어오면 이 액터의 물리 토글을 그 판정 결과로 대체하게 된다.
 *
 * 로봇 수학(FSerial6DoFModel/FRobotDlsIK)에는 어떤 영향도 주지 않는다. 박스는 IK의 목표 위치를
 * 제공하는 입력일 뿐이고, 반대로 로봇 상태가 박스를 끌고 다니는 단방향 의존이다.
 *
 * BP로 서브클래싱해 메시/머티리얼/크기를 바꿔도 된다 — APickPlaceTaskActor는 BoxClass로 그 BP를
 * 참조해 스폰하며, 파지 높이는 실제 바운드에서 읽으므로 크기를 바꿔도 코드 수정이 필요 없다.
 */
UCLASS()
class ROBOTSIM_ASSIGNMENT_API APickPlaceBoxActor : public AActor
{
	GENERATED_BODY()

public:
	APickPlaceBoxActor();

	virtual void BeginPlay() override;

	#pragma region BoxAPI

	/**
	 * 파지 시작: 물리 시뮬레이션을 끄고 툴 팁 추종 모드로 전환한다.
	 * 실제 추종(트랜스폼 갱신)은 APickPlaceTaskActor가 매 스텝 수행한다 — 박스는 상태만 바꾼다.
	 */
	void BeginGrasp();

	/**
	 * 파지 해제: 물리 시뮬레이션을 켜서 Chaos에 넘긴다.
	 * 놓는 순간의 속도는 0으로 두어(툴이 Release 단계에서 정지 상태이므로) 박스가 튀지 않게 한다.
	 */
	void EndGrasp();

	/** 현재 파지 중인지 (= 물리가 꺼진 추종 상태인지). */
	FORCEINLINE bool IsGrasped() const { return bGrasped; }

	/** 박스 질량 (kg). BeginPlay에서 메시 컴포넌트에 override mass로 적용한다. */
	FORCEINLINE double GetMassKg() const { return MassKg; }

	/**
	 * 파지 기준점 (월드) = **박스 윗면 중심**.
	 *
	 * 실제 팔레타이징 그리퍼(진공/클램프)는 박스 윗면에 접촉하므로 여기가 툴 팁이 가야 할 곳이다.
	 * 박스 중심을 잡으면 그리퍼가 박스 속에 박힌 것처럼 보인다.
	 *
	 * GetActorLocation()이 아니라 **바운드**에서 계산한다 — 프롭 메시는 피벗이 바닥/구석에 있는 경우가
	 * 흔해서 액터 위치가 기하학적 중심이라는 보장이 없기 때문이다. 바운드 기준이면 BP에서 어떤 메시를
	 * 물려도(피벗이 어디든) 그대로 동작한다.
	 */
	FVector GetGraspPointWorld() const;

	/** 박스 전체 높이 (cm, 월드 스케일 반영). 윗면을 잡으므로 적재 시 툴 목표 = 지지면 + 이 값이다. */
	double GetHeightCm() const;

	/** 박스 바운드 절반 크기 (cm, 월드 AABB). 슬롯 간격이 박스 폭보다 좁은지 검사하는 데 쓴다. */
	FVector GetBoundsExtentCm() const;

	/** 바운드 바닥면의 월드 Z. "지지면 위에 정확히 앉히기"를 피벗 위치와 무관하게 계산하는 데 쓴다. */
	double GetBoundsBottomZWorld() const;

	#pragma endregion

protected:
	/** 박스 본체. 루트이자 물리 바디이며, 이 컴포넌트의 트랜스폼이 곧 박스 자세다. */
	UPROPERTY(VisibleAnywhere, Category = "Box")
	TObjectPtr<UStaticMeshComponent> BoxMesh;

	/**
	 * 박스 질량 (kg). Chaos에 override mass로 전달한다.
	 *
	 * 단위 주의: 동역학 값이므로 SI(kg)다 — 길이(cm)와 단위계가 다르다는 STEP B 규약을 따른다.
	 * STEP B의 마찰 grasp 판정과 RNEA payload 항이 들어오면 같은 값을 그쪽에서도 읽게 된다.
	 */
	UPROPERTY(EditAnywhere, Category = "Box", meta = (ClampMin = "0.001"))
	double MassKg = 2.0;

private:
	/** 현재 파지 상태 (물리 off + 툴 추종). */
	bool bGrasped = false;
};

/**
 * @class APickPlaceTaskActor
 * @brief 박스를 집어 팔레트에 놓는 pick&place 사이클을 구동하는 FSM 액터.
 *
 * STEP C 수직 슬라이스의 진입점이다. 로봇에 대해 **읽기 + SetJointAngles만** 수행하며
 * ASerial6DoFRobotActor를 일절 수정하지 않는다. 이 액터가 없으면 STEP A 동작은 완전히 그대로다 —
 * 즉 STEP A 회귀 위험이 구조적으로 0이다.
 *
 * @details
 * **시나리오**: 팔레트(SourceSurfaceActor) 위에 박스가 스폰되고, 로봇이 하나씩 집어 레일/컨베이어
 * (DestinationSurfaceActor) 위에 갖다 놓는다 — 디팔레타이징이며, KUKA R3200 **PA(Palletizing)**의
 * 실제 용도다.
 *
 * **좌표 규약 — 출발지/도착지 위치는 계산하지 않고 배치된 액터에서 읽는다.**
 * 두 지점 모두 사용자가 뷰포트에서 정하는 의도이지 계산 대상이 아니다. 좌표로 계산하려 들면 메시 피벗,
 * 지지면 트레이스, 로봇 로컬 변환이 곱해져 어디로 갈지 예측이 안 된다(그렇게 만들었다가 팔레트가 엉뚱한
 * 곳에 스폰됐다). 배치된 액터의 트랜스폼+바운드를 읽는 쪽으로 통일하니 지지면 트레이스와 로봇 로컬 좌표
 * 추측이 통째로 사라졌다 — BuildSlotsOnSurface() 하나가 양쪽을 다 처리한다.
 *
 * 나머지 배치 프로퍼티(GraspRotation / ApproachOffsetCm)는 **로봇 액터 공간** 기준이다. 월드가 아니다.
 * 로봇 모델(FSerial6DoFModel)이 BaseTransform=Identity인 액터 로컬 공간에서
 * FK를 계산하므로, 배치값도 같은 공간에 두면 IK 목표를 좌표 변환 없이 그대로 넘길 수 있고,
 * 무엇보다 **로봇을 레벨 어디로 옮기든 기본값이 그대로 유효하다.** 월드 좌표로 두면 로봇이
 * 원점에서 벗어나는 순간 모든 기본값이 무의미해진다(실제로 그렇게 만들었다가 팔레트가 1000cm 밖으로
 * 나가 IK가 976cm 오차로 실패했다). 박스 위치만 월드에서 읽어 로봇 공간으로 변환한다.
 *
 * 제어 구조:
 * - 목표 자세(로봇 공간) → FRobotDlsIK::SolveDlsIK로 단계 진입 시 **한 번** 푼다.
 * - 그 해와 시작 자세 사이를 quintic smoothstep으로 관절공간 보간한다. IK를 매 프레임 돌리지 않으므로
 *   궤적이 결정론적이고, singularity 근처에서 프레임마다 해가 튀는 문제도 없다.
 * - 관절 상태의 source of truth는 이 액터의 ActiveState다. 매 프레임 Robot->SetJointAngles로 밀어넣는다.
 * - IK가 목표에 MaxReachErrorCm보다 멀리서 멈추면 사이클을 Aborted로 중단한다.
 *
 * 고정 타임스텝:
 * DeltaSeconds를 누적해 FixedTimeStepSec 단위로만 FSM을 전진시킨다. 프레임레이트가 흔들려도 궤적과
 * CSV 샘플 간격이 동일하게 나오며, 이것이 필수 항목인 "고정 타임스텝 + 물리값 CSV"의 전제다.
 *
 * 틱 순서 (중요):
 * ASerial6DoFRobotActor::Tick은 매 틱 에디터 프로퍼티(JointAnglesDeg)를 관절 상태에 되쓴다.
 * 따라서 이 액터는 반드시 로봇 **뒤에** 틱해야 하며, BeginPlay에서 AddTickPrerequisiteActor(Robot)로
 * 그 순서를 강제한다. 로봇을 수정하지 않고 되쓰기를 이기는 유일한 방법이다.
 *
 * 범위 밖(의도적 미구현): RNEA 역동역학, 순동역학, 시간최적 S-curve, computed torque 제어,
 * 마찰 원뿔 grasp 판정, N층 파라메트릭 적재, 충돌 회피 경로계획, 모션 JSON 기록/리플레이, UMG 위젯.
 */
UCLASS()
class ROBOTSIM_ASSIGNMENT_API APickPlaceTaskActor : public AActor
{
	GENERATED_BODY()

public:
	APickPlaceTaskActor();

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void Tick(float DeltaSeconds) override;

	#pragma region TaskAPI

	/** 현재 FSM 단계. */
	FORCEINLINE EPickPlacePhase GetPhase() const { return Phase; }

	/** 현재 처리 중인 박스 인덱스 (Boxes 배열 기준, 사이클 종료 후에는 범위를 벗어날 수 있다). */
	FORCEINLINE int32 GetCurrentBoxIndex() const { return CurrentBoxIndex; }

	/** 사이클 시작 이후 누적된 시뮬레이션 시간 (초, 고정 타임스텝의 정수배). */
	FORCEINLINE double GetSimTimeSec() const { return SimTimeSec; }

	#pragma endregion

	#pragma region DispatcherAPI

	/**
	 * 지금 새 작업을 받을 수 있는가 (초기화 완료 + 배급 대기 중 + 진행 중인 작업 없음).
	 *
	 * `bCycleStarted` 조건이 중요하다: dispatcher는 태스크 액터보다 **먼저** 틱하므로(prerequisite),
	 * 첫 프레임에는 태스크 액터의 StartCycle이 아직 안 돌았다. 그때 배급하면 곧이어 실행되는
	 * StartCycle이 Phase를 Idle로 되돌려 배급이 조용히 증발한다.
	 */
	bool IsIdle() const;

	/** dispatcher가 작업을 배급한다. 즉시 ToPickApproach로 진입한다. */
	void AssignTask(const FPickPlaceTask& InTask);

	/** 현재 배급된 작업 (미배급이면 Box == nullptr). */
	FORCEINLINE const FPickPlaceTask& GetAssignedTask() const { return AssignedTask; }

	/** 구동 중인 로봇 (dispatcher가 베이스 위치/거리 계산에 쓴다). */
	FORCEINLINE ASerial6DoFRobotActor* GetRobot() const { return Robot; }

	/** 배급 순서 결정론 키 (작을수록 먼저). dispatcher가 이 값으로 정렬한다. */
	FORCEINLINE int32 GetRobotPriority() const { return RobotPriority; }

	/**
	 * 이 로봇의 **시각 파지점**이 해당 월드 지점에 도달 가능한가.
	 *
	 * "이 로봇이 이 일을 할 수 있는가"에 대한 정직한 술어다 — 사이클의 도달 판정과 **같은 함수**
	 * (SolveForVisualGraspPoint)를 쓰므로 R3200의 최소 반경(dead zone)까지 반영한다. 별도의 근사
	 * 판정을 두면 "배급은 됐는데 실행하면 Aborted"가 되어 판정 두 벌이 어긋난다.
	 *
	 * 관절 상태를 잠시 움직이지만 반드시 복원한다 — 판정만으로 팔이 튀면 안 된다.
	 *
	 * **접근 자세까지 함께 검사한다.** 사이클은 목표 지점만 가는 게 아니라 ApproachOffsetCm만큼
	 * 위에서 수직 진입/이탈하며, 팔이 위로 뻗을수록 도달 반경이 줄어 **접근 자세가 더 빡빡하다.**
	 * 목표 지점만 보고 배급하면 접근 단계에서 Aborted가 나고 배급→반납이 반복된다 (실제로 그랬다).
	 *
	 * **비싸다**: 내부가 고정점 반복이고 반복마다 DLS(최대 80회)가 돈다. 매 배급마다 부르지 말고
	 * dispatcher가 사이클 시작 시점에 한 번 캐시할 것.
	 */
	bool CanReachGraspPointWorld(const FVector& GraspPointWorld);

	#pragma endregion

	#pragma region EditorUtility

	/**
	 * 사이클을 처음부터 시작한다. 현재 로봇 관절 상태를 시작 자세로 삼고 첫 박스의 ToPickApproach로 진입한다.
	 * 이미 진행 중이어도 무조건 처음부터 다시 시작한다 (데모 중 재실행 편의).
	 */
	UFUNCTION(CallInEditor, BlueprintCallable, Category = "PickPlace")
	void StartCycle();

	/**
	 * 사이클을 Idle로 되돌린다. 파지 중인 박스가 있으면 물리를 켜서 놓는다.
	 *
	 * 이 액터는 Idle에서 SetJointAngles를 멈추므로, 로봇은 다음 틱부터 자기 JointAnglesDeg로 되돌아간다
	 * (= 팔이 디테일 패널의 각도 자세로 스냅한다). 관절 소유권을 STEP A 쪽에 되돌려주는 것이 의도다.
	 */
	UFUNCTION(CallInEditor, BlueprintCallable, Category = "PickPlace")
	void ResetCycle();

	/** 누적된 CSV 샘플을 지금 즉시 파일로 기록한다 (사이클이 끝나기 전에 확인하고 싶을 때). */
	UFUNCTION(CallInEditor, BlueprintCallable, Category = "PickPlace")
	void FlushCsvNow();

	/**
	 * 로봇의 SkeletalMesh 레퍼런스 스켈레톤(바인드 포즈) 뼈 치수를 로그로 덤프한다 (읽기 전용 진단).
	 *
	 * **용도**: 수학 모델(FSerial6DoFModel.LinkOffsets)을 실제 메시 치수에 맞춰 authoring하기 위한 실측값.
	 * 수학 팔과 시각 메시의 크기가 다르면 두 EE는 홈 자세에서만 일치하고 관절이 돌자마자 갈라진다 —
	 * ToolOffset 캘리브레이션은 그 불일치를 홈 자세에서만 상쇄하는 미봉책이라 파지가 어긋난다.
	 *
	 * 각 뼈의 **부모 상대 위치**가 곧 LinkOffsets 후보다(뼈 체인 순서 = J0→J5 가정). 이 값을
	 * DA_RobotConfig에 옮겨 적으면 수학 팔이 메시와 같은 크기가 된다.
	 *
	 * 로봇의 public API(GetComponentByClass)만 사용하므로 ASerial6DoFRobotActor를 수정하지 않는다.
	 * 관절/박스/모델에 어떤 영향도 주지 않는 순수 조회다.
	 */
	UFUNCTION(CallInEditor, BlueprintCallable, Category = "PickPlace")
	void LogRobotSkeletonGeometry();

	//~ 모션 CSV 재생 (D-02) — RecordCsvRow/WriteCsvToDisk의 **역방향**. 저장한 관절 궤적을
	//~ 고정 타임스텝으로 되재생해 "결정론적 재현"을 눈으로 증명한다.

	/**
	 * CSV를 파싱해 ReplayFrames에 관절 궤적을 채운다 (재생은 하지 않는다).
	 *
	 * 로드와 재생을 분리한 이유: 같은 파일을 여러 번 재생할 때 매번 파싱하지 않기 위해서다.
	 * 컬럼은 **0행 헤더를 파싱해 이름(q0_deg~q5_deg)으로 찾는다** — 고정 인덱스는 컬럼 순서가
	 * 한 번만 바뀌어도 조용히 틀린 열을 읽는다. 기록↔재생 대칭의 계약은 컬럼 이름이지 위치가 아니다.
	 */
	UFUNCTION(CallInEditor, BlueprintCallable, Category = "PickPlace")
	void LoadMotionCsv();

	/**
	 * <Project>/Saved/RobotSim/ 폴더의 *.csv 파일명 목록을 반환한다 (UI 드롭다운용).
	 *
	 * 파일 시스템을 훑으므로 BlueprintPure가 아니라 BlueprintCallable이다 — 매 프레임 폴링하면 안 되고,
	 * 드롭다운을 열 때(또는 사이클 종료 후 갱신)만 부른다.
	 */
	UFUNCTION(CallInEditor, BlueprintCallable, Category = "PickPlace")
	TArray<FString> GetAvailableMotionCsvFiles() const;

	/**
	 * 재생할 파일명을 지정한다 (드롭다운 OnSelectionChanged → 여기 → PlayReplay 순서).
	 * ReplayMotionFileName 프로퍼티를 BP에서 쓰기 위한 setter다. 다음 LoadMotionCsv/PlayReplay부터 적용된다.
	 */
	UFUNCTION(BlueprintCallable, Category = "PickPlace")
	void SetReplayMotionFileName(const FString& InFileName);

	/** 로드한 궤적을 처음부터 재생한다. ReplayFrames가 비어 있으면 LoadMotionCsv를 한 번 자동 호출한다. */
	UFUNCTION(CallInEditor, BlueprintCallable, Category = "PickPlace")
	void PlayReplay();

	/** 재생을 중단하고 Idle로 돌아간다. */
	UFUNCTION(CallInEditor, BlueprintCallable, Category = "PickPlace")
	void StopReplay();

	/**
	 * 누적 CSV 파일을 삭제해 처음부터 다시 쌓게 한다 (bAccumulateMotionAcrossRuns용).
	 * 로드해둔 재생 프레임과 과거 누적 행도 함께 비운다.
	 */
	UFUNCTION(CallInEditor, BlueprintCallable, Category = "PickPlace")
	void ClearMotionCsv();

	#pragma endregion

	#pragma region UIBinding

	//~ UMG 바인딩 표면 (STEP D-01). Widget Blueprint가 이 액터를 ViewModel처럼 읽는다.
	//~
	//~ **모든 BlueprintPure getter는 캐시/기존 상태만 반환한다 — RNEA도 IK도 절대 돌리지 않는다.**
	//~ BlueprintPure는 바인딩된 위젯 수 × 프레임만큼 호출되므로(같은 프레임에 여러 번), getter가
	//~ 계산을 하면 프레임이 무너진다. 토크는 StepFixed가 한 번 계산해 캐시한 값을 읽을 뿐이다.
	//~
	//~ **선택 상태(어느 로봇을 보고 있는가)는 여기 없다.** UI 정책이 시뮬레이션 액터로 새면 안 된다 —
	//~ dispatcher가 GetTaskActors()로 목록만 주고 패널 생성/선택은 위젯이 한다.
	//~
	//~ double이 아니라 float/TArray를 반환하는 이유: Blueprint는 double과 고정 배열을 다루기 불편하다.
	//~ 단위 변환(rad→deg)과 정밀도 축소는 **UI 경계에서만** 일어나며 내부 상태는 그대로 double이다.

	/** 로봇 표시 이름 (로봇이 없으면 이 액터 이름). 멀티로봇 패널의 제목. */
	UFUNCTION(BlueprintPure, Category = "PickPlace|UI")
	FText GetRobotDisplayText() const;

	/** 현재 FSM 단계 이름. 로그/CSV와 **같은 문자열**을 쓴다 — 화면과 로그가 갈라지면 대조가 안 된다. */
	UFUNCTION(BlueprintPure, Category = "PickPlace|UI")
	FText GetPhaseDisplayText() const;

	/** 현재 단계의 진행률 0~1 (ProgressBar용). 대기 시간이 0인 단계는 0을 반환한다. */
	UFUNCTION(BlueprintPure, Category = "PickPlace|UI")
	float GetPhaseProgress() const;

	/** 관절각 6개 (도). 이 액터가 소유한 ActiveState 기준 — 로봇 컴포넌트를 읽지 않는다. */
	UFUNCTION(BlueprintPure, Category = "PickPlace|UI")
	TArray<float> GetJointAnglesDeg() const;

	/** 관절 각속도 6개 (도/초). 고정 스텝 유한차분 캐시. CSV의 qd 컬럼과 **같은 값**이다. */
	UFUNCTION(BlueprintPure, Category = "PickPlace|UI")
	TArray<float> GetJointVelocityDegPerSec() const;

	/**
	 * 관절 토크 6개 (N·m). B-02 RNEA의 결과이며 CSV의 tau 컬럼과 **같은 캐시**를 읽는다.
	 * **qdd=0 준정적 추정이다** — 중력/마찰 항만 살아 있고 관성·코리올리 항은 빠져 있다.
	 * 이 토크는 표시용이며 실제 구동에 관여하지 않는다 (구동은 여전히 SetJointAngles 기구학이다).
	 */
	UFUNCTION(BlueprintPure, Category = "PickPlace|UI")
	TArray<float> GetJointTorqueNm() const;

	/** 관절별 토크 사용률 |τ_i| / MaxTorqueNm_i, 0~1 clamp (게이지 바용). 한계가 0 이하면 0. */
	UFUNCTION(BlueprintPure, Category = "PickPlace|UI")
	TArray<float> GetJointTorqueRatio() const;

	/** 현재 툴 팁 위치 (월드, cm). 수학 FK 결과이며 시각 그리퍼 위치가 아니다. */
	UFUNCTION(BlueprintPure, Category = "PickPlace|UI")
	FVector GetToolLocationWorld() const;

	/** 마지막 IK가 남긴 시각 파지점 오차 (cm). MaxReachErrorCm를 넘으면 사이클이 중단된 것이다. */
	UFUNCTION(BlueprintPure, Category = "PickPlace|UI")
	float GetLastReachErrorCm() const;

	/** 이 로봇이 완료한 박스 수 (사이클 시작 이후 누적). */
	UFUNCTION(BlueprintPure, Category = "PickPlace|UI")
	int32 GetCompletedBoxCount() const;

	/** 프레임 시간 (ms). **EMA 평활값이다** — 원시값은 진동이 심해 HUD에서 숫자를 읽을 수 없다. */
	UFUNCTION(BlueprintPure, Category = "PickPlace|UI")
	float GetFrameTimeMs() const;

	/**
	 * 지금 CSV에 궤적을 기록하는 중인가 (UI의 🔴 REC 표시등용). IsReplaying과 대칭이다.
	 *
	 * "기록 중"의 정의: 로깅이 켜져 있고(bEnableCsvLogging), 사이클이 시작됐으며(bCycleStarted),
	 * 재생 중이 아니다. 재생은 관절을 되재생만 하지 새 샘플을 쌓지 않으므로 기록이 아니다.
	 * 별도의 record 버튼은 없다 — StartCycle이 곧 기록 시작이고, 저장은 사이클 종료 시 자동이다.
	 * 이 getter는 그 "자동으로 켜진 기록 상태"를 화면에 드러내기 위한 것이다.
	 */
	UFUNCTION(BlueprintPure, Category = "PickPlace|UI")
	bool IsRecording() const;

	/** 지금 모션 CSV를 재생 중인가 (D-01의 IsReplaying/GetReplayProgress 표면과 대칭). */
	UFUNCTION(BlueprintPure, Category = "PickPlace|UI")
	bool IsReplaying() const;

	/** 재생 진행률 0~1 (ProgressBar용). 재생 중이 아니면 0. */
	UFUNCTION(BlueprintPure, Category = "PickPlace|UI")
	float GetReplayProgress() const;

	/** 사이클이 실제로 전진 중인가 (Idle/Done/Aborted가 아니고 시작됐는가). */
	UFUNCTION(BlueprintPure, Category = "PickPlace|UI")
	bool IsCycleRunning() const;

	/** 일시정지 상태인가. */
	UFUNCTION(BlueprintPure, Category = "PickPlace|UI")
	bool IsPaused() const;

	/**
	 * 일시정지 설정. FSM 고정 스텝 전진과 CSV 행 기록을 멈춘다.
	 *
	 * 관절 상태 밀어넣기(SetJointAngles)와 박스 추종은 **멈추지 않는다** — 멈추면 로봇 Tick의
	 * JointAnglesDeg 되쓰기가 이겨서 팔이 홈 자세로 튕긴다. 잡고 있던 박스가 현재 자세에 그대로
	 * 붙어 있는 것도 이 때문이다.
	 */
	UFUNCTION(BlueprintCallable, Category = "PickPlace")
	void SetPaused(bool bInPaused);

	/** 속도 프로파일 튜닝: 관절 속도 한계 대비 사용 비율 (0.01~1). */
	UFUNCTION(BlueprintPure, Category = "PickPlace|Tuning")
	float GetVelocityScale() const;

	/** 속도 프로파일 튜닝. 다음 **단계 진입**부터 반영된다 — 진행 중인 궤적의 소요시간은 이미 확정됐다. */
	UFUNCTION(BlueprintCallable, Category = "PickPlace|Tuning")
	void SetVelocityScale(float NewVelocityScale);

	/** Grasp/Release 대기 시간 (초). */
	UFUNCTION(BlueprintPure, Category = "PickPlace|Tuning")
	float GetDwellSec() const;

	/** Grasp/Release 대기 시간 설정 (0~5초로 clamp). */
	UFUNCTION(BlueprintCallable, Category = "PickPlace|Tuning")
	void SetDwellSec(float NewDwellSec);

	/**
	 * 모션 저장: 누적된 CSV 샘플을 지금 즉시 파일로 기록한다 (FlushCsvNow와 동일 동작).
	 * PDF의 "모션 저장" 슬롯이다. **로드는 없다** — trajectory 로드 시스템이 없으므로 만들지 않았다.
	 */
	UFUNCTION(BlueprintCallable, Category = "PickPlace")
	void SaveMotionCsvNow();

	#pragma endregion

protected:
	#pragma region Setup

	/**
	 * 구동할 로봇. 이 액터는 로봇의 public API(SetJointAngles/GetModel/GetJointState/GetEndEffectorPose)만
	 * 사용하며 로봇 클래스를 수정하지 않는다. 모든 배치 프로퍼티가 이 로봇의 액터 공간 기준이다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Setup")
	TObjectPtr<ASerial6DoFRobotActor> Robot;

	/**
	 * 처리할 박스들. 배열 순서대로 하나씩 집어 놓는다. 비어 있는(None) 슬롯은 건너뛴다.
	 *
	 * **standalone 전용** — Dispatcher가 물려 있으면 박스는 dispatcher가 소유하며 이 배열은 쓰이지 않는다.
	 * bSpawnBoxesOnBeginPlay가 켜져 있으면 BeginPlay에서 **스폰된 박스로 덮어쓴다** — 이때는 레벨에
	 * 박스를 직접 배치할 필요가 없다. 끄면 여기에 수동 배치한 박스를 할당해 쓴다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Setup",
		meta = (EditCondition = "Dispatcher == nullptr", EditConditionHides))
	TArray<TObjectPtr<APickPlaceBoxActor>> Boxes;

	/** BeginPlay에서 자동으로 사이클을 시작할지 여부. 끄면 StartCycle()을 직접 호출해야 한다. */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Setup")
	bool bAutoStartOnBeginPlay = true;

	/**
	 * 작업을 배급받을 dispatcher. **비워 두면 standalone 모드**로 지금까지와 완전히 동일하게 동작한다
	 * (자기 Boxes 배열, 자기 스폰, 자기 슬롯).
	 *
	 * @details
	 * dispatcher 모드에서는 박스/슬롯의 소유권이 dispatcher로 넘어간다 — 이 액터는 배급된 작업
	 * 하나(FPickPlaceTask)를 수행하고 Idle로 돌아가 다음 배급을 기다린다. FSM 9단계 자체는 두 모드가
	 * 완전히 동일하다.
	 *
	 * standalone 폴백을 남겨둔 이유는 보험이다: dispatcher 배선이 틀려도 단일 로봇 데모가 그대로
	 * 살아 있어야 한다. dispatcher 경로는 **추가지 대체가 아니다.**
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Setup")
	TObjectPtr<APickPlaceDispatcher> Dispatcher;

	/**
	 * 배급 순서를 정하는 우선순위 (작을수록 먼저 배급받는다). 로봇마다 **서로 다른 값**을 줄 것.
	 *
	 * @details
	 * 이 값이 멀티로봇 결정론의 근거다. dispatcher가 배급 대상을 순회하는 순서가 실행마다 달라지면
	 * 겹친 작업 영역의 박스를 누가 가져갈지가 매번 바뀌어, 고정 타임스텝으로 확보한 결정론이 무의미해진다.
	 * UE는 액터 간 틱 순서를 보장하지 않으므로(AddTickPrerequisiteActor는 로봇→태스크만 잡는다)
	 * "먼저 요청한 로봇이 가져간다" 방식은 쓸 수 없다. 명시적 우선순위로 순서를 고정한다.
	 *
	 * 같은 값이면 액터 이름(FName)으로 tie-break하므로 여전히 결정론적이지만, 의도를 드러내려면
	 * 직접 다르게 주는 편이 낫다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Setup")
	int32 RobotPriority = 0;

	#pragma endregion

	#pragma region Spawning

	//~ 이 섹션은 **standalone 모드 전용**이다. Dispatcher가 물려 있으면 박스/슬롯 소유권이 그쪽으로
	//~ 넘어가 여기 값들은 읽히지 않으므로, EditConditionHides로 패널에서 숨긴다.
	//~ (같은 이름의 프로퍼티가 dispatcher에도 있어 둘 다 보이면 어느 쪽이 실제로 쓰이는지 헷갈린다 —
	//~  실제로 "Num Boxes To Spawn을 어디서 바꾸나"에서 혼선이 났다.)
	//~ 코드 경로 자체는 그대로 살아 있다: Dispatcher를 비우면 C-01과 동일하게 동작하는 보험이다.

	/**
	 * BeginPlay에서 박스를 자동 스폰할지 여부.
	 *
	 * 켜두는 것을 권장한다. 레벨에 손으로 배치하면 로봇 작업 반경 밖에 두기 쉽고(그러면 IK가 실패한다)
	 * 로봇을 옮길 때마다 다시 배치해야 하지만, 스폰은 좌표가 로봇 기준이라 항상 반경 안에 들어온다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Spawning",
		meta = (EditCondition = "Dispatcher == nullptr", EditConditionHides))
	bool bSpawnBoxesOnBeginPlay = true;

	/**
	 * 스폰할 박스 클래스. **BP 서브클래스를 지정하면 메시/머티리얼/크기를 BP에서 자유롭게 꾸밀 수 있다.**
	 * 비어 있으면 C++ 기본 APickPlaceBoxActor(15cm 회색 큐브)를 쓴다.
	 * 파지 높이는 실제 바운드에서 읽으므로 BP에서 크기를 바꿔도 코드는 그대로 동작한다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Spawning",
		meta = (EditCondition = "Dispatcher == nullptr && bSpawnBoxesOnBeginPlay", EditConditionHides))
	TSubclassOf<APickPlaceBoxActor> BoxClass;

	/** 스폰할 박스 개수. */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Spawning",
		meta = (ClampMin = "1", EditCondition = "Dispatcher == nullptr && bSpawnBoxesOnBeginPlay", EditConditionHides))
	int32 NumBoxesToSpawn = 3;

	//~ 배치 좌표는 여기에 없다 — 출발지/도착지 모두 Layout의 배치된 액터를 읽는다.
	//   지지면 트레이스도 없앴다: 상판 높이는 그 액터의 바운드에서 나오므로 찾을 필요가 없다.

	#pragma endregion

	#pragma region Layout

	/**
	 * **출발지** — 박스가 이 액터의 상판 위에 스폰된다 (팔레트). 레벨에 직접 배치한 액터를 할당한다.
	 *
	 * @details
	 * 위치는 **계산 대상이 아니라 사용자 의도**다. 좌표로 계산해 스폰하면 메시 피벗, 지지면 트레이스,
	 * 로봇 로컬 변환이 전부 곱해져 어디로 갈지 예측이 안 된다(실제로 그래서 엉뚱한 곳에 스폰됐다).
	 * 뷰포트에서 원하는 곳에 끌어다 놓은 그 트랜스폼이 곧 정답이므로 읽기만 한다.
	 *
	 * 슬롯은 이 액터 기준으로 자동 배치된다 (FPickPlaceLayout::BuildSlotsOnSurface 참조):
	 * - 위치: 액터 로컬 공간에서 stride 간격, 행 중앙이 **바운드 중심**에 오도록 정렬
	 * - 방향: 액터를 회전시키면 슬롯 행도 같이 돈다
	 * - 높이: **바운드 윗면** → 피벗이 어디 있든 박스가 상판에 정확히 앉는다
	 *
	 * **standalone 전용** — Dispatcher가 물려 있으면 dispatcher의 동명 프로퍼티가 쓰이고 이 값은 무시된다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Layout",
		meta = (EditCondition = "Dispatcher == nullptr", EditConditionHides))
	TObjectPtr<AActor> SourceSurfaceActor;

	/** 출발지 슬롯 간격 (cm, **출발지 액터 로컬 공간**). standalone 전용. 단층 1열 — N층 적재/해체는 범위 밖이다. */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Layout",
		meta = (EditCondition = "Dispatcher == nullptr", EditConditionHides))
	FVector SourceSlotStrideCm = FVector(0.0, 40.0, 0.0);

	/** 출발지 슬롯 행 미세 조정 (cm, 출발지 액터 로컬). standalone 전용. 0이면 행이 바운드 중심에 정렬된다. */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Layout",
		meta = (EditCondition = "Dispatcher == nullptr", EditConditionHides))
	FVector SourceSlotOffsetCm = FVector::ZeroVector;

	/**
	 * **도착지** — 박스를 이 액터의 상판 위에 갖다 놓는다 (레일/컨베이어). standalone 전용.
	 * 규약은 SourceSurfaceActor와 완전히 동일하다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Layout",
		meta = (EditCondition = "Dispatcher == nullptr", EditConditionHides))
	TObjectPtr<AActor> DestinationSurfaceActor;

	/** 도착지 슬롯 간격 (cm, **도착지 액터 로컬 공간**). standalone 전용. */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Layout",
		meta = (EditCondition = "Dispatcher == nullptr", EditConditionHides))
	FVector DestinationSlotStrideCm = FVector(0.0, 40.0, 0.0);

	/** 도착지 슬롯 행 미세 조정 (cm, 도착지 액터 로컬). standalone 전용. */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Layout",
		meta = (EditCondition = "Dispatcher == nullptr", EditConditionHides))
	FVector DestinationSlotOffsetCm = FVector::ZeroVector;

	/**
	 * true면 IK가 GraspRotation까지 맞추고, false면 **위치만 맞추고 툴 회전은 로봇이 내는 대로 둔다.**
	 *
	 * 기본값 false — 이 프로젝트의 로봇이 KUKA R3200 QUANTEC **PA(Palletizing)**이기 때문이다.
	 * 팔레타이징 로봇은 설계상 플랜지가 항상 아래를 향하도록 손목 자유도가 제한돼 있어서, 툴 자세를
	 * 외부에서 지정하려 들면 낼 수 없는 자세를 요구하게 된다. 실제로 GraspRotation을 강제했더니
	 * 어떤 반경에서도 도달 불가였고, DLS IK가 RotationWeight=30으로 불가능한 회전을 쫓느라 위치를
	 * 통째로 희생해 팔이 목표에서 190cm 떨어진 곳으로 날아갔다.
	 *
	 * 손목이 자유로운 6R 로봇으로 교체하면 켜서 툴 자세를 통제할 수 있다. 켜기 전에 StartCycle의
	 * "도달 반경 탐색 (B)" 로그가 그 자세로 도달 가능하다고 말하는지 먼저 확인할 것.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Layout")
	bool bConstrainGraspRotation = false;

	/**
	 * 파지/적재 시 툴 팁의 목표 회전 (**로봇 액터 공간**). 집을 때와 놓을 때 모두 같은 값을 쓴다.
	 * bConstrainGraspRotation이 false면 IK 목표로는 쓰이지 않고 도달 반경 탐색 로그의 기준으로만 남는다.
	 * 값은 툴 +X축(기본 모델의 ToolOffset 방향)이 아래(-Z)를 향하는 수직 하강 자세를 뜻한다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Layout")
	FRotator GraspRotation = FRotator(-90.0, 0.0, 0.0);

	/**
	 * 접근/이탈 시 목표 지점에서 띄울 오프셋 (cm, 로봇 액터 공간).
	 * ToPickApproach/ToLift/ToPlaceApproach/ToRetreat가 이만큼 위에서 대기하고,
	 * ToPick/ToPlace만 실제 목표 지점으로 내려간다. 이 수직 진입이 박스 옆면과의 간섭을 피하는
	 * 유일한 수단이다 (충돌 회피 경로계획은 범위 밖).
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Layout")
	FVector ApproachOffsetCm = FVector(0.0, 0.0, 25.0);

	#pragma endregion

	#pragma region Motion

	/**
	 * FSM/궤적을 전진시키는 고정 타임스텝 (초). 기본값 1/120초.
	 *
	 * 프레임 DeltaSeconds를 누적해 이 단위로만 적분하므로, 프레임레이트와 무관하게 궤적과 CSV
	 * 샘플 간격이 동일하다. 필수 항목 "고정 타임스텝 + 물리값 CSV"가 요구하는 결정론의 근거다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Motion", meta = (ClampMin = "0.0005"))
	double FixedTimeStepSec = 1.0 / 120.0;

	/**
	 * 관절 최대 각속도(FRobotJointLimit::MaxVelRadPerSec) 대비 실제 사용 비율 (0~1).
	 * 궤적 소요시간을 이 비율로부터 역산한다 — 낮을수록 느리고 안전하다.
	 * 시간최적 S-curve 프로파일링은 범위 밖이므로, 여기서는 "한계의 일정 비율"이라는 단순 규칙만 쓴다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Motion", meta = (ClampMin = "0.01", ClampMax = "1.0"))
	double VelocityScale = 0.35;

	/**
	 * Grasp/Release 단계에서 정지해 있는 시간 (초). 실제 그리퍼 개폐 시간의 대역이다.
	 * 물리적 그리퍼가 없는 슬라이스이므로 개폐를 "시간"으로만 모델링한다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Motion", meta = (ClampMin = "0.0"))
	double DwellSec = 0.3;

	/**
	 * 궤적 최소 소요시간 (초). 이동량이 거의 0인 구간에서 duration이 0으로 수렴해
	 * 관절이 순간이동(무한 가속)하는 것을 막는 하한이다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Motion", meta = (ClampMin = "0.01"))
	double MinTrajectoryDurationSec = 0.4;

	/**
	 * 단계 목표에 이만큼(cm) 이내로 도달하지 못하면 사이클을 Aborted로 중단한다.
	 *
	 * DLS IK의 수렴 허용오차가 0.5cm이므로 정상 solve는 항상 이 값 아래로 들어온다. 초과 = 목표가
	 * 작업 반경 밖이거나 그 자세로는 도달 불가라는 뜻이다. 이때 그냥 최선해로 진행하면 툴이 목표에서
	 * 1m 떨어진 채 박스를 든 것처럼 움직여서 원인이 보이지 않으므로, 멈추고 로그로 말한다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Motion", meta = (ClampMin = "0.1"))
	double MaxReachErrorCm = 3.0;

	/**
	 * 시각 파지점 반복 보정의 감쇠 계수 α (0~1). **1.0은 거의 항상 발산한다.**
	 *
	 * @details
	 * 반복은 `X ← X + α·(목표 − f(X))`이고, f(X)는 "수학 EE를 X로 보냈을 때 시각 그리퍼가 가는 곳"이다.
	 * KUKA 메시(3.2m)는 수학 팔(1.6m)의 약 2배 크기이고 두 체인이 같은 관절각으로 도므로
	 * **f(X) ≈ 2X**다. 그러면 반복 행렬이 (1 − 2α)이므로:
	 *   - α=1.0 → |1−2| = 1 → **진동**하며 영원히 수렴하지 않는다 (실제로 잔여 오차 77.6cm로 실패했다)
	 *   - α=0.5 → 0 → **한 번에 수렴** (메시/수학 크기비가 정확히 2일 때 최적)
	 *   - 일반적으로 크기비 k에 대해 α = 1/k가 최적이고, 0 < α < 2/k면 수렴한다
	 *
	 * 즉 이 값은 대략 **(수학 팔 도달거리 ÷ 메시 팔 도달거리)** 로 두면 된다. 기본값 0.5는
	 * 1.6m 수학 팔 + 3.2m KUKA 조합에 맞춘 값이다. 다른 메시로 바꾸면 이 비율을 다시 잡아야 한다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Motion", meta = (ClampMin = "0.05", ClampMax = "1.0"))
	double VisualSolveDamping = 0.5;

	#pragma endregion

	#pragma region Logging

	/** CSV 물리값 로깅 활성화 여부. 켜면 고정 타임스텝마다 한 행씩 버퍼에 쌓고 사이클 종료 시 기록한다. */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Logging")
	bool bEnableCsvLogging = true;

	/**
	 * CSV 파일명. 저장 경로는 <Project>/Saved/RobotSim/<CsvFileName>.
	 *
	 * **멀티로봇 주의**: 같은 이름을 쓰는 태스크 액터가 둘 이상이면 서로 덮어써서 한쪽 데이터가
	 * 조용히 사라진다. StartCycle이 그 충돌을 감지하면 **양쪽 모두** 액터 이름을 붙여 자동으로
	 * 구분하고 Warning을 남긴다 (한쪽만 붙이면 어느 파일이 누구 것인지 알 수 없어 비대칭이 된다).
	 * 자동 구분에 기대지 말고 로봇마다 이름을 직접 주는 편이 읽기 좋다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Logging")
	FString CsvFileName = TEXT("PickPlace.csv");

	/** 현재 단계/박스 인덱스를 화면에 표시할지 여부 (데모 영상용). */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Logging")
	bool bDrawDebugStatus = true;

	/**
	 * StartCycle에서 시각 그리퍼 도달 XY반경 구간을 탐색해 로그에 찍을지 여부 (배치 튜닝용 진단).
	 *
	 * 반경 하나마다 IK를 최대 12회 + 메시 동기화를 돌리므로 PIE 시작이 잠깐 멈춘다. 배치가 확정된
	 * 뒤에는(특히 데모 녹화 시) 꺼도 된다 — 사이클 동작에는 아무 영향이 없다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Logging")
	bool bLogWorkspaceProbe = true;

	#pragma endregion

	#pragma region Replay

	/**
	 * 재생할 모션 CSV 파일명 (경로는 <Project>/Saved/RobotSim/<이름>).
	 * **비어 있으면 이 액터의 CsvFileName을 쓴다** — 방금 자기가 기록한 궤적을 그대로 재생하는 것이
	 * 가장 흔한 경우이므로 기본값을 그쪽에 맞춘다. 멀티로봇에서 파일명이 충돌하면 StartCycle이
	 * 액터 이름을 붙이므로, 그 경우엔 실제 저장된 파일명(예: PickPlace_A.csv)을 여기에 지정한다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Replay")
	FString ReplayMotionFileName;

	/**
	 * 켜면 새 사이클을 기존 CSV **뒤에 이어붙인다** (여러 실행을 한 파일에 누적). 끄면(기본) 매 실행이
	 * 파일을 덮어써 항상 "가장 최근 사이클"만 남는다.
	 *
	 * @details
	 * 켜두면 PIE를 여러 번 돌린 궤적이 한 파일에 시간순으로 쌓이고, PlayReplay가 그 전체를 이어서
	 * 재생한다 — "여러 사이클 누적 재생". 누적을 처음부터 다시 시작하려면 ClearMotionCsv를 누른다.
	 *
	 * 구현은 디스크 append가 아니라 **기존 파일을 읽어 앞에 붙여 다시 쓰기**다: append는 UTF-8 BOM이
	 * 파일 중간에 박혀 파서를 깨뜨린다. 기존 파일의 헤더가 현재 헤더와 다르면(컬럼 스키마 변경) 섞지
	 * 않고 경고 후 새로 시작한다 — 스키마가 다른 행이 섞이면 재생/분석이 조용히 틀어지기 때문이다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Replay")
	bool bAccumulateMotionAcrossRuns = false;

	#pragma endregion

private:
	#pragma region FSMState

	/** 현재 FSM 단계. */
	EPickPlacePhase Phase = EPickPlacePhase::Idle;

	/** 현재 처리 중인 박스 인덱스 (Boxes 기준). */
	int32 CurrentBoxIndex = INDEX_NONE;

	/** 현재 단계에서 경과한 시간 (초). */
	double PhaseElapsedSec = 0.0;

	/** 현재 단계의 총 소요시간 (초). 궤적은 역산값, Grasp/Release는 DwellSec. */
	double PhaseDurationSec = 0.0;

	/** 사이클 시작 이후 누적 시뮬레이션 시간 (초). */
	double SimTimeSec = 0.0;

	/** 프레임 DeltaSeconds 누적기. FixedTimeStepSec 단위로 소진한다. */
	double TimeAccumulatorSec = 0.0;

	/**
	 * 자동 시작이 예약됐는지. BeginPlay가 아니라 **첫 Tick**에서 StartCycle을 부르기 위한 플래그다.
	 *
	 * BeginPlay에서 부르면 로봇의 BeginPlay가 아직 안 돌았을 수 있어 VisualGraspPoint가 본에 붙기 전이고,
	 * 그러면 파지 기준이 조용히 수학 EE로 폴백해 반복 보정이 통째로 무의미해진다.
	 * AddTickPrerequisiteActor는 Tick 순서만 보장하지 BeginPlay 순서는 보장하지 않는다.
	 */
	bool bPendingAutoStart = false;

	/** StartCycle이 실행됐는지. dispatcher가 배급 가능 여부를 판단하는 조건 (IsIdle 주석 참조). */
	bool bCycleStarted = false;

	/** dispatcher가 배급한 현재 작업 (standalone 모드에서는 항상 비어 있다). */
	FPickPlaceTask AssignedTask;

	#pragma endregion

	#pragma region Trajectory

	/** 관절 상태의 source of truth. 매 프레임 Robot->SetJointAngles로 밀어넣는다. */
	FRobot6DJointState ActiveState;

	/** 직전 스텝의 관절 상태. CSV 각속도를 유한차분으로 구하는 데 쓴다. */
	FRobot6DJointState PreviousState;

	/** 현재 궤적의 시작 관절 자세. */
	FRobot6DJointState TrajectoryStartState;

	/** 현재 궤적의 목표 관절 자세 (단계 진입 시 IK로 한 번 계산). */
	FRobot6DJointState TrajectoryGoalState;

	/** 현재 단계의 목표 툴 팁 자세 (**로봇 액터 공간**). CSV의 target 열과 오차 계산에 쓴다. */
	FTransform CurrentTargetLocal = FTransform::Identity;

	/**
	 * ToPickApproach 진입 시점에 스냅샷한 박스 파지 자세 (로봇 액터 공간).
	 * 박스는 물리 바디라 미세하게 움직일 수 있으므로, 접근과 하강이 같은 지점을 겨냥하도록 고정한다.
	 */
	FTransform PickPoseLocal = FTransform::Identity;

	/**
	 * 팔레트 슬롯별 툴 목표 위치 (로봇 액터 공간), StartCycle에서 지지면 트레이스로 확정한다.
	 * 박스 바닥이 지지면에 정확히 닿도록 슬롯 Z = 지지면 Z + 박스 절반 높이로 보정된 값이다.
	 */
	TArray<FVector> PalletSlotLocations;

	#pragma endregion

	#pragma region Grasp

	/** 현재 파지 중인 박스 (없으면 nullptr). */
	UPROPERTY()
	TObjectPtr<APickPlaceBoxActor> HeldBox;

	/** 파지 순간의 툴 팁 기준 박스 상대 변환. 파지 중 박스 = 이 값 * 툴 팁 월드. */
	FTransform HeldBoxRelativeToTool = FTransform::Identity;

	#pragma endregion

	#pragma region Telemetry

	//~ UI getter와 CSV가 **공유하는 캐시**다 (STEP D-01). 고정 스텝마다 한 번 갱신되며, 읽는 쪽은
	//~ 절대 재계산하지 않는다. 두 소비자가 각자 계산하면 같은 프레임에 다른 토크를 보게 되고,
	//~ 그러면 "화면의 게이지가 CSV와 다르다"는 조용한 불일치가 생긴다.

	/** 직전 고정 스텝의 관절 각속도 (rad/s). 유한차분 (ActiveState − PreviousState) / FixedTimeStepSec. */
	FRobot6DJointVelocity LastJointVelocityRadPerSec;

	/**
	 * 직전 고정 스텝의 관절 토크 (N·m) — B-02 RNEA 결과.
	 *
	 * **qdd=0 준정적 추정이다.** 현 구동은 위치 지령 + 관절공간 보간이라 관절 가속도가 동역학
	 * 적분의 결과가 아니다. 궤적의 2차 미분을 넣어도 "실제 구동 토크"라는 물리적 의미가 없으므로
	 * 넣지 않는다. 살아 있는 항은 **중력과 마찰**이고, 관성 M(q)q̈와 코리올리 C(q,q̇)q̇는 빠진다.
	 * 온전한 토크는 B-06 computed torque 제어가 들어와야 나온다.
	 */
	FRobot6DJointTorque LastJointTorqueNm;

	/** 프레임 시간 (ms), EMA 평활값. 원시값은 진동이 심해 HUD에서 읽을 수 없다. */
	double LastFrameTimeMs = 0.0;

	/** 마지막 IK가 남긴 시각 파지점 오차 (cm). BeginTrajectoryTo가 채운다. */
	double LastReachErrorCm = 0.0;

	/** 사이클 시작 이후 완료한 박스 수. */
	int32 CompletedBoxCount = 0;

	/** 일시정지 (UI). FSM 전진과 CSV 기록만 멈추고 관절 밀어넣기는 계속한다 — SetPaused 주석 참조. */
	bool bPaused = false;

	#pragma endregion

	#pragma region CsvBuffer

	/** CSV 행 버퍼. 사이클 종료(Done/Aborted) 또는 EndPlay/FlushCsvNow 시 파일로 기록한다. */
	TArray<FString> CsvRows;

	/**
	 * 실제로 기록할 CSV 파일명. StartCycle이 다른 태스크 액터와의 충돌을 검사해 확정한다.
	 * 충돌이 없으면 CsvFileName 그대로, 있으면 액터 이름이 붙는다. 비어 있으면 아직 미확정이다.
	 */
	FString ResolvedCsvFileName;

	/**
	 * bAccumulateMotionAcrossRuns가 켜졌을 때, 이번 사이클 시작 시점에 기존 파일에서 읽어둔 과거 데이터 행
	 * (헤더 제외). WriteCsvToDisk가 [헤더] + 이 행들 + [이번 사이클 행] 순서로 써서 누적을 유지한다.
	 */
	TArray<FString> AccumulatedPriorRows;

	#pragma endregion

	#pragma region ReplayState

	/**
	 * 재생 중인가. **FSM 단계가 아니라 그 위에 얹는 별도 모드다** — EPickPlacePhase는 건드리지 않는다.
	 * true면 Tick이 FSM 전진을 통째로 건너뛰고 재생 스테퍼만 돌리며, IsIdle()이 false가 되어
	 * dispatcher가 이 로봇에 배급하지 않는다.
	 */
	bool bReplayActive = false;

	/** LoadMotionCsv가 파싱한 관절 궤적. 각 원소가 한 고정 스텝의 6관절 각도(radian). */
	TArray<FRobot6DJointState> ReplayFrames;

	/** 현재 재생 프레임 인덱스. */
	int32 ReplayFrameIndex = 0;

	/** 재생 프레임 전진용 시간 누적기 (FSM의 TimeAccumulatorSec와 독립 — 두 모드가 공존하지 않으므로 분리한다). */
	double ReplayTimeAccumulatorSec = 0.0;

	#pragma endregion

	#pragma region Internal

	/** FSM을 고정 타임스텝 하나만큼 전진시킨다. Tick이 누적기를 소진하며 반복 호출한다. */
	void StepFixed(double DeltaSec);

	/**
	 * 재생을 고정 타임스텝으로 전진시킨다 (Tick이 bReplayActive일 때 호출). 기록이 CSV 한 행 = 한
	 * 고정 스텝으로 저장했으므로, 재생도 FixedTimeStepSec마다 프레임 하나를 ActiveState에 되넣는다.
	 * 마지막 프레임을 지나면 bReplayActive=false로 끄고 Idle로 복귀한다.
	 */
	void StepReplay(double DeltaSec);

	/**
	 * 새 단계로 진입한다. 궤적 단계면 목표 자세를 정해 IK를 풀고 궤적을 준비하며,
	 * Grasp/Release면 박스 부착/해제를 수행하고 대기 시간을 건다.
	 */
	void EnterPhase(EPickPlacePhase NewPhase);

	/** 현재 단계가 끝났을 때 다음 단계로 전이한다 (ToRetreat 끝의 다음 박스/종료 분기 포함). */
	void AdvanceToNextPhase();

	/** 사이클을 Aborted로 중단하고 이유를 Error 로그로 남긴다. 파지 중인 박스는 놓는다. */
	void AbortCycle(const FString& Reason);

	/** CurrentBoxIndex 다음의 유효한(None 아닌) 박스 인덱스를 찾는다. 없으면 INDEX_NONE. */
	int32 FindNextValidBoxIndex(int32 SearchFrom) const;

	/**
	 * BoxClass를 로봇 기준 위치에 NumBoxesToSpawn개 스폰해 Boxes를 채운다.
	 * 각 박스는 지지면 트레이스 결과 위에 정확히 얹어 스폰하므로, 사이클 시작 시점에 이미 정지해 있다.
	 */
	void SpawnBoxes();


	/**
	 * 지정 방향/높이에서 **시각 그리퍼**가 도달 가능한 XY반경 구간 [최소, 최대]를 훑어 로그에 찍는다.
	 *
	 * 배치가 실패했을 때 "얼마나 어느 쪽으로 옮겨야 하는가"를 추측하지 않기 위한 진단이다. 대형 6축 팔은
	 * 최소 반경(dead zone)이 커서 베이스에 **너무 가까운** 지점도 도달 불가인데, 이 구간을 모르면 목표를
	 * 반대 방향으로 옮기는 실수를 하게 된다.
	 *
	 * 수학 EE가 아니라 시각 그리퍼 기준이다 — 박스가 붙는 곳이 거기이고 사이클의 도달 판정도 그 기준이라,
	 * 수학 EE 구간을 찍으면 실제 동작과 다른 숫자를 보고하게 된다.
	 *
	 * 단위는 **XY 평면 반경**이다(3D 거리가 아니다). 중단 메시지도 같은 양을 찍어야 비교가 성립한다.
	 * 관절을 잠시 움직이지만 원래 자세로 복원하므로 사이클 상태에는 영향이 없다.
	 */
	void LogReachableRadiusBand(const FVector& DirectionXY, double ZLocal, const TCHAR* Label);

	/**
	 * 도착지 슬롯별 툴 목표 위치를 확정해 PalletSlotLocations(로봇 공간)를 채운다.
	 * **standalone 모드 전용** — dispatcher 모드에서는 dispatcher가 슬롯을 소유한다.
	 * @param BoxHeightCm 박스 전체 높이. 파지점이 박스 윗면이므로 툴 목표 Z = 상판 + 이 값이다.
	 */
	void BuildPalletSlots(double BoxHeightCm);

	/** 현재 처리 중인 박스 — dispatcher 모드면 배급된 박스, standalone이면 Boxes[CurrentBoxIndex]. */
	APickPlaceBoxActor* GetCurrentBox() const;

	/** 현재 작업의 도착지 슬롯 자세 (로봇 공간). 모드에 따라 dispatcher 슬롯 풀 또는 자기 슬롯을 읽는다. */
	FTransform GetCurrentPlacePoseLocal(bool bApproach) const;

	/**
	 * 목표 툴 팁 자세(로봇 공간)를 향해 IK를 풀고 TrajectoryStart/Goal/Duration을 설정한다.
	 * @return 도달 가능해서 궤적을 시작했으면 true. MaxReachErrorCm 초과로 중단했으면 false.
	 */
	bool BeginTrajectoryTo(const FTransform& TargetLocal);

	/**
	 * **시각 파지점**이 DesiredGraspLocal에 오도록 관절 해를 구한다 (고정점 반복).
	 *
	 * @details
	 * 왜 필요한가: IK는 수학 EE만 목표에 보낼 수 있는데, 박스는 시각 파지점에 붙는다. 메시를 축소하지
	 * 않기로 했으므로 두 점은 홈 자세에서만 겹치고 관절이 돌수록 벌어진다(visual calibration offset).
	 * 그대로 두면 수학 EE는 박스에 가는데 그리퍼는 딴 데 있어서, 박스가 그리퍼로 순간이동해 붙고
	 * 팔레트에도 엉뚱한 곳에 놓인다.
	 *
	 * 두 체인이 **같은 관절각**으로 구동되므로 offset은 자세의 매끄러운 함수다. 따라서
	 *   수학목표 ← 수학목표 + (원하는 파지 위치 − 실제 시각 파지점)
	 * 를 반복하면 시각 파지점이 목표로 수렴한다. offset을 상수로 뺄 수 없는 이유가 바로 자세 의존성이고,
	 * 그래서 단계 진입 시마다 이 반복으로 흡수한다.
	 *
	 * 수학 레이어는 호출만 한다 — FSerial6DoFModel/FRobotDlsIK는 수정하지 않는다.
	 *
	 * @param DesiredGraspLocal 시각 파지점이 도달해야 할 자세 (로봇 공간)
	 * @param OutState          해 (수렴 실패 시에도 최선해)
	 * @param OutFinalErrorCm   최종 시각 파지점 오차 (cm)
	 * @return 시각 파지점이 MaxReachErrorCm 이내로 수렴했으면 true
	 */
	bool SolveForVisualGraspPoint(
		const FTransform& DesiredGraspLocal, FRobot6DJointState& OutState, double& OutFinalErrorCm);

	/**
	 * 시작/목표 관절 자세와 속도 한계로부터 궤적 소요시간을 역산한다.
	 *
	 * quintic smoothstep의 최대 각속도는 평균의 15/8배이므로, 관절 i의 소요시간 하한은
	 * (15/8)·|Δq_i| / (MaxVel_i · VelocityScale)이다. 모든 관절 중 최대값과
	 * MinTrajectoryDurationSec 중 큰 쪽을 택하면 어떤 관절도 속도 한계를 넘지 않는다.
	 */
	double ComputeTrajectoryDuration(const FRobot6DJointState& From, const FRobot6DJointState& To) const;

	/** 현재 PhaseElapsedSec 기준으로 궤적을 평가해 ActiveState를 갱신한다. */
	void EvaluateTrajectory();

	/** 파지 중인 박스를 툴 팁에 붙여 따라오게 한다 (물리 off 상태에서 트랜스폼 직접 갱신). */
	void UpdateHeldBoxTransform();

	/** 박스 i의 팔레트 적재 자세 (로봇 공간). bApproach면 ApproachOffsetCm만큼 위. */
	FTransform GetPlacePoseLocal(int32 BoxIndex, bool bApproach) const;

	/** PickPoseLocal에 ApproachOffsetCm을 더한 접근 자세 (로봇 공간). */
	FTransform GetPickApproachPoseLocal() const;

	/** 현재 ActiveState 기준 툴 팁 자세 (로봇 공간 = 수학 FK 결과 그대로). */
	FTransform ComputeToolLocalTransform() const;

	/** 로봇 공간 → 월드 변환 (로봇이 없으면 입력을 그대로 반환). */
	FTransform LocalToWorld(const FTransform& Local) const;

	/**
	 * 구동 중 텔레메트리 캐시를 갱신한다 (각속도 유한차분 + RNEA 토크). 고정 스텝마다 **한 번**.
	 *
	 * 호출 위치가 중요하다: StepFixed가 맨 위에서 PreviousState = ActiveState를 하고 EvaluateTrajectory가
	 * ActiveState를 갱신하므로, **EvaluateTrajectory 뒤 · RecordCsvRow 앞**에서만 불러야 한다.
	 * 그보다 앞에서 부르면 두 상태가 같아 각속도가 항상 0이 되고, 마찰 항까지 조용히 죽는다.
	 */
	void UpdateTelemetryCache();

	/**
	 * 정지 상태(Idle/Done/Aborted/Pause)의 **자세 유지 중력 토크**를 캐시한다. Tick당 한 번.
	 *
	 * 왜 0으로 두지 않는가: dispatcher 모드의 Idle은 "다음 배급 대기 중 자세 유지"이고 Pause도
	 * 마찬가지다 — 팔은 실제로 중력을 버티고 있다. 게이지를 0으로 그리면 화면이 거짓말을 한다.
	 * qd=qdd=0이므로 마찰/관성 항이 자연히 사라져 순수 중력 보상 토크 g(q)만 남는다.
	 */
	void UpdateHoldingTorqueCache();

	/** 현재 스텝의 물리값을 CSV 행 하나로 버퍼에 추가한다. */
	void RecordCsvRow();

	/**
	 * 다른 태스크 액터와 CSV 파일명이 겹치는지 검사해 ResolvedCsvFileName을 확정한다.
	 *
	 * 겹치면 서로 덮어써서 한쪽 데이터가 **조용히** 사라진다 — 로그도 에러도 없이 파일 하나만 남으므로
	 * 멀티로봇에서 가장 놓치기 쉬운 실수다. 충돌 시 양쪽 모두 액터 이름을 붙여 구분한다.
	 */
	void ResolveCsvFileName();

	/** 이 액터가 기록/재생에 쓰는 CSV의 전체 경로 (<Project>/Saved/RobotSim/<확정 파일명>). */
	FString GetMotionCsvPath() const;

	/** CSV 버퍼를 <Project>/Saved/RobotSim/<ResolvedCsvFileName>에 기록한다 (누적 모드면 과거 행을 앞에 붙인다). */
	void WriteCsvToDisk();

	/**
	 * 누적 모드에서 사이클 시작 시 기존 파일의 과거 데이터 행을 AccumulatedPriorRows에 읽어둔다.
	 * 파일이 없으면 빈 채로 둔다. **기존 헤더가 현재 헤더와 다르면** 섞지 않고 경고 후 새로 시작한다.
	 */
	void LoadPriorRowsForAccumulation();

	/** 단계 이름을 로그/CSV용 문자열로 반환한다. */
	static const TCHAR* PhaseToString(EPickPlacePhase InPhase);

	#pragma endregion
};
