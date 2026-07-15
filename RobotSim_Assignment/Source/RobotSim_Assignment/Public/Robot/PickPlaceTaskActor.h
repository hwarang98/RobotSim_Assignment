// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Robot/RobotTypes.h"
#include "Templates/SubclassOf.h"
#include "PickPlaceTaskActor.generated.h"

class ASerial6DoFRobotActor;
class UStaticMesh;
class UStaticMeshComponent;

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
	 * 처리할 박스들. 배열 순서대로 하나씩 집어 팔레트에 놓는다. 비어 있는(None) 슬롯은 건너뛴다.
	 *
	 * bSpawnBoxesOnBeginPlay가 켜져 있으면 BeginPlay에서 **스폰된 박스로 덮어쓴다** — 이때는 레벨에
	 * 박스를 직접 배치할 필요가 없다. 끄면 여기에 수동 배치한 박스를 할당해 쓴다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Setup")
	TArray<TObjectPtr<APickPlaceBoxActor>> Boxes;

	/** BeginPlay에서 자동으로 사이클을 시작할지 여부. 끄면 StartCycle()을 직접 호출해야 한다. */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Setup")
	bool bAutoStartOnBeginPlay = true;

	#pragma endregion

	#pragma region Spawning

	/**
	 * BeginPlay에서 박스를 자동 스폰할지 여부.
	 *
	 * 켜두는 것을 권장한다. 레벨에 손으로 배치하면 로봇 작업 반경 밖에 두기 쉽고(그러면 IK가 실패한다)
	 * 로봇을 옮길 때마다 다시 배치해야 하지만, 스폰은 좌표가 로봇 기준이라 항상 반경 안에 들어온다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Spawning")
	bool bSpawnBoxesOnBeginPlay = true;

	/**
	 * 스폰할 박스 클래스. **BP 서브클래스를 지정하면 메시/머티리얼/크기를 BP에서 자유롭게 꾸밀 수 있다.**
	 * 비어 있으면 C++ 기본 APickPlaceBoxActor(15cm 회색 큐브)를 쓴다.
	 * 파지 높이는 실제 바운드에서 읽으므로 BP에서 크기를 바꿔도 코드는 그대로 동작한다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Spawning", meta = (EditCondition = "bSpawnBoxesOnBeginPlay"))
	TSubclassOf<APickPlaceBoxActor> BoxClass;

	/** 스폰할 박스 개수. */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Spawning", meta = (ClampMin = "1", EditCondition = "bSpawnBoxesOnBeginPlay"))
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
	 * 슬롯은 이 액터 기준으로 자동 배치된다 (BuildSlotsOnSurface 참조):
	 * - 위치: 액터 로컬 공간에서 stride 간격, 행 중앙이 액터 중심에 오도록 정렬
	 * - 방향: 액터를 회전시키면 슬롯 행도 같이 돈다
	 * - 높이: **바운드 윗면** → 피벗이 어디 있든 박스가 상판에 정확히 앉는다
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Layout")
	TObjectPtr<AActor> SourceSurfaceActor;

	/** 출발지 슬롯 간격 (cm, **출발지 액터 로컬 공간**). 단층 1열 — N층 적재/해체는 범위 밖이다. */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Layout")
	FVector SourceSlotStrideCm = FVector(0.0, 40.0, 0.0);

	/** 출발지 슬롯 행 미세 조정 (cm, 출발지 액터 로컬). 0이면 행이 액터 바운드 중심에 정렬된다. */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Layout")
	FVector SourceSlotOffsetCm = FVector::ZeroVector;

	/**
	 * **도착지** — 박스를 이 액터의 상판 위에 갖다 놓는다 (레일/컨베이어).
	 * 규약은 SourceSurfaceActor와 완전히 동일하다.
	 */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Layout")
	TObjectPtr<AActor> DestinationSurfaceActor;

	/** 도착지 슬롯 간격 (cm, **도착지 액터 로컬 공간**). */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Layout")
	FVector DestinationSlotStrideCm = FVector(0.0, 40.0, 0.0);

	/** 도착지 슬롯 행 미세 조정 (cm, 도착지 액터 로컬). */
	UPROPERTY(EditAnywhere, Category = "PickPlace|Layout")
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

	/** CSV 파일명. 저장 경로는 <Project>/Saved/RobotSim/<CsvFileName>. */
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

	#pragma region CsvBuffer

	/** CSV 행 버퍼. 사이클 종료(Done/Aborted) 또는 EndPlay/FlushCsvNow 시 파일로 기록한다. */
	TArray<FString> CsvRows;

	#pragma endregion

	#pragma region Internal

	/** FSM을 고정 타임스텝 하나만큼 전진시킨다. Tick이 누적기를 소진하며 반복 호출한다. */
	void StepFixed(double DeltaSec);

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
	 * 배치된 액터의 **상판 위**에 슬롯 SlotCount개를 자동 배치해 월드 좌표로 반환한다.
	 *
	 * 출발지(박스 스폰)와 도착지(적재) 양쪽이 이 함수 하나를 공유한다 — 두 곳 다 "배치된 액터를 읽는다"는
	 * 같은 규약이기 때문이다. 좌표를 계산해 맞추려던 시도가 오늘 실패의 대부분이었고, 배치된 것을 읽는
	 * 쪽으로 통일하니 지지면 트레이스와 로봇 로컬 좌표 추측이 통째로 사라졌다.
	 *
	 * - 행은 액터 바운드 중심에 정렬된다 → 박스 개수를 바꿔도 액터를 다시 옮길 필요가 없다
	 * - StrideCm/OffsetCm은 **액터 로컬 공간**이라 액터를 회전시키면 행도 같이 돈다
	 * - 상판 높이는 **바운드**에서 읽으므로 메시 피벗 위치와 무관하다
	 * - Z는 상판 기준으로 덮어쓴다 (액터가 기울어져 있어도 중력 기준으로 쌓아야 하므로)
	 *
	 * @param SurfaceActor          기준 액터 (nullptr이면 false)
	 * @param SlotCount             슬롯 개수
	 * @param StrideCm              슬롯 간격 (액터 로컬)
	 * @param OffsetCm              행 전체 미세 조정 (액터 로컬)
	 * @param HeightAboveSurfaceCm  상판에서 띄울 높이 (스폰=0, 툴 목표=박스 높이)
	 * @param OutSlotWorld          결과 슬롯 월드 좌표
	 * @return 유효한 상판을 얻어 슬롯을 만들었으면 true
	 */
	bool BuildSlotsOnSurface(
		const AActor* SurfaceActor, int32 SlotCount, const FVector& StrideCm, const FVector& OffsetCm,
		double HeightAboveSurfaceCm, TArray<FVector>& OutSlotWorld) const;

	/**
	 * 도착지 슬롯별 툴 목표 위치를 확정해 PalletSlotLocations(로봇 공간)를 채운다.
	 * @param BoxHeightCm 박스 전체 높이. 파지점이 박스 윗면이므로 툴 목표 Z = 상판 + 이 값이다.
	 */
	void BuildPalletSlots(double BoxHeightCm);

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

	/** 현재 스텝의 물리값을 CSV 행 하나로 버퍼에 추가한다. */
	void RecordCsvRow();

	/** CSV 버퍼를 <Project>/Saved/RobotSim/<CsvFileName>에 기록하고 버퍼를 비운다. */
	void WriteCsvToDisk();

	/** 단계 이름을 로그/CSV용 문자열로 반환한다. */
	static const TCHAR* PhaseToString(EPickPlacePhase InPhase);

	#pragma endregion
};
