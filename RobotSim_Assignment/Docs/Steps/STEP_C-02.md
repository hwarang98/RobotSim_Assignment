# Step C-02: 멀티로봇 작업 할당 (dispatcher)

## 목표

필수 항목 **"멀티 로봇(2대+) 충돌 회피·작업 할당"** 을 채운다. 박스와 슬롯의 소유권을 dispatcher로
옮겨, 어떤 로봇이 어떤 박스를 맡을지 **배분하는 주체**를 만든다.

## 범위 밖

RNEA 통합, 토크 제어, S-curve, 마찰 grasp, **팔 링크 간 정밀 충돌 검사**, 경로계획, UMG.
`ASerial6DoFRobotActor`와 수학 레이어(`FSerial6DoFModel` / `FRobotDlsIK` / RNEA)는 수정하지 않았다.
기존 테스트 40종도 무수정이다.

## 왜 기존 구조가 "할당"이 아닌가

C-01의 `APickPlaceTaskActor`는 **자기 `Boxes` 배열을 소유**하고 `FindNextValidBoxIndex()`로 순서대로
처리한다. 로봇을 2대 놓으면 각자 에디터에서 미리 지정된 자기 박스만 처리하는 **독립 실행 2개**다.
배분하는 주체가 없으니 할당이 아니다.

## 소유권 역전

```
[C-01]  TaskActor ──소유──> Boxes[], 슬롯 풀        (각자 독립)
[C-02]  Dispatcher ─소유──> Boxes[], 슬롯 풀
             │
             └─배급──> TaskActor (FPickPlaceTask 하나씩 수행 → Idle → 다음 배급 대기)
```

`FPickPlaceTask`는 **박스와 슬롯을 한 몸으로** 들고 다닌다. 중단 시 박스만 반납하고 슬롯을 빠뜨리면
그 슬롯이 영구 점유로 새어 뒤쪽 박스들이 놓을 곳을 잃는다.

**FSM 9단계는 두 모드가 완전히 동일하다.** 바뀐 건 "다음 박스를 누가 고르는가"뿐이다.

## 요청이 아니라 배급 — 결정론의 근거

`TryClaimTask(Requester)` 방식(태스크가 자기 Tick에서 요청)은 **채택하지 않았다.**

누가 먼저 요청하느냐가 **액터 틱 순서**에 달리는데, UE는 태스크 액터 A와 B 사이의 틱 순서를 보장하지
않는다 — `AddTickPrerequisiteActor(Robot)`은 로봇→태스크만 잡을 뿐 태스크끼리는 자유다. 그러면 겹친
작업 영역의 박스를 누가 가져갈지가 실행마다 달라져, 고정 타임스텝으로 확보한 결정론이 깨진다.
그건 `APickPlaceTaskActor` 헤더가 이미 주장하고 있는 성질이라 조용히 무효화하면 안 된다.

대신 dispatcher가 **자기 고정 스텝마다 정렬된 순서로** 순회하며 배급한다:

```
정렬: RobotPriority → 액터 이름(문자열)      ← 결정론 절반
for each Task in 정렬된 순서:
    if !Task->IsIdle(): continue
    슬롯 = 도달 가능 + 상호배제 통과한 가장 낮은 빈 슬롯
    후보 = 미할당 && 블랙리스트 아님 && 도달 가능한 박스
    선택 = 베이스에서 가장 가까운 후보
           (거리 동률 ±1cm² 이내면 박스 인덱스가 작은 쪽)   ← 결정론 나머지 절반
    Task->AssignTask({Box, Slot})
```

**정렬 키와 tie-break 두 개가 재실행 시 같은 할당을 보장한다.** `FName::FastLess`는 내부 인덱스
순이라 실행마다 달라질 수 있어 쓰지 않았다 — 이름 문자열 비교를 쓴다.

`IsIdle()`이 `bCycleStarted`를 함께 보는 이유: dispatcher는 태스크 액터보다 먼저 틱하므로 첫
프레임에는 태스크의 `StartCycle`이 아직 안 돌았다. 그때 배급하면 뒤이어 실행되는 `StartCycle`이
Phase를 Idle로 되돌려 **배급이 조용히 증발한다.**

## 도달 가능성 판정과 캐시

### 판정은 사이클과 같은 함수로

`APickPlaceTaskActor::CanReachGraspPointWorld()`는 내부에서 `SolveForVisualGraspPoint`를 부른다 —
**사이클의 도달 판정과 같은 함수다.** 별도의 근사 판정(예: XY반경 구간)을 두면 "배급은 됐는데
실행하면 Aborted"가 되어 판정 두 벌이 어긋난다. 같은 술어를 쓰면 R3200의 최소 반경(dead zone)까지
자동으로 반영된다.

관절 상태는 스냅샷 → 판정 → 복원한다. 안 하면 조회만으로 팔이 튄다 — dispatcher가 사이클 시작 전에
수십 번 부르기 때문이다.

### 캐시가 필요한 이유

`SolveForVisualGraspPoint`는 내부가 고정점 반복이고 반복마다 DLS가 최대 80회 돈다.
**로봇 2대 × 박스 6개 × 2자세(박스/슬롯) = DLS 288회**가 매 배급마다 터진다 — PIE 시작 직후,
즉 **녹화가 시작되는 지점**에서. 스톨이 나면 태스크 액터의 시간 누적기가 튀어
`MaxFixedStepsPerFrame` 클램프가 걸리고, 그건 곧 결정론 경고다.

→ 사이클 시작 시점에 (로봇 × 박스) / (로봇 × 슬롯) 도달성 행렬을 **한 번만** 계산해 캐시하고,
이후 배급은 O(1) 조회로 한다. 박스는 집히기 전까지 움직이지 않으므로 캐시가 유효하다. 스톨도
사이클 시작 전으로 밀려나 녹화에 잡히지 않는다.

## Abort 시 반납 + 블랙리스트

로봇이 중단하면 `ReturnTask(Task, bBlacklist=true, Reason)`을 부른다:

1. **박스를 미할당 풀로 반납** — 다른 로봇이 가져갈 수 있게
2. **슬롯도 함께 반납** — 안 하면 영구 점유로 새어 뒤쪽 박스들이 놓을 곳을 잃는다
3. **(로봇, 박스) 블랙리스트 등록** — 안 하면 같은 로봇이 즉시 그 박스를 다시 가져가 또 실패하고,
   배급→중단→배급이 무한 반복되어 화면에서 팔이 같은 자리를 영원히 버벅인다.
   **다른 로봇은 여전히 그 박스를 시도할 수 있다** (로봇마다 도달 영역이 다르므로).

dispatcher 모드에서 중단은 **로봇의 죽음이 아니라 작업의 실패**다. 반납 후 `Idle`로 돌아가 다음
배급을 기다린다. `Aborted`로 굳히면 로봇 한 대가 통째로 죽어 나머지 작업이 멈춘다.
(standalone 모드는 기존대로 `Aborted`로 간다 — 배급해 줄 주체가 없으므로.)

`ResetCycle` / `EndPlay`에서도 반납한다. 단 블랙리스트는 걸지 않는다 — 리셋은 그 로봇이 그 박스에
실패했다는 뜻이 아니다.

## standalone 폴백 — 이 작업의 보험

**`Dispatcher`가 null이면 C-01과 완전히 동일하게 동작한다.** 자기 `Boxes` 배열, 자기 스폰, 자기 슬롯.
`Boxes` / `bSpawnBoxesOnBeginPlay` / `SourceSurfaceActor` / `NumBoxesToSpawn` 등 기존 프로퍼티를
하나도 지우지 않았다.

이건 유일하게 돌아가는 데모를 건드리는 작업이라, **dispatcher 배선이 틀려도 기존 단일 로봇 데모가
살아 있어야 최악의 경우 영상은 건진다.** dispatcher 경로는 추가지 대체가 아니다.

부수 효과 하나: dispatcher 모드에서 `Idle`은 "배급 대기"이므로 관절 자세를 유지해야 한다
(놓으면 작업 사이마다 팔이 홈으로 튕긴다). standalone의 `Idle`은 여전히 "로봇에게 소유권 반환"이다.

## 충돌 회피 — 도착지 슬롯 상호배제

PDF 원문은 "충돌 회피·작업 할당"이라 할당만으로는 필수의 절반이다. dispatcher가 이미 중재자이므로
나머지 절반을 싸게 채운다:

> **다른 로봇이 작업 중인 슬롯에서 `MinSlotSeparationCm`(기본 60cm) 이내인 슬롯은 배급하지 않는다.**

두 팔이 실제로 부딪칠 위험이 가장 큰 지점 — "둘 다 같은 레일 근처에 놓으러 가는 순간" — 을 정확히
겨냥한다. 놓기가 끝난 슬롯은 팔이 떠났으므로 제약을 푼다(`SlotActiveTask`를 `INDEX_NONE`으로).

### 한계 (명시)

**팔 링크 간 정밀 충돌 검사가 아니다.** 로봇 링크 메시는 `NoCollision`이라 두 팔이 같은 공간을
지나면 그냥 통과한다. 경로 중간의 간섭은 **로봇을 작업 영역이 심하게 겹치지 않게 배치**해서
회피해야 한다. 정밀 충돌은 예약 기반 상호배제나 경로계획이 필요하고, 그건 이 단계의 범위 밖이다.

## Dispatch.csv — 할당의 증거

`Saved/RobotSim/Dispatch.csv`. 평가자에게 "할당이 실제로 일어났다"는 증거이자, 재실행 시 같은
할당이 나오는지(결정론) 확인하는 수단이다.

| 컬럼 | 내용 |
|---|---|
| `sim_time_sec` | dispatcher 고정 스텝 누적 시간. 배급이 dispatcher 틱에서 일어나므로 시간 기준도 dispatcher가 갖는다 |
| `event` | `Assign` / `Complete`(박스 1개 완료) / `Return` / `Blacklist` / `AllDone`(전체 요약 1행) |
| `robot` | 태스크 액터 이름 |
| `box` | 박스 인덱스 (dispatcher 풀 기준) |
| `slot` | 도착지 슬롯 인덱스 |
| `reason` | 배급 근거(`nearest_reachable dist=...`) 또는 반납/블랙리스트 사유 |

로봇별 물리값 CSV(`PickPlace_*.csv`)는 태스크 액터가 따로 기록한다. 같은 파일명을 쓰는 태스크
액터가 둘이면 서로 덮어쓰므로, `StartCycle`이 충돌을 감지해 **양쪽 모두** 액터 이름을 붙인다.

## 코드 공유 — 중복 없음

`BuildSlotsOnSurface` / `SpawnBoxesOnSlots`를 `FPickPlaceLayout`의 static으로 승격해 dispatcher와
태스크 액터가 **같은 함수를 쓴다.** 복제하면 두 경로의 배치 규약이 조용히 갈라진다 — 슬롯 Z 계산
하나만 어긋나도 박스가 상판을 뚫거나 뜬다. 본문 로직은 그대로 옮겼다.

## 테스트를 만들지 않은 근거

**신규 자동화 테스트를 추가하지 않았다.** dispatcher는 월드/액터/PoseableMesh에 묶여 있어 순수
단위테스트가 되지 않는다 — 도달성 판정 하나가 로봇 액터 + SkeletalMesh + 본 부착을 요구한다.
그 하네스(월드 스폰 + 메시 로드 + 결정론 시드)를 만드는 것은 지금 남은 시간에 할 일이 아니고,
급조하면 오히려 신뢰할 수 없는 테스트가 된다.

검증 근거는 다음 둘로 대체한다:

1. **`Dispatch.csv`** — 재실행 시 `Assign` 순서와 (robot, box, slot) 조합이 동일한가.
   결정론이 깨지면 여기서 바로 드러난다.
2. **기존 40종 회귀 통과** — 수학 레이어를 건드리지 않았음의 증거.

수학 레이어(FK/IK/RNEA)는 여전히 순수 단위테스트로 덮여 있고, 이번 변경은 그 레이어에 손대지
않았다는 점이 중요하다.

### 실측 결과 (로봇 2대 / 박스 4개)

```
sim_time_sec, event,    robot,  box, slot, reason
0.01667,      Assign,   C_1,    0,   2,    nearest_reachable dist=190.2cm
0.01667,      Assign,   C_2,    3,   0,    nearest_reachable dist=173.0cm
9.20000,      Complete, C_1,    0,   2
9.20833,      Assign,   C_1,    1,   3,    nearest_reachable dist=292.5cm
11.17500,     Complete, C_2,    3,   0
11.18333,     Assign,   C_2,    2,   1,    nearest_reachable dist=272.2cm
18.19167,     Complete, C_1,    1,   3
20.04167,     Complete, C_2,    2,   1
20.05000,     AllDone,  ?,     -1,  -1,    all_boxes_done n=4
```

**Assign 4 / Complete 4 / Return 0 / Blacklist 0.** 두 로봇이 2개씩 나눠 가졌고, 각자 완료 직후
(9.208s, 11.183s) 다음 작업을 배급받았다 — 유휴 감지와 재배급이 동작한다는 증거다.

## 배치가 곧 성패다 (실패에서 배운 것)

이 시스템의 실패는 대부분 코드가 아니라 **배치**에서 온다. 초기 실행은 6개 중 2개, 그다음 3개만
처리하고 정지했다. 원인과 처방:

| 증상 | 원인 | 처방 |
|---|---|---|
| 슬롯이 `2/6`뿐 | 도착지 행(400cm)이 도달 반경보다 길어 한쪽 끝만 닿음 | `DestinationSlotStrideCm` 축소 |
| 가운데 박스만 `아무도 도달 불가` | 두 로봇의 도달 영역 사이에 **구멍** | `SourceSlotStrideCm` 축소 / 표면을 로봇 쪽으로 |
| 배급됐는데 실행 중 Aborted | 작업 영역 **가장자리**에서 IK가 seed에 민감 | stride 축소 + seed 폴백(아래) |

### IK seed 민감성 — 캐시와 사이클의 판정이 갈라진 이유

작업 영역 가장자리에서는 DLS가 국소 최소점에 갇혀 **seed에 따라 결과가 달라진다.** 실측 증거:
`ToPickApproach`가 성공한 목표에 대해 **좌표가 동일한** `ToLift`가 43cm 오차로 실패했다 —
차이는 IK seed(홈 근처 vs 박스에 내려간 자세)뿐이었다.

도달성 캐시는 홈 자세를 seed로 계산하므로 사이클보다 낙관적이 되고, "배급 → 실행 실패 → 반납 →
블랙리스트"가 반복됐다. 그래서 `SolveForVisualGraspPoint`가 현재 자세로 실패하면 **홈 자세를 seed로
재시도**한다. 이 폴백이 캐시와 사이클의 판정을 일치시킨다.

근본적으로는 **작업 영역 가장자리를 피하는 배치**가 답이다. 반경 250~285cm에서 놀던 것을 stride
축소로 150~200cm대로 당기자 seed 민감도 자체가 사라졌다(4/4 완료).

### 배치 시각화

`bDrawDebugLayout`(기본 켜짐)이 슬롯 행/박스/표면 바운드를 도달 로봇 수로 색칠해 뷰포트에 그린다:
🔴 아무도 못 감 / 🟡 1대 / 🟢 2대 이상(겹침 — 배급 경쟁과 슬롯 상호배제가 일어나는 구간).
빨간 마커가 행의 **어느 쪽 끝에 몰렸는지**가 곧 "행이 긴가 / 치우쳤나"의 답이다. 녹화 시엔 끈다.

## 변경/추가 파일

**신규**
- `Public/Robot/PickPlaceDispatcher.h`, `Private/Robot/PickPlaceDispatcher.cpp`
- `Docs/Steps/STEP_C-02.md` (이 문서)

**수정**
- `Public/Robot/PickPlaceTaskActor.h`, `Private/Robot/PickPlaceTaskActor.cpp`
  - `FPickPlaceTask` / `FPickPlaceLayout`(static 승격) 추가
  - dispatcher API: `IsIdle` / `AssignTask` / `CanReachGraspPointWorld` / `GetRobot` / `GetRobotPriority`
  - `Dispatcher` / `RobotPriority` 프로퍼티
  - `GetCurrentBox` / `GetCurrentPlacePoseLocal`로 모드 분기
  - CSV 파일명 충돌 감지(`ResolveCsvFileName`)

**수정하지 않음**
- `ASerial6DoFRobotActor`, `FSerial6DoFModel`, `FRobotDlsIK`, RNEA, 테스트 40종

## 한계

1. **팔 링크 충돌은 배치로 회피한다** (위 참조).
2. **박스 위치 캐시는 사이클 시작 시점 기준이다.** 박스가 물리로 크게 밀리면 도달성 캐시가
   낡을 수 있다. C-01의 스폰 간격 검사가 그 원인(박스끼리 겹침)을 미리 잡는다.
3. **작업 재배분이 없다.** 로봇 A가 느려도 B가 A의 배급된 작업을 뺏지 않는다. 배급은 유휴 로봇에게만
   일어난다.
4. **블랙리스트는 영구적이다.** 자세가 바뀌면 도달 가능해질 수도 있지만 재시도하지 않는다 —
   무한 루프를 막는 쪽을 택했다.

## 다음 단계

- **B-01 보강** → **B-03** → **B-04/B-05**(S-curve가 C-01의 quintic smoothstep을 대체)
- **B-06**: computed torque 제어 (태스크 액터의 고정 타임스텝 루프와 CSV를 재사용)
- **B-07**: 마찰 원뿔 grasp 판정
