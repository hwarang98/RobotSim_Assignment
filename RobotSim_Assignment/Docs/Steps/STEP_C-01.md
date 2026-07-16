# Step C-01: Pick & Place 수직 슬라이스 (디팔레타이징 한 사이클)

## 목표

박스를 집어서 다른 표면에 놓는 **한 사이클이 실제로 돌아가게** 한다. 데모 영상의 내용물이자,
필수 항목(고정 타임스텝 + 물리값 CSV, 멀티로봇 작업 할당)의 전제조건이다.

시나리오는 **디팔레타이징**이다 — 팔레트 위에 스폰된 박스를 하나씩 집어 레일/컨베이어에 옮긴다.
이 프로젝트의 로봇이 KUKA R3200 QUANTEC **PA(Palletizing)** 이므로 실제 용도와 일치한다.

## 범위 밖 (이번 단계에서 구현하지 않음)

RNEA 역동역학 소비, 순동역학, 시간최적 S-curve, computed torque 제어, 마찰 원뿔 grasp 판정,
N층 파라메트릭 적재, 충돌 회피 경로계획, 모션 JSON 기록/리플레이, UMG 위젯.

현재 구동은 **기구학**이다: `SetJointAngles`로 관절각을 직접 밀어넣으며, 토크는 계산하지도
적용하지도 않는다. B-02의 RNEA는 아직 이 경로에 연결되지 않았다.

> **D-01 갱신**: RNEA는 이제 **소비된다** — D-01이 고정 스텝마다 토크를 계산해 CSV `tau*` 컬럼과
> UI 게이지에 노출한다. 다만 **구동은 여전히 기구학이다.** 토크는 계산되지만 적용되지는 않는다
> (`## 한계` 1번 참조). computed torque 제어는 여전히 범위 밖이며 B-06 몫이다.

## STEP A 회귀 위험을 0으로 만든 설계

STEP A의 수학 레이어(`FSerial6DoFModel` / `FRobotPoseError` / `FRobotJacobian` / `FRobotDlsIK`)와
테스트 32종은 **한 줄도 수정하지 않았다.** `APickPlaceTaskActor`는 로봇의 public API
(`SetJointAngles` / `GetModel` / `GetJointState` / `GetEndEffectorPose`)만 호출한다. 이 액터를
레벨에서 빼면 STEP A 동작이 그대로 남는다.

### 틱 순서 의존 (중요)

`ASerial6DoFRobotActor::Tick`은 매 틱 `ApplyAnglesFromEditor()`로 에디터 프로퍼티
`JointAnglesDeg`(= 홈 자세)를 관절 상태에 되쓴다. 따라서 태스크 액터는 반드시 로봇 **뒤에**
틱해야 하며, `BeginPlay`에서 `AddTickPrerequisiteActor(Robot)`로 순서를 강제한다. 로봇을
수정하지 않고 되쓰기를 이기는 유일한 방법이다.

이 구조의 대가 — **프레임 전반부에는 로봇이 홈 자세다**:

| 프레임 내 순서 | 로봇 자세 |
|---|---|
| 1. 로봇 Tick → `ApplyAnglesFromEditor()` | 홈 자세로 리셋 (메시도 여기로 동기화) |
| 2. 태스크 Tick → `StepFixed()` | 아직 홈 자세 |
| 3. 태스크 Tick → `SetJointAngles(ActiveState)` | 진짜 자세로 복구 |
| 4. 태스크 Tick → `UpdateHeldBoxTransform()` | 진짜 자세 (박스가 여기서 붙는다) |

렌더링은 모든 틱 뒤에 일어나므로 **화면은 항상 옳다.** 그러나 2번 시점에 로봇/메시 컴포넌트를
읽으면 홈 자세 값이 나온다. 실제로 CSV에 흡착판 월드 좌표를 기록했다가 사이클 내내 상수로
찍혀서 시뮬레이션이 깨진 줄 알았다 — 화면은 멀쩡했고 계측기만 틀렸다. 그래서 **CSV는 로봇
컴포넌트를 읽지 않고 `ActiveState`에서 직접 계산한 값만 기록한다.**

같은 이유로 로봇의 `bDrawDebugFrames`와 `PoseLogIntervalSeconds`는 꺼야 한다 — 둘 다 로봇 Tick
안에서 실행되어 홈 자세를 그리고 기록한다.

## visual calibration offset (핵심 설계 결정)

### 문제

수학 모델(`FSerial6DoFModel`)은 도달거리 약 **1.6m**인 단순화된 6R이고, 시각화에 쓰는 KUKA
메시는 **3.2m**다. `SyncSkeletalPoseToMath()`의 델타 회전 리타겟은 본 **회전만** 얹고 본
길이는 메시 고유값을 유지하므로, 시각 관절 위치는 메시 비율을 따르고 수학 프레임과 어긋난다.
이는 STEP A가 "시각 전용이므로 허용"으로 받아들인 동작이며 해당 함수 주석에 명시되어 있다.

즉 **수학 EE와 시각 그리퍼는 홈 자세에서만 겹치고, 관절이 돌수록 벌어진다.**

### A-06.1 ToolOffset 캘리브레이션이 이 문제를 가렸다

A-06.1은 Target을 시각 그리퍼 끝에 두고 `ToolOffset`을 역산했다. 그 결과가
`(114.3, -0.4, 64.2)cm` — **길이 131cm의 막대기**였다. 그리퍼가 아니라 수학 팔과 메시의 크기
차이를 홈 자세에서 상쇄하는 보정값이었다.

이 캘리브레이션의 검증 로그는 홈 자세에서 초록불이었다. 당연하다 — 그 자세에서 맞도록 계산된
값이니까. **관절이 돌면 131cm 지렛대가 수학 EE를 엉뚱한 곳으로 휘두른다**는 사실은 STEP C에서
팔을 실제로 움직이고 나서야 드러났다. 한 자세에서만 검증하는 캘리브레이션의 구조적 한계다.

> **교훈**: 자세 의존적인 정합은 반드시 **여러 자세에서** 검증해야 한다. `LogVisualToolAlignment()`가
> 홈 자세에서 실행되면 "홈에서 맞는 건 아무것도 증명하지 못한다"고 경고하는 이유다.

### 선택한 해법: 메시를 유지하고 offset을 인정한다

검토한 대안:

| 방안 | 결과 | 채택 |
|---|---|---|
| 메시를 수학 크기로 균일 축소 | 두 체인이 겹치나 3.2m 산업용 로봇이 장난감이 됨. 비율이 다르면 잔차도 남음 | ❌ |
| `LinkOffsets`를 KUKA 실측값으로 재작성 | 정답이지만 FK/IK/골든 테스트 전면 재검증 필요 | ❌ (마감) |
| **메시 유지 + 파지 기준을 시각 그리퍼로 이전** | 보이는 그리퍼가 물체를 잡는다. 수학 EE는 IK/검증 기준으로 유지 | ✅ |

**수학 모델은 source of truth로 유지한다.** 대신 박스는 수학 EE가 아니라
`ASerial6DoFRobotActor::VisualGraspPoint`(그리퍼 본 `VisualGraspBoneName`에 부착된 SceneComponent)에
붙인다. 두 점의 차이가 **visual calibration offset**이며, 버그가 아니라 이 결정의 대가다.
자세마다 값이 달라지므로 상수로 뺄 수 없다. `LogVisualToolAlignment()`가 현재 자세의 offset을
보고한다.

### 고정점 반복으로 offset을 흡수한다

문제가 하나 남는다: IK는 **수학 EE**만 목표로 보낼 수 있는데 박스는 **시각 그리퍼**에 붙는다.
보정 없이 두면 수학 EE만 박스에 가고 그리퍼는 딴 데 있어서, 박스가 순간이동해 붙고 도착지에도
엉뚱한 곳에 놓인다.

두 체인이 **같은 관절각**으로 구동되므로 offset은 자세의 매끄러운 함수다. 따라서
`SolveForVisualGraspPoint()`가 고정점 반복으로 역산한다:

```
수학목표 ← 원하는 파지 위치
반복:
    q ← IK(수학목표)
    q 적용 → 메시 동기화 → 시각 그리퍼 위치 f(수학목표) 측정
    수학목표 += α · (원하는 위치 − f(수학목표))
```

**감쇠 계수 α가 필수다.** 메시(3.2m)가 수학 팔(1.6m)의 약 2배이므로 `f(X) ≈ 2X`이고, 그러면
무감쇠(α=1) 반복은 `X ← X + (T − 2X) = T − X`가 되어 두 값 사이를 **영원히 진동한다**
(실제로 잔여 오차 77.6cm로 실패했다). 반복 행렬이 `(1 − 2α)`이므로:

- α = 1.0 → |1−2| = 1 → 진동, 수렴 안 함
- **α = 0.5 → 0 → 한 번에 수렴** (크기비가 정확히 2일 때 최적)
- 일반적으로 크기비 k에 대해 α = 1/k가 최적이고 `0 < α < 2/k`면 수렴

`VisualSolveDamping` 기본값 0.5는 이 조합에 맞춘 값이며, 대략
**(수학 팔 도달거리 ÷ 메시 팔 도달거리)** 로 두면 된다.

도달 판정도 **시각 그리퍼 오차 기준**이다 — 수학 EE 오차가 0이어도 그리퍼가 박스에서 1m 떨어져
있으면 실패다.

## 좌표 규약: 계산하지 말고 배치된 것을 읽는다

출발지(팔레트)와 도착지(레일)의 위치는 **사용자 의도이지 계산 대상이 아니다.** 좌표로 계산해
스폰하려 들면 메시 피벗, 지지면 트레이스, 로봇 로컬 변환이 곱해져 결과를 예측할 수 없다.

초기 구현은 로봇 로컬 좌표 + 아래 방향 트레이스로 위치를 "찾으려" 했고, 그 결과:

- 로봇 규모를 두 번 오판 (`CreateDefault()` 105cm 기준으로 기본값을 잡았는데 실제는 R3200)
- 팔레트가 엉뚱한 곳에 스폰
- 목표를 최소 반경(dead zone) 안에 배치 — "더 가까이 당기기"가 정확히 반대 처방인 상황

`SourceSurfaceActor` / `DestinationSurfaceActor`로 뒤집으니 이 문제군이 통째로 사라졌다.
`BuildSlotsOnSurface()` **하나가 양쪽을 모두 처리**한다:

```
슬롯 i 월드 = 액터트랜스폼 ∘ (OffsetCm + StrideCm × (i − (N−1)/2))
슬롯 Z     = 액터 바운드 윗면 + HeightAboveSurfaceCm
```

- 행이 액터 중심에 자동 정렬 → 박스 개수를 바꿔도 액터를 옮길 필요 없음
- Stride/Offset이 **액터 로컬** → 액터를 회전시키면 행도 같이 돎
- 상판 높이를 **바운드**에서 읽음 → 메시 피벗 위치와 무관
- Z만 상판 기준으로 덮어씀 → 액터가 기울어져도 박스는 중력 기준 수평

출발지는 `HeightAboveSurfaceCm = 0`(박스 **바닥**이 앉을 높이), 도착지는 `= 박스 높이`(툴이
윗면을 잡으므로). **같은 함수, 다른 인자 하나** — 규약이 하나라 헷갈릴 여지가 없다.

이로써 지지면 트레이스(`TraceSupportSurfaceZ`)와 로봇 로컬 배치 좌표가 전부 제거됐다.

나머지 배치 프로퍼티(`GraspRotation` / `ApproachOffsetCm`)는 **로봇 액터 공간** 기준이다.
모델이 `BaseTransform = Identity`인 액터 로컬 공간에서 FK를 계산하므로 IK 목표를 좌표 변환 없이
넘길 수 있고, 무엇보다 로봇을 레벨 어디로 옮겨도 값이 유효하다.

## 파지 규약: 바운드 윗면 중심

파지 기준점은 `GetActorLocation()`이 아니라 **바운드 윗면 중심**(`GetGraspPointWorld()`)이다.

- **피벗 독립**: 프롭 메시는 피벗이 바닥/구석에 있는 경우가 흔해 "액터 위치 = 기하학적 중심"이
  성립하지 않는다. 그 가정 때문에 박스가 절반 높이만큼 뜬 채 스폰돼 낙하했다.
- **윗면인 이유**: 실제 팔레타이징 그리퍼(진공/클램프)는 박스 윗면에 접촉한다. 중심을 잡으면
  그리퍼가 박스 속에 박힌 것처럼 보인다.

파지 시 `HeldBoxRelativeToTool`(파지 순간의 흡착판 기준 상대 변환)을 보존해 박스를 흡착판으로
순간이동시키지 않는다. 이게 성립하는 전제는 반복 보정이 "흡착판이 박스 윗면에 `MaxReachErrorCm`
이내로 도달"을 보장한다는 것이다. 그 보장이 없으면 이 상대 변환이 곧 "박스가 그리퍼에서 떨어진
거리"로 굳어져 공중에 뜬 채 따라다닌다.

파지 중에는 물리와 **콜리전을 모두 끈다.** 물리만 끄면 박스가 "콜리전 있는 kinematic 바디"가
되어 팔이 휘두를 때 바닥의 다른 박스를 후려친다(실제로 박스가 90cm 밀려났다).

## 고정 타임스텝 + CSV

`DeltaSeconds`를 누적해 `FixedTimeStepSec`(기본 1/120초) 단위로만 FSM을 전진시킨다. 프레임레이트가
흔들려도 궤적 형상과 CSV 샘플 간격이 동일하다. 프레임 hitch 시 따라잡기가 다시 hitch를 유발하는
"죽음의 나선"은 `MaxFixedStepsPerFrame`(8)로 끊고 남은 빚은 버린다 — 시뮬레이션 시간이 실시간보다
느려질 뿐, 스텝 간격은 여전히 정확히 `FixedTimeStepSec`다.

CSV는 `<Project>/Saved/RobotSim/PickPlace.csv`에 31개 컬럼으로 기록한다:

```
time_s, phase, box_index,
q0_deg..q5_deg,            관절각
qd0_degps..qd5_degps,      관절 각속도 (고정 스텝 유한차분)
ee_x/y/z_cm, ee_roll/pitch/yaw_deg,   수학 EE 자세
target_x/y/z_cm,           현재 단계의 시각 그리퍼 목표
box_held,
tau0_nm..tau5_nm           관절 토크 — B-02 RNEA, **qdd=0 준정적 근사** (D-01에서 추가)
```

> `tau*` 6개는 D-01이 **끝에 덧붙인** 컬럼이다(25 → 31). 중간에 끼우면 기존 컬럼 인덱스가 밀려서
> 이미 만든 분석 스크립트가 조용히 다른 열을 읽는다. 한계는 아래 `## 한계` 1번 참조.

**모든 값을 `ActiveState`에서 직접 계산한다** — 로봇/메시 컴포넌트를 읽지 않는다. 위의 틱 순서
표 때문이다. 각속도는 스텝 간격이 항상 일정하므로 프레임레이트에 오염되지 않은 물리값이 된다.

수학 EE와 목표의 "위치 오차"는 **의도적으로 기록하지 않는다.** 목표는 시각 그리퍼가 갈 지점이고
수학 EE는 거기서 visual calibration offset만큼 떨어져 있는 것이 정상이라(`ToPick`에서 100cm 넘게
나온다), 그 이름으로 남기면 IK가 고장난 것처럼 읽힌다. 도달 실패는 `Aborted` 로그가 말한다.

## FSM

```
Idle → ToPickApproach → ToPick → Grasp → ToLift
     → ToPlaceApproach → ToPlace → Release → ToRetreat
     → (다음 박스면 ToPickApproach) → Done
                                    ↘ Aborted (도달 불가)
```

- `To*`: 관절공간 궤적 추종. `Grasp`/`Release`: `DwellSec` 정지 (그리퍼 개폐 시간의 대역).
- 접근/이탈 단계는 `ApproachOffsetCm`만큼 위 — 이 수직 진입이 박스 옆면 간섭을 피하는 유일한
  수단이다(충돌 회피 경로계획이 범위 밖이므로).
- `Grasp` 진입 시 부착, `Release` **진입 시** 해제 — 툴이 정지한 채로 박스가 `DwellSec` 동안
  자리를 잡은 뒤 팔이 빠지도록.

### 궤적: quintic smoothstep

IK는 단계 진입 시 **한 번만** 푼다(반복 보정 포함). 매 프레임 풀면 singularity 근처에서 해가
프레임마다 튀어 고정 타임스텝의 결정론이 무의미해진다.

그 해와 시작 자세 사이를 `S(u) = 6u⁵ − 15u⁴ + 10u³`로 보간한다. `S'(0)=S'(1)=0`,
`S''(0)=S''(1)=0`이라 시작/끝에서 속도와 가속도가 모두 0이다. cubic과 달리 가속도까지 연속이라
이후 토크 제어에서 지령 토크에 계단 불연속이 생기지 않는다.

소요시간은 속도 한계에서 역산하되 **피크 기준**이다: smoothstep은 중간에서 평균의 **15/8배**
속도로 지나가므로, 평균으로 역산하면 한계를 넘는다.

> 이는 B-04/B-05(저크 제한 S-curve + 6관절 동기화)의 **자리를 채우는 임시 구현**이다.
> 시간최적이 아니고 저크 제한도 없다.

## 진단 도구

배치 실패의 원인을 추측하지 않기 위해 로그가 숫자로 말하게 했다:

- **`시각 그리퍼 도달 XY반경 탐색`** — 파지 높이/접근 높이/도착지 높이 3곳에서 도달 가능한
  XY반경 구간을 IK로 훑어 보고. 대형 6축 팔은 **최소 반경(dead zone)** 이 커서 베이스에 너무
  가까운 지점도 도달 불가인데, 이 구간을 모르면 목표를 반대 방향으로 옮기는 실수를 한다.
  단위는 **XY 평면 반경**이며 `Aborted` 메시지도 같은 양을 찍는다(3D 거리와 섞으면 비교가 성립하지 않는다).
- **`[기구학]` 덤프** — 홈 자세 EE, 툴 축 방향, `ToolOffset`, J0~J5 축/오프셋/한계.
  `DA_RobotConfig`가 바이너리라 소스만 읽어서는 규모도 툴 축 방향도 알 수 없다.
- **`Log Robot Skeleton Geometry`** — 레퍼런스 스켈레톤 본 치수 덤프.
- **`Log Visual Tool Alignment`** — 현재 자세의 visual calibration offset.
- **스폰 간격 검사** — `SourceSlotStrideCm`이 박스 폭보다 좁으면 필요 최소 간격을 계산해 경고.
  겹치면 물리가 밀어내 파지 지점 스냅샷이 어긋나는데, 겉보기엔 "IK가 이상하다"로 보인다.
- **파지 간격 검사** — 흡착판과 박스가 벌어지면 "박스가 움직였다"고 원인을 특정해 경고.

### 실패는 조용히 진행하지 않는다

`MaxReachErrorCm`(기본 3cm)를 넘으면 `Aborted`로 중단한다. 초기 구현은 "최선해로 진행"했는데,
그러면 툴이 목표에서 1m 떨어진 채 박스를 든 것처럼 움직여서 **배치 문제가 파지 버그로 위장된다.**
멈추고 숫자를 말하는 편이 낫다.

## 변경/추가 파일

**신규**
- `Public/Robot/PickPlaceTaskActor.h`, `Private/Robot/PickPlaceTaskActor.cpp`
  — `APickPlaceBoxActor`(박스) + `APickPlaceTaskActor`(FSM) 두 UCLASS
- `Docs/Steps/STEP_C-01.md` (이 문서)

**수정 — 시각 레이어 한정**
- `Public/Robot/RobotConfig.h` — `VisualGraspBoneName` 추가
- `Public/Robot/Serial6DoFRobotActor.h`, `Private/Robot/Serial6DoFRobotActor.cpp`
  — `VisualGraspPoint` 컴포넌트, `GetVisualGraspPointWorld()`, `AttachVisualGraspPointToBone()`,
    `LogVisualToolAlignment()`

**수정하지 않음**
- `FSerial6DoFModel` / `FRobotPoseError` / `FRobotJacobian` / `FRobotDlsIK` / RNEA 및 테스트 32종
- `ToModel()`이 시각 정합 필드를 복사하지 않으므로 수학 모델은 그 존재조차 모른다.

### `GetVisualGraspPointWorld()`는 본을 직접 읽는다

`VisualGraspPoint->GetComponentTransform()`에 의존하지 않고
`GetBoneTransformByName(..., WorldSpace)`로 본을 직접 읽은 뒤 에디터에서 맞춘 상대 오프셋을 얹는다.
부착된 자식 컴포넌트의 월드 트랜스폼 전파 시점이 `SyncSkeletalPoseToMath` 호출 경로와 어긋날 수
있고, 한 프레임이라도 밀리면 박스가 그리퍼와 따로 논다(강체 결합인데도). 본 트랜스폼은 메시가
화면에 제대로 그려지는 이상 확실히 최신이므로 타이밍에 무관하게 항상 옳다.

## 한계

1. **토크를 기록하지만 토크로 구동하지는 않는다.** D-01에서 B-02의 RNEA를 연결해 CSV의
   `tau0_nm`~`tau5_nm` 컬럼과 UI 토크 게이지에 관절 토크를 기록한다. 두 가지 단서가 붙는다:
   (a) **qdd=0 준정적 근사다** — 현 구동은 위치 지령 + 관절공간 보간이라 관절 가속도가 동역학
   적분의 결과가 아니므로 궤적의 2차 미분을 넣어도 "실제 구동 토크"라는 의미가 없다. 중력·마찰
   항은 살아 있고 관성 M(q)q̈와 코리올리 C(q,q̇)q̇는 빠진다. (b) **이 토크는 실제 구동에 관여하지
   않는다** — 구동은 여전히 `SetJointAngles` 기구학이고, 토크는 계산된 결과일 뿐 원인이 아니다.
   토크가 원인이 되려면 B-06의 computed torque 제어가 들어와야 한다.
2. **visual calibration offset이 남아 있다.** 수학 EE와 시각 그리퍼가 자세마다 어긋난다. 반복
   보정이 파지/적재 지점에서만 이를 흡수하므로, **궤적 중간 구간에서는 그리퍼가 수학 EE 궤적을
   따르지 않는다.** 근본 해결은 `LinkOffsets`를 KUKA 실측값으로 재작성하는 것이고, 그건 FK/IK
   전면 재검증을 수반한다.
3. **grasp이 해석적이지 않다.** 물리/콜리전을 끄고 트랜스폼으로 붙이는 kinematic 방식이다.
   마찰 원뿔 미끄러짐 판정(B-07)이 들어오면 이 토글을 그 판정 결과로 대체해야 한다.
4. **팔과 박스가 서로 통과한다.** 로봇 링크 메시가 `NoCollision`이라 충돌하지 않는다. 수직 진입
   (`ApproachOffsetCm`)으로 시각적 간섭만 줄인다.
5. **단층 1열 적재.** N층 파라메트릭 적재는 범위 밖이다.
6. **궤적이 시간최적이 아니다.** quintic smoothstep + 속도 한계 역산이며 저크 제한이 없다(B-04/B-05).
7. **도달 반경 탐색이 비싸다.** 반경 하나마다 IK를 최대 12회 + 메시 동기화를 돌리므로 PIE 시작이
   잠깐 멈춘다. 배치 확정 후 `bLogWorkspaceProbe`를 끄면 된다.

## 에디터 설정

1. `DA_RobotConfig` → `VisualGraspBoneName`에 흡착판이 달린 본 이름
2. 로봇 액터 → 컴포넌트 트리에서 `VisualGraspPoint` 선택 → 흡착면 끝에 오도록 Relative Location 조정
3. 로봇 액터 → **`bDrawDebugFrames` 해제, `PoseLogIntervalSeconds` = 0** (틱 순서 때문에 홈 자세로 그려진다)
4. 팔레트/레일을 레벨에 배치 → `SourceSurfaceActor` / `DestinationSurfaceActor`에 할당
5. `BoxClass`에 `APickPlaceBoxActor` 상속 BP (메시/크기는 BP에서 자유롭게)
6. 데모 중 Target 드래그 금지 — `SolveIKToTarget`이 관절을 덮어쓰려 해 IK 로그가 오염된다

두 표면 모두 로봇 도달 반경 안이어야 한다. `Aborted`가 뜨면 로그의 XY반경 구간과 슬롯별 XY반경을
대조해 액터를 옮기면 된다.

## 다음 단계

- **B-01 보강**: `FRobotJointLimit`에 `MaxAccelRadPerSec2` / `MaxJerkRadPerSec3` 추가.
  B-04의 S-curve가 이 값을 입력으로 요구하는데 현재 없다.
- **B-03**: 질량행렬 M(q) + 순동역학
- **B-04/B-05**: 저크 제한 S-curve → 6관절 동기화 궤적 (여기 quintic smoothstep을 대체)
- **B-06**: computed torque 제어 (이 액터의 고정 타임스텝 루프와 CSV를 재사용)
- **B-07**: 마찰 원뿔 grasp 판정 (`APickPlaceBoxActor`의 물리 토글을 대체)
