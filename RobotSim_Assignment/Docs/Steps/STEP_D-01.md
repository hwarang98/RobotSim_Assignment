# Step D-01: UMG 바인딩 표면 (FSM 상태·관절각·속도·RNEA 토크 게이지·멀티로봇 목록)

## 목표

과제의 **STEP D "UMG 제어 UI — 사용성"** 을 채운다. 그리고 그 과정에서 B-02의 부채를 갚는다.

B-02는 스스로 이렇게 끝났다 — "**아직 어디에도 연결되지 않았다.** 액터/런타임 경로가 RNEA를
호출하지 않으므로 실제 시뮬레이션 거동은 B-01과 동일하다." 정확하게 구현하고 8종의 테스트로
검증했지만, **아무도 부르지 않는 함수는 데모에서 존재하지 않는다.** 이 단계가 그 함수를 부른다:
고정 스텝마다 토크를 계산해 CSV 컬럼과 UI 게이지로 내보낸다.

C++은 **바인딩 표면만** 소유한다. Widget Blueprint 에셋은 만들지 않는다 (아래 근거 참조).

## 범위 밖 (이번 단계에서 구현하지 않음)

3D 기즈모, undo/redo, UMG 위젯 에셋 자동 생성, RNEA 기반 computed torque 제어, 순동역학,
S-curve 신규 구현, 마찰 원뿔 grasp 판정, 모션 **로드**, PID 제어.

**수정하지 않은 것** (회귀 위험 0의 근거):
`FSerial6DoFModel` / `FRobotPoseError` / `FRobotJacobian` / `FRobotDlsIK` / `RobotDynamicsRNEA`의
수학은 **한 줄도 건드리지 않았다.** `ASerial6DoFRobotActor`도 마찬가지다 — D-01은 그 액터의
public API(`GetModel` / `GetJointState`)를 **읽기만** 한다. 기존 자동화 테스트 40종은 그대로다.
`FSerial6DoFModel`의 axis/offset/link length/limit도 변경하지 않았다.

변경은 `APickPlaceTaskActor`, `APickPlaceDispatcher`, 문서 3개에만 있다.

## 과제 Step D 요구사항 대응표

| 요구사항 | 이번 구현 | 상태 |
|---|---|---|
| UMG 제어 UI | C++ 바인딩 표면 + 에디터 제작 가이드 | ✅ |
| MVVM 패턴 | 액터가 ViewModel 역할 (BlueprintPure 표면). **UMG Viewmodel 플러그인은 안 씀** | △ |
| 관절각 HUD 실시간 표시 | `GetJointAnglesDeg` | ✅ |
| 토크 HUD 실시간 표시 | `GetJointTorqueNm` / `GetJointTorqueRatio` (B-02 RNEA) | ✅ |
| 속도 HUD 실시간 표시 | `GetJointVelocityDegPerSec` (고정 스텝 유한차분) | ✅ |
| 프레임타임 HUD 실시간 표시 | `GetFrameTimeMs` (EMA 평활) | ✅ |
| 재생 | `StartAllRobots` | ✅ |
| 정지 | `PauseAllRobots(true/false)` | ✅ |
| 리셋 | `ResetAllRobots` (박스 재스폰까지) | ✅ |
| 모션 저장 | `SaveMotionCsvNow` / `FlushAllCsvNow` → CSV | ✅ |
| 모션 로드 | 없음 — trajectory 로드 시스템이 없다 | ❌ |
| 런타임 속도 프로파일 튜닝 | `SetVelocityScale` / `SetDwellSec` | ✅ |
| 런타임 PID 튜닝 | 없음 — PID 구동 자체가 없다 | ❌ |
| 3D 기즈모 (가산점) | 범위 밖 | ❌ |
| undo/redo (가산점) | 범위 밖 | ❌ |

**❌ 두 개(모션 로드 / PID)를 만들지 않은 것은 시간 부족이 아니라 판단이다.** 아래 `## 한계` 3·4번 참조.

## 왜 Widget Blueprint를 C++로 만들지 않았는가

| 방안 | 결과 | 채택 |
|---|---|---|
| C++에서 `UUserWidget` 서브클래스 + 위젯 트리 코드 생성 | 레이아웃을 코드로 짜야 한다. 스타일 하나 바꾸려면 재컴파일. 마감일에 컴파일 실패 = 데모 전멸 | ❌ |
| C++은 바인딩 표면만, 위젯은 에디터에서 | 배치/스타일링이 뷰포트에서 즉시. C++ 실패해도 기존 데모는 산다 | ✅ |

에디터가 **훨씬 잘하는 일**을 코드로 재구현할 이유가 없다. UMG는 원래 디자이너 도구이고, C++이
잘하는 것은 "무엇을 보여줄 것인가"(데이터)이지 "어떻게 배치할 것인가"(레이아웃)가 아니다.
경계를 그 선에 맞췄다: **C++은 ViewModel, Widget BP는 View.**

## RNEA를 데모에 연결하는 지점

이 단계의 핵심 주장이다. B-02는 정확했지만 죽어 있었다. 살리는 접점은 정확히 두 곳이다:

1. **CSV `tau0_nm`~`tau5_nm` 컬럼** — 재현 가능한 물리값 로그
2. **UI 토크 게이지** (`GetJointTorqueRatio`) — 화면에서 즉시 보이는 값

```
StepFixed()
  ├─ EvaluateTrajectory()      ActiveState 갱신
  ├─ UpdateTelemetryCache()    qd 유한차분 → SolveInverseDynamicsRNEA → LastJointTorqueNm  ← RNEA 호출
  └─ RecordCsvRow()            LastJointTorqueNm 읽기 (재계산 없음)
                                      ↑
                    GetJointTorqueNm() / GetJointTorqueRatio()  ← UMG 바인딩도 같은 캐시
```

**계산은 한 곳, 소비는 두 곳.** 이 구조가 아니면 "화면의 게이지"와 "CSV의 토크"가 같은 스텝의 같은
수라는 보장이 없다.

### qdd = 0 준정적 근사의 의미와 한계

토크 계산에 **관절 가속도를 0으로 넣는다. 의도적이다.**

현재 구동은 위치 지령 + 관절공간 보간(quintic smoothstep)이다. 즉 **관절 가속도가 동역학 적분의
결과가 아니다** — 궤적이 먼저 있고 팔이 그대로 따라갈 뿐이다. 이 상황에서 궤적의 2차 미분을 qdd로
넣으면 숫자는 나오지만 "이 팔을 이렇게 움직이는 데 실제로 필요한 토크"라는 물리적 의미가 없다.
정직한 이름은 **준정적(quasi-static) 토크 추정**이다.

| 항 | 상태 | 이유 |
|---|---|---|
| 중력 g(q) | ✅ 살아 있다 | 자세만의 함수다. 이 근사에서 완전히 정확하다 |
| 마찰 (점성 + 쿨롱) | ✅ 살아 있다 | qd가 유한차분으로 실제 값이다 |
| 관성 M(q)·q̈ | ❌ 빠진다 | q̈ = 0으로 뒀다 |
| 코리올리/원심 C(q,q̇)·q̇ | ❌ 빠진다 | 강체 재귀에는 있지만 q̈=0에서 지배항이 사라진다 |
| 로터 관성 | ❌ 꺼 둠 | q̈에만 곱해지므로 q̈=0에서 **아무 효과가 없다.** 켜면 거짓 정밀도만 생긴다 |

즉 **가속 구간의 토크는 과소평가된다.** 팔레타이징처럼 느린 동작(`VelocityScale` 기본 0.35)에서는
중력항이 지배적이라 실용적으로 쓸 만하지만, 이것이 "실제 구동 토크"라고 주장하지는 않는다.
온전한 토크는 B-06의 computed torque 제어가 들어와야 나온다.

### 왜 BlueprintPure getter가 RNEA를 직접 돌리면 안 되는가

**BlueprintPure는 캐싱되지 않는다.** 위젯 바인딩에 물리면 그 값을 쓰는 노드마다, 프레임마다 다시
실행된다. 토크 게이지 6개 + 숫자 텍스트 6개를 한 패널에 놓고 로봇이 2대면 `GetJointTorqueNm`이
프레임당 24번 불린다. 각각이 RNEA를 돌리면 프레임당 RNEA 24회다 — 그리고 그건 **틱 순서에 따라
서로 다른 관절 상태를 읽어서 값이 미세하게 갈라지기까지 한다.**

그래서 규약을 하나로 못 박았다: **모든 getter는 캐시만 반환한다.** 계산은 `UpdateTelemetryCache()`가
고정 스텝마다 한 번 한다. `PickPlaceTaskActor.h`의 `#pragma region UIBinding` 주석에 이 불변식을
명시해 뒀다.

### 왜 Idle에서 0이 아니라 중력 토크를 계산하는가

dispatcher 모드의 `Idle`은 "정지"가 아니라 **"다음 배급을 기다리며 자세를 유지"** 다 (일시정지도
마찬가지다). 팔은 실제로 중력을 버티고 있고, 그 토크는 0이 아니다. 게이지를 0으로 그리면 화면이
거짓말을 한다.

그래서 FSM이 전진하지 않는 프레임에서는 B-02의 **다른 함수** `ComputeGravityTorque(Model, q)`를
Tick당 한 번 부른다. qd=qdd=0이므로 마찰·관성 항이 자연히 사라져 순수 중력 보상 토크 g(q)만 남는다.
결과적으로 B-02의 두 공개 함수가 **둘 다** 데모에 연결된다.

## 노출한 바인딩 표면

### `APickPlaceTaskActor` — 로봇 한 대의 ViewModel

| 함수 | 종류 | 반환/의미 |
|---|---|---|
| `GetRobotDisplayText` | Pure | 로봇 이름 (없으면 태스크 액터 이름) |
| `GetPhaseDisplayText` | Pure | FSM 단계 — **로그/CSV와 같은 문자열** |
| `GetPhaseProgress` | Pure | 0~1 (ProgressBar) |
| `GetJointAnglesDeg` | Pure | `TArray<float>` ×6 (도) |
| `GetJointVelocityDegPerSec` | Pure | `TArray<float>` ×6 (도/초) — CSV `qd*`와 동일 캐시 |
| `GetJointTorqueNm` | Pure | `TArray<float>` ×6 (N·m) — CSV `tau*`와 동일 캐시 |
| `GetJointTorqueRatio` | Pure | `TArray<float>` ×6, `\|τ\|/MaxTorqueNm` 0~1 clamp (게이지) |
| `GetToolLocationWorld` | Pure | 툴 팁 월드 위치 (cm) |
| `GetLastReachErrorCm` | Pure | 마지막 IK의 시각 파지점 오차 |
| `GetCompletedBoxCount` | Pure | 완료 박스 수 |
| `GetFrameTimeMs` | Pure | 프레임 시간 (EMA 평활) |
| `IsCycleRunning` / `IsPaused` | Pure | 상태 |
| `SetPaused` | Callable | 일시정지/재개 |
| `GetVelocityScale` / `SetVelocityScale` | Pure / Callable | 속도 프로파일 (0.01~1 clamp) |
| `GetDwellSec` / `SetDwellSec` | Pure / Callable | 대기 시간 (0~5초 clamp) |
| `SaveMotionCsvNow` | Callable | 모션 저장 (CSV) |
| `StartCycle` / `ResetCycle` / `FlushCsvNow` | Callable | (기존, 이미 BlueprintCallable) |

**`TArray<float>`인 이유**: Blueprint는 `double`과 C 고정 배열(`double[6]`)을 다루지 못한다.
`FRobot6DJointState`류는 리플렉션이 없는 순수 데이터(USTRUCT 아님)라는 기존 규약을 유지하면서,
**UI 경계에서만** 복사 + 단위 변환(rad→deg) + 정밀도 축소(double→float)를 한다. 내부 상태는 double 그대로다.

### `APickPlaceDispatcher` — 멀티로봇 대시보드의 진입점

| 함수 | 종류 | 반환/의미 |
|---|---|---|
| `GetTaskActors` | Pure | 로봇 목록. **순서 = 배급 우선순위** |
| `GetUnassignedBoxCount` | Pure | 미배급 박스 수 |
| `GetFreeSlotCount` | Pure | 빈 도착지 슬롯 수 |
| `GetAssignmentSummary` | Pure | 여러 줄 배급 현황 |
| `StartAllRobots` | Callable | 전체 시작 |
| `PauseAllRobots(bool)` | Callable | 전체 일시정지 — **배급도 함께 멈춘다** |
| `ResetAllRobots` | Callable | 전체 재초기화 (박스 재스폰) |
| `FlushAllCsvNow` | Callable | 전체 CSV + Dispatch.csv 기록 |

`GetUnassignedBoxCount`와 `GetFreeSlotCount`를 나란히 노출한 이유: C-02가 겪은 **조용한 정지**를
화면에서 즉시 진단하기 위해서다. "미배급 박스는 남았는데 빈 슬롯이 0"이면 병목이 슬롯이라는 뜻이고,
C-02에서는 그걸 도달성 캐시 숫자에서 역산해야 했다.

## 곁가지로 고친 것: CSV flush가 데이터를 지우고 있었다

`Save CSV` 버튼(`FlushAllCsvNow`)을 배선하다 발견한 **기존 버그**다.

`WriteCsvToDisk()`는 파일을 쓴 뒤 `CsvRows.SetNum(1)`로 버퍼를 헤더만 남기고 잘랐다. 주석은
"재기록 시 중복 append를 막는다"였는데, **전제가 틀렸다** — `FFileHelper::SaveStringArrayToFile`은
기본 WriteFlags가 `FILEWRITE_None`이라 **append가 아니라 덮어쓰기**다.

```
flush 1회차: 파일 = [헤더, 1~100행]     버퍼 → [헤더]
flush 2회차: 버퍼 = [헤더, 101~200행]
             파일 = [헤더, 101~200행]   ← 1~100행이 사라졌다
```

지금까지 드러나지 않은 이유는 flush가 사실상 사이클당 한 번(Done/EndPlay)만 일어났기 때문이다.
D-01이 **버튼**을 달면서 "사이클 도중에 두 번 이상 flush"가 평범한 조작이 됐고, 그 순간 이 버그는
클릭 한 번에 터진다 — 그리고 로그도 에러도 없이 조용히 터진다. 이 프로젝트가 가장 싫어하는 종류의
실패다.

**고친 방법**: 버퍼를 자르지 않는다. 매번 전체를 덮어쓰므로 몇 번을 눌러도 파일은 항상 처음부터
끝까지 온전하다. 사이클 하나가 120Hz × 수십 초 = 수천 행이라 전량 보관해도 메모리는 무시할 만하다.
`APickPlaceDispatcher::WriteDispatchCsvToDisk()`도 같은 버그·같은 수정을 했다 — 그쪽은 "할당이
실제로 일어났다는 증거"가 용도라 앞부분이 사라지면 파일의 존재 이유 자체가 무너진다.

## 선택 상태를 C++가 아니라 Widget BP에 둔 이유

"지금 어느 로봇을 보고 있는가"는 **UI 정책이지 시뮬레이션 상태가 아니다.** dispatcher에 `SelectedRobot`을
두면 배급자가 동시에 UI 상태 저장소가 되어 두 책임이 섞이고, 그 순간 "위젯을 안 띄우면 선택이 뭐지?"
같은 무의미한 질문이 생긴다. 헤드리스 실행에서도 dispatcher는 그대로 돌아야 한다.

그래서 C++은 **목록만** 준다(`GetTaskActors`). 패널 생성·선택·강조는 위젯이 한다.

`GetTaskActors`의 순서가 `SortTaskActorsDeterministically()`의 결과(RobotPriority → 액터 이름)와
같다는 것이 중요하다. UI 순서와 실제 배급 우선순위가 다르면 "위에 있는 로봇이 왜 나중에 받지?"라는
혼란이 생기고, 그건 C-02가 힘들게 확보한 결정론을 **사용자 눈에는 비결정론으로 보이게** 만든다.

## 테스트를 만들지 않은 근거

C-02의 선례를 따른다.

**신규 수학이 없다.** 이 단계는 알고리즘을 만들지 않았다 — 이미 있는 상태(`ActiveState`, `Phase`,
`BoxTaken`)와 이미 검증된 함수(`SolveInverseDynamicsRNEA`, B-02의 테스트 8종)를 UI에 여는 배선 작업이다.
`GetJointAnglesDeg`가 `RadiansToDegrees(ActiveState.Q[i])`를 반환하는지 검증하는 테스트는
구현을 그대로 다시 쓰는 동어반복이지 검증이 아니다.

**진짜 위험은 자동화 테스트가 잡을 수 없는 곳에 있다.** 위젯 바인딩이 붙었는가, 게이지가 채워지는가,
Pause 중에 팔이 튕기지 않는가 — 전부 PIE에서 눈으로 봐야 하는 것이다. `HowToTest.md`가 스스로
"액터도 월드도 스폰하지 않는 순수 계산 테스트"라고 선언한 문서인 것도 같은 이유다.

**기존 40종은 그대로 통과해야 한다.** 수학 레이어를 건드리지 않았으므로 하나라도 깨지면 이 작업이
선을 넘은 것이고, 그게 이 단계의 회귀 판정 기준이다.

### 검증 절차

1. **빌드** — UHT가 `TArray<float>` / `FText` 반환 UFUNCTION 서명을 통과시키는지.
2. **자동화 테스트 40종** (`RobotSim.*`) 전원 통과 — 회귀 없음의 증거. `HowToTest.md` 참조.
3. **CSV** — `Saved/RobotSim/PickPlace_*.csv`의 헤더가 **31컬럼**이고 `tau0_nm`~`tau5_nm`가 있는지.
   - **J0(τ₀)는 항상 0에 가까워야 한다.** yaw축이 중력과 평행이라 중력 토크가 구조적으로 0이라는
     B-02의 불변식이다. 0이 아니면 프레임 규약이 깨진 것이다.
   - J1이 자세에 따라 수십~수백 N·m 대역에서 움직이는지 (B-02 실측: 최악 자세 −303.4 N·m).
   - 100배 어긋나면 cm↔m 경계 버그, 부호가 뒤집혔으면 중력 방향 버그 (`HowToTest.md`의 로그 읽기 절).
4. **배급 현황** — 위젯을 만들기 전에 Level BP에서 `GetAssignmentSummary`를 `Print String`으로 찍어
   로봇 2대에 **서로 다른** 박스가 동시에 배급되는지 확인한다. 위젯 배선 실패와 배급 실패를 분리해
   진단하기 위해서다.
5. **Pause 회귀** — 일시정지 중 (a) 팔이 홈 자세로 튕기지 않는지, (b) 잡은 박스가 그리퍼에 붙어
   있는지, (c) 재개 시 `MaxFixedStepsPerFrame` 경고가 **안 뜨는지**. (c)가 누적기 처리가 맞았다는 증거다.

## 에디터 설정

C++은 여기까지다. 아래는 에디터에서 한다.

**1. `WBP_RobotDashboard` 생성** — Content Browser → User Interface → Widget Blueprint.

**2. Dispatcher 참조 변수 추가** — 타입 `PickPlaceDispatcher`(Object Reference), **Instance Editable** +
**Expose on Spawn** 체크.

**3. 로봇 패널 반복 생성** — `GetTaskActors()` → `ForEachLoop` → 각 원소로 로봇 패널을 만든다.
패널을 별도 위젯(`WBP_RobotPanel`, `PickPlaceTaskActor` 참조 변수 보유)으로 빼면 바인딩이 훨씬 깔끔하다.
`Vertical Box` + `Create Widget` + `Add Child`.

**4. 각 로봇 패널 바인딩** (전부 태스크 액터 참조 → 아래 함수):

| 위젯 | 바인딩 |
|---|---|
| 이름 (TextBlock) | `GetRobotDisplayText` |
| 상태 (TextBlock) | `GetPhaseDisplayText` |
| 진행률 (ProgressBar) | `GetPhaseProgress` |
| 관절각 ×6 (TextBlock) | `GetJointAnglesDeg` → `Get (index)` → `Format Text` |
| 관절속도 ×6 (TextBlock) | `GetJointVelocityDegPerSec` |
| 토크 ×6 (TextBlock) | `GetJointTorqueNm` |
| **토크 게이지 ×6 (ProgressBar)** | `GetJointTorqueRatio` → `Get (index)` |
| EE 위치 (TextBlock) | `GetToolLocationWorld` |
| 도달 오차 (TextBlock) | `GetLastReachErrorCm` |
| 완료 수 (TextBlock) | `GetCompletedBoxCount` |
| 프레임타임 (TextBlock) | `GetFrameTimeMs` |

> 배열 getter는 `Get (a copy)` 노드로 인덱스를 꺼낸다. **바인딩마다 배열이 새로 만들어지므로**
> 관절 6개를 각각 바인딩하면 프레임당 6번 복사된다 — 6원소 배열이라 무시할 만하지만, 더 깔끔하게
> 하려면 `Event Tick`에서 한 번 받아 `Set Percent`/`Set Text`로 밀어넣으면 된다.

**5. 버튼 연결** (전부 Dispatcher 참조 → 아래 함수):

| 버튼 | OnClicked |
|---|---|
| Start | `StartAllRobots` |
| Pause | `PauseAllRobots(true)` |
| Resume | `PauseAllRobots(false)` |
| Reset | `ResetAllRobots` |
| Save CSV | `FlushAllCsvNow` |

Pause/Resume을 한 버튼으로 토글하려면 `IsPaused`를 태스크 액터에서 읽어 `Not` → `PauseAllRobots`.

**6. 튜닝 슬라이더** — 각 로봇 패널에:

| 슬라이더 | 범위 | OnValueChanged |
|---|---|---|
| VelocityScale | 0.01 ~ 1.0 | `SetVelocityScale` |
| DwellSec | 0.0 ~ 5.0 | `SetDwellSec` |

초기값은 `GetVelocityScale` / `GetDwellSec`로 `Event Construct`에서 채운다. C++이 어차피 clamp하므로
슬라이더 범위가 틀려도 안전하다.

**7. HUD에 붙이기** — Level Blueprint의 `Event BeginPlay`에서:

```
Create Widget (Class = WBP_RobotDashboard)
  → Dispatcher 핀에 레벨의 BP_Dispatcher 참조 연결   ← Expose on Spawn이면 여기 핀이 생긴다
  → Add to Viewport
```

Dispatcher 참조를 빠뜨리면 위젯이 전부 기본값(0)을 그린다 — 바인딩 버그처럼 보이지만 참조가 없는 것이다.

**8. CSV 확인** — `<Project>/Saved/RobotSim/`의 `PickPlace_*.csv`를 열어 `tau0_nm`~`tau5_nm` 컬럼 확인
(위 `### 검증 절차` 3번의 판정 기준 적용).

## 변경/추가 파일

**수정**
- `Source/RobotSim_Assignment/Public/Robot/PickPlaceTaskActor.h` — `UIBinding` / `Telemetry` region 추가
- `Source/RobotSim_Assignment/Private/Robot/PickPlaceTaskActor.cpp` — 텔레메트리 캐시, 토크 컬럼, Pause
- `Source/RobotSim_Assignment/Public/Robot/PickPlaceDispatcher.h` — `UIBinding` region 추가
- `Source/RobotSim_Assignment/Private/Robot/PickPlaceDispatcher.cpp` — UI getter, 전체 제어, Pause
- `Docs/Steps/STEP_C-01.md` — "토크가 없다" 한계와 CSV 컬럼 목록 갱신 (25 → 31)
- `Docs/Steps/STEP_B-02.md` — "아직 어디에도 연결되지 않았다" 한계에 D-01 해소 주석 추가
  (그대로 두면 제출 문서 안에서 B-02와 D-01이 서로 모순된다)

**신규**
- `Docs/Steps/STEP_D-01.md` (본 문서)

**수정하지 않음**
- `Serial6DoFModel` / `RobotPoseError` / `RobotJacobian` / `RobotDlsIK` / `RobotDynamicsRNEA` — 수학 전부
- `Serial6DoFRobotActor` — 읽기만 한다
- 기존 테스트 40종

## 한계

1. **엄밀한 의미의 MVVM이 아니다.** UMG Viewmodel 플러그인(`UMVVMViewModelBase` + 필드 변경 통지)을
   쓰지 않고, 액터 자체를 `BlueprintPure` 표면으로 노출해 위젯이 **폴링**하게 했다. 즉 데이터가
   "변경을 알리는" 것이 아니라 위젯이 "매 프레임 물어본다". 시뮬레이션 값이 고정 스텝마다 어차피
   전부 바뀌는 성질이라 실용적 손해는 거의 없지만, 플러그인 의존성과 마감 리스크를 지지 않기로 한
   **의도적 타협**이다. View/ViewModel 분리라는 패턴의 목적 자체는 지켰다 — 위젯은 로직을 모른다.
2. **토크는 qdd=0 준정적 추정이며 구동에 관여하지 않는다.** 중력·마찰만 살아 있고 관성·코리올리는
   빠진다. 가속 구간에서 과소평가된다. 그리고 계산될 뿐 **적용되지 않는다** — 구동은 여전히
   `SetJointAngles` 기구학이다 → **B-06**.
3. **PID 튜닝 슬롯이 비어 있다.** computed torque/PID 구동 자체가 없으므로 슬라이더를 달면 아무것도
   하지 않는 **가짜 컨트롤**이 된다. 평가자가 만졌을 때 반응이 없는 UI는 "미구현"보다 나쁘다 —
   구현했다고 주장하면서 거짓말을 하기 때문이다. 만들지 않는 편이 정직하다 → **B-06**.
4. ~~**모션 로드가 없다.** 저장(CSV)만 있다.~~ → **D-02에서 해소.** CSV를 읽어 관절 궤적을 고정
   타임스텝으로 재생한다(`PlayReplay`). 기록↔재생이 대칭이 되어 결정론적 재현이 눈으로 증명된다.
   단, 재생 대상은 **팔 관절각만**이고 박스는 제외한다 — CSV가 어느 박스가 어디 있었는지를 담지
   못하므로(STEP_D-02.md 참조).
5. **프레임타임은 EMA 평활값이다** (α=0.1). 원시값은 진동이 심해 HUD에서 숫자를 읽을 수 없다.
   평활의 대가로 **스파이크가 과소평가된다** — 정확한 프레임 분석은 `stat unit`을 쓸 것.
6. **`ResetAllRobots`가 한 번 스톨한다.** 도달성 캐시를 재계산하므로(DLS 최대 288회) 버튼을 누르는
   순간 프레임이 멈춘다. PIE 시작 시의 스톨과 같은 비용이고 사용자가 명시적으로 누른 시점이지만,
   데모 녹화 중에 누르면 그대로 찍힌다.
7. **토크 게이지가 1.0에 붙박일 수 있다.** `MaxTorqueNm` 기본값이 100 N·m인데 B-02의 실측은 J1에서
   −303.4 N·m였다. 그 경우 비율이 1.0에 고정되는데, **이건 게이지 버그가 아니라 `MaxTorqueNm`가
   실제로 부족하다는 신호다** (B-02의 "권장 MaxTorqueNm 산정 방식" 절이 758 N·m를 제안한다).
   DA_RobotConfig에서 관절별로 실측 기반 값을 넣으면 게이지가 의미 있는 범위로 움직인다.
8. **`GetTaskActors` 순서는 초기화 이후에만 확정이다.** dispatcher가 첫 Tick에 정렬하므로, 그 전에
   위젯이 목록을 읽으면 등록 순서(비결정론)가 나온다. `Event Construct`가 아니라 `Event Tick`이나
   버튼 클릭에서 읽으면 문제없다.

## 다음 단계

- **B-01 보강**: `FRobotJointLimit`에 `MaxAccelRadPerSec2` / `MaxJerkRadPerSec3` 추가.
  그리고 `MaxTorqueNm`를 B-02의 실측 기반 권장값으로 갱신 (위 한계 7번 — 게이지가 그때 의미를 갖는다).
- **B-03**: 질량행렬 M(q) + 순동역학
- **B-04/B-05**: 저크 제한 S-curve → 6관절 동기화 궤적
- **B-06**: computed torque 제어 — **이 단계의 토크 캐시가 그대로 feedforward 항이 된다.**
  그때 qdd=0 근사가 사라지고(한계 2), PID 슬라이더가 진짜가 되며(한계 3), 토크가 표시값이 아니라
  구동의 원인이 된다.
- **B-07**: 마찰 원뿔 grasp 판정
- **D-02 (선택)**: 모션 로드/리플레이 — CSV를 읽어 `ActiveState`를 재생하면 한계 4번이 풀린다.
