# Step B-01: 동역학 파라미터 (질량·관성·토크 한계)

## 목표

STEP B(강체 동역학)에 필요한 **파라미터만** 수학 모델(`FSerial6DoFModel`)과 DataAsset(`URobotConfig`)에
추가한다. 링크 질량·질량중심·관성 텐서, 관절 최대 토크, 액추에이터 특성(로터 관성·마찰), 중력 벡터를
정의하고 기본값을 채운 뒤, 단위 규약과 링크 바디 정의를 확정하는 것이 전부다.

**이 단계는 로봇의 거동을 전혀 바꾸지 않는다.** 데이터가 늘어날 뿐이다.

## 범위 밖 (이번 단계에서 구현하지 않음)

RNEA/역동역학, 질량행렬 M(q), 순동역학 적분, S-curve 궤적, 토크 제어 루프, 고정 타임스텝 루프,
CSV 로깅, grasp/마찰/충돌, FSM, UMG. 전부 B-02 이후 단계다.

액터(`ASerial6DoFRobotActor`)도 수정하지 않았다 — 아직 노출할 UI도, 구동할 로직도 없다.
`Tick` / `ApplyJointState` / `SolveIKToTarget`은 한 줄도 바뀌지 않았다.

관절 **가속도/저크 한계**(`MaxAccelRadPerSec2` / `MaxJerkRadPerSec3`)도 이번엔 넣지 않았다.
저크 제한 S-curve를 실제로 만드는 **B-04**에서, 그 프로파일이 요구하는 형태로 함께 정의하는 편이
빈 필드를 미리 심어두는 것보다 낫다.

## 왜 이 단계를 따로 두는가

`FRobotJointLimit`과 `URobotConfig`는 **STEP A의 유일한 회귀 위험 지점**이다. STEP A의 FK/IK/테스트
26종이 전부 이 두 타입에 의존하므로, 동역학 계산을 얹기 전에 "데이터만 늘리고 거동은 그대로"를
먼저 통과시켜 둔다. 그러면 B-02에서 RNEA가 틀렸을 때 원인이 수식인지 데이터인지 헷갈리지 않는다.

## 단위 규약 — 동역학 내부만 SI

| 레이어 | 길이 | 각도 | 질량 | 토크 |
|---|---|---|---|---|
| FK/IK (STEP A) | **cm** | radian | — | — |
| 동역학 (STEP B) | **m** | radian | kg | N·m |
| 에디터 authoring | cm (기구학) / m (COM) | **도(degree)** | kg | N·m |

**왜 동역학만 SI인가.** 토크가 N·m로 나와야 실제 산업용 로봇 스펙과 대조할 수 있고, 필수 제출물인
물리값 CSV 로그도 의미를 갖는다. 전부 cm로 두면 토크 단위가 kg·cm²/s²(= 1e-4 N·m)라는 기묘한 값이
되어 "동역학 정확성"을 검증할 기준 자체가 사라진다.

**왜 STEP A의 cm를 그대로 두는가.** FK/IK를 m로 이전하면 골든값 (105, 0, 120), 테스트 26종,
액터 컴포넌트 미러링, DLS의 `PositionToleranceCm`/`RotationWeight` 튜닝까지 전부 건드려야 한다.
"STEP A 수학 코드와 테스트는 안정화된 상태로 유지"라는 원칙과 정면으로 충돌한다.

**cm↔m 경계는 어디인가.** 이번 단계에는 **없다.** 동역학 값을 저장만 하고 어떤 계산에도 쓰지 않기
때문이다. `URobotConfig::ToModel()`도 동역학 필드에 대해서는 **순수 복사**다(authoring 단위가 이미 SI).
경계 변환은 **B-02에서 RNEA 진입점 한 곳**에 모아 처리한다 — 거기서 `LinkOffsets`(cm) 같은 기구학
값을 SI로 올리게 되므로, 변환 지점이 한 함수 안에 모여 검증하기 쉽다.

이는 기존 코드베이스가 "도(degree)는 에디터 경계에서만" 규약을 지키는 것과 같은 발상이다.
**모든 변수명에 단위 접미사를 붙였다**(`MassKg`, `CenterOfMassLocalM`, `InertiaDiagonalKgM2`,
`MaxTorqueNm`, `LinkOffsetCm`) — 같은 구조체 안에 cm와 m가 공존하므로 이름이 유일한 방어선이다.

## 링크 바디 정의 (인덱스가 한 칸 밀린다)

`FSerial6DoFModel`의 FK 합성은 이렇다:

```
JointLocal  = FTransform(FQuat(JointAxes[i], Q[i]), LinkOffsets[i])
Accumulated = JointLocal * Accumulated        // child-first
```

즉 **관절 i의 회전은 `LinkOffsets[i]`만큼 떨어진 프레임 i 원점에서 일어난다.** 따라서 관절 i에 딸려
회전하는 강체(= 링크 i)는 **프레임 i 원점에서 다음 프레임 원점까지**를 차지한다:

| 링크 | span 벡터 (프레임 i 기준) | 통칭 | 길이 | 질량 |
|---|---|---|---|---|
| Link0 | `LinkOffsets[1]` = (0, 0, 20) | 어깨 | 20cm | 25kg |
| Link1 | `LinkOffsets[2]` = (0, 0, 60) | 상완 | 60cm | 20kg |
| Link2 | `LinkOffsets[3]` = (60, 0, 0) | 전완 | 60cm | 12kg |
| Link3 | `LinkOffsets[4]` = (20, 0, 0) | 손목 | 20cm | 5kg |
| Link4 | `LinkOffsets[5]` = (15, 0, 0) | 플랜지 | 15cm | 3kg |
| Link5 | `ToolOffset` = (10, 0, 0) | 툴/그리퍼 | 10cm | 2kg |

**함정 둘:**

1. **링크 i의 span은 `LinkOffsets[i]`가 아니라 `LinkOffsets[i+1]`이다** (i=5는 `ToolOffset`).
2. **`LinkOffsets[0]` = (0, 0, 40) 베이스 기둥은 동역학 대상이 아니다.** J0 피벗보다 아래에 있어
   어떤 관절과도 함께 회전하지 않는다 — 고정 베이스의 일부다.

`CreateDefault()`의 기존 주석은 `LinkOffsets[i]`를 오프셋 기준으로 "베이스 기둥/어깨/상완/전완/손목/
플랜지"라 라벨링한다. **위 바디 기준 라벨과 인덱스가 한 칸 어긋나므로** 그대로 옮겨 쓰면 질량이 전부
한 링크씩 밀린다. `InitializeDefaultLinkDynamics()`가 span을 `LinkOffsets[i+1]`에서 **직접 유도**하도록
짠 이유이고, 테스트 `ComLiesAtLinkSpanMidpoint`가 이 실수를 잡는다.

## 기본값 유도

숫자를 손으로 박지 않고 **기하에서 유도**한다 (`FSerial6DoFModel::InitializeDefaultLinkDynamics()`).

각 링크를 span 방향 **균일 밀도 실린더**(반지름 r, 길이 L)로 근사한다:

```
COM        = span 중점 = 0.5 × span
I_axial    = ½ m r²                       (span 축 방향)
I_transverse = (1/12) m (3r² + L²)        (나머지 두 축)
```

기본 로봇의 span은 전부 좌표축에 정렬돼 있으므로(±X 또는 ±Z) 관성 텐서가 프레임 축에서 **대각**이
되어 비대각 성분을 저장할 필요가 없다. span 축 성분만 `I_axial`, 나머지 두 축은 `I_transverse`다.

질량·반경은 도달거리 105cm급 산업용 암(KUKA KR16류)을 근사했고, 근위 링크가 무겁고 원위로 갈수록
가벼워지는 단조성만 지켰다 (반경 6cm → 3cm, 총 질량 67kg).

`MaxTorqueNm`(J0~J5 = 400 / 600 / 350 / 120 / 80 / 40)은 **잠정값**이다 — 아직 중력 토크를 계산할 수
없으므로 근위일수록 크게 두는 정도의 근사다. B-02에서 RNEA로 최악 자세의 정지 중력 토크를 산출한 뒤
여유 2~3배가 되도록 확정한다.

## 추가된 필드

**`FRobotJointLimit`** (`RobotTypes.h`) — 기존 필드/메서드는 불변, 하나만 추가:

| 필드 | 기본값 | 의미 |
|---|---|---|
| `MaxTorqueNm` | 100.0 | 관절 액추에이터 최대 토크. B-06에서 지령 토크 saturation에 사용 |

**`FRobotLinkDynamics`** (`RobotTypes.h`, 신규 plain struct — 리플렉션 불필요한 순수 계산용):

| 필드 | 의미 |
|---|---|
| `MassKg` | 링크 질량 |
| `CenterOfMassLocalM` | 관절 i 프레임 기준 COM (m) |
| `InertiaDiagonalKgM2` | COM 기준, 프레임 축 정렬 관성 대각 |
| `RotorInertiaKgM2` | 감속기 반영 로터 관성 (0 = 미모델링) |
| `ViscousFrictionNmsPerRad` | 점성 마찰 계수 (0 = 미모델링) |
| `CoulombFrictionNm` | Coulomb 마찰 토크 크기 (0 = 미모델링) |

관절 i와 링크 i가 1:1 대응하므로 링크 관성과 관절 액추에이터 특성을 한 구조체에 담았다.

**`FSerial6DoFModel`**: `LinkDynamics[6]`, `GravityMPerSec2` = (0, 0, −9.81).

**`URobotConfig`**: `FRobotLinkDynamicsConfig LinkDynamics[6]`, `GravityMPerSec2`,
`FRobotJointConfig::MaxTorqueNm`.

## FK 불변 보장

`ComputeJointWorldTransform` / `ComputeEndEffectorTransform` / `ComputeEndEffectorPose` /
`ClampToLimits` / `IsWithinLimits`는 **`LinkDynamics`와 `GravityMPerSec2`를 읽지 않는다.**
새 필드를 어떤 값으로 바꿔도 STEP A의 기구학 결과는 변하지 않으며, 테스트
`ZeroPoseFKUnaffectedByDynamics`가 이를 실증한다(질량 12345kg·중력 (100,100,100)으로 바꿔도 골든 포즈 유지).

## 방어 처리

`URobotConfig::ToModel()`은 동역학 값을 복사하되 물리적으로 불가능한 값을 기본 로봇 값으로 대체한다
(기존의 "0 벡터 축 → 폴백", "Min/Max 뒤집힘 정렬"과 같은 톤):

| 대상 | 통과 조건 | 위반 시 |
|---|---|---|
| `MassKg`, `InertiaDiagonalKgM2` 각 성분, `MaxTorqueNm` | 유한 **且** > 0 | `CreateDefault()` 값 |
| `RotorInertiaKgM2`, 마찰 2종 | 유한 **且** ≥ 0 (0 = 미모델링이라 유효) | `CreateDefault()` 값 |
| `CenterOfMassLocalM`, `GravityMPerSec2` | 유한 (음수/0 성분은 유효) | `CreateDefault()` 값 |

NaN/Inf가 모델로 새어나가지 않으므로, B-02 RNEA가 NaN을 만나면 원인이 데이터가 아니라 수식임을
확신할 수 있다.

## 변경/추가 파일

- 수정 `Public/Robot/RobotTypes.h` — `FRobotJointLimit::MaxTorqueNm`, 신규 `FRobotLinkDynamics`
- 수정 `Public/Robot/Serial6DoFModel.h` — `LinkDynamics[6]`, `GravityMPerSec2`, `InitializeDefaultLinkDynamics()` 선언
- 수정 `Private/Robot/Serial6DoFModel.cpp` — `CreateDefault()`에 토크 한계 추가, `InitializeDefaultLinkDynamics()` 구현
- 수정 `Public/Robot/RobotConfig.h` — 신규 `FRobotLinkDynamicsConfig`, `FRobotJointConfig::MaxTorqueNm`, `LinkDynamics[6]`, `GravityMPerSec2`
- 수정 `Private/Robot/RobotConfig.cpp` — 생성자 기본값 채움, `RobotConfigSanitize` 익명 네임스페이스, `ToModel()` 복사 + 방어
- 신규 `Private/Tests/RobotDynamicsParamsTests.cpp` — `RobotSim.Dynamics.Params.*`
- 신규 `Docs/Steps/STEP_B-01.md` (본 문서)

기존 테스트 26종(`RobotSim.Kinematics.*` / `PoseError.*` / `Jacobian.*` / `IK.*` / `IK.Nullspace.*`)은
**한 줄도 수정하지 않았다.**

## 테스트 (`RobotSim.Dynamics.Params.*`)

B-01은 데이터만 추가하므로 **데이터 불변성**만 검증한다. 동역학 "계산"을 검증하는 테스트는 B-02부터다.

| 테스트 | 검증 | 기대 |
|---|---|---|
| `DefaultsArePositiveFinite` | 기본값 전부 | 질량/관성/토크는 유한 양수, 로터·마찰은 유한 ≥ 0, 중력은 −Z |
| `InertiaSatisfiesTriangleInequality` | 관성 실현 가능성 | 세 조합 모두 `Ixx + Iyy ≥ Izz` 성립 (실제 강체의 필요조건) |
| `ComLiesAtLinkSpanMidpoint` | 링크 바디 정의 | COM = `LinkOffsets[i+1]` 중점 (한 칸 밀림 실수 방지) |
| `ZeroPoseFKUnaffectedByDynamics` | **회귀 가드** | 동역학 값을 극단적으로 바꿔도 Q=0 EE = (105, 0, 120), 임의 자세 EE도 동일 |
| `ConfigToModelCopiesDynamicsAndKeepsFK` | `ToModel()` 왕복 | 동역학 값이 그대로 복사되고 FK 골든값 유지 |
| `ConfigToModelSanitizesInvalidValues` | 방어 처리 | NaN/Inf/0 이하를 심어도 모델은 유효, FK 정상 |

`URobotConfig`를 쓰는 두 테스트는 월드가 필요 없도록 `NewObject` + `TStrongObjectPtr`(GC 보호)로
생성한다. CDO(`GetMutableDefault`)를 쓰지 않는 이유는 sanitize 테스트가 값을 변조하기 때문이다 —
CDO를 오염시키면 같은 세션의 다른 테스트에 영향을 준다.

## 한계

- **질량·관성에 CAD 근거가 없다.** 균일 밀도 실린더 근사로 유도한 추정치이며, 실제 KUKA 링크의
  질량 분포(모터·감속기가 관절 쪽에 몰림)와 다르다. 절대 토크값의 정확도보다 **거동의 정성적 타당성**이
  목표다. 메시 비율/`LinkOffsets` 보정은 이번에도 건드리지 않았다.
- **관성 텐서가 대각 근사다.** 축 정렬 균일 형상을 가정하므로 비대각 성분이 0이다. 실제 링크는
  비대각 성분이 있다. 필요해지면 `InertiaDiagonalKgM2`를 3×3으로 확장하면 되고 RNEA 수식은 그대로다.
- **`MaxTorqueNm`이 잠정값이다.** B-02에서 중력 토크를 산출한 뒤 확정한다.
- **베이스 기둥이 동역학에서 빠진다.** 고정 링크라 관절 토크에 기여하지 않으므로 의도된 것이지만,
  베이스 전체 반력(바닥 고정 하중)을 계산하려면 별도로 다뤄야 한다.
- **액추에이터 특성(로터 관성·마찰)이 미검증이다.** B-06 토크 제어에서 튜닝 대상이며, 현재 값은
  자리를 잡아둔 것에 가깝다.
- **`RotorInertiaKgM2`를 링크 struct에 담았다.** 엄밀히는 관절 액추에이터 속성이라 `FRobotJointLimit`
  쪽이 자연스럽지만, 관절-링크가 1:1이라 한 구조체로 합쳤다. 대신 `MaxTorqueNm`만 `FRobotJointLimit`에
  있어 액추에이터 파라미터가 두 구조체에 나뉘어 있다.

## 다음 단계: B-02 (RNEA 역동역학)

이 값들이 어떻게 쓰이는지:

- `LinkDynamics[i].MassKg` / `CenterOfMassLocalM` / `InertiaDiagonalKgM2` →
  Recursive Newton-Euler의 **forward pass**(각 링크 프레임의 속도·가속도 전파)와
  **backward pass**(링크에 작용하는 힘·모멘트를 말단부터 역으로 누적)의 입력.
- `GravityMPerSec2` → forward pass의 **base acceleration 초기값**. 베이스를 `−g`로 가속시키는
  고전적 트릭으로 중력 보상 토크가 자동으로 나온다.
- `RotorInertiaKgM2` → 관절 축 방향 관성에 가산.
- `MaxTorqueNm` → B-02에서는 산출된 중력 토크와 대조해 **값 자체를 확정**하는 데 쓰고,
  B-06에서 실제 saturation에 쓴다.
- **cm→m 변환이 B-02에서 처음 등장한다** — `LinkOffsets`(cm)를 RNEA 진입점에서 m로 올린다.
  이 변환을 한 함수 안에 가두는 것이 B-02의 설계 제약이다.

B-02의 검증 축: 정지 자세(qd = qdd = 0)에서 나온 토크가 손으로 계산한 중력 토크와 일치하는가,
무중력·무운동에서 토크가 0인가, 그리고 M(q)를 B-03에서 뽑았을 때 대칭 양정부호인가.
