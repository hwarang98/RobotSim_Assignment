# Step B-02: RNEA 역동역학 (관절 상태 → 토크)

## 목표

B-01이 저장만 해둔 동역학 파라미터를 **처음으로 소비**한다. 관절 상태 (q, qd, qdd)에서 각 관절이
내야 하는 토크 τ(N·m)를 구하는 Recursive Newton-Euler 역동역학을 순수 수학 함수로 구현한다.

핵심 산출물은 **중력 보상 토크**다 — qd = qdd = 0을 넣으면 그 자세를 유지하는 데 필요한 정지
토크가 나온다. 이것이 B-06 토크 제어의 feedforward 항이자 `MaxTorqueNm` 산정의 근거가 된다.

## 범위 밖 (이번 단계에서 구현하지 않음)

질량행렬 M(q) 추출, 순동역학 적분, S-curve 궤적, 토크 제어 루프, 고정 타임스텝 루프, CSV 로깅,
grasp/마찰 접촉/충돌, FSM, UMG. 전부 B-03 이후다.

액터(`ASerial6DoFRobotActor`)는 수정하지 않았다 — RNEA는 아직 어떤 런타임 경로에도 연결되지 않은
순수 수학 함수이며, 이번 단계에서 검증하는 것은 수식의 정확성뿐이다.

`FSerial6DoFModel`의 기구학(`ComputeJointWorldTransform` / `ComputeEndEffectorTransform` /
`LinkOffsets` / `JointAxes` / `JointLimits`)도 **한 줄도 바꾸지 않았다.** RNEA는 모델을 const로
읽기만 한다.

## 왜 역동역학이 먼저인가

순동역학(τ → qdd)이 최종 목표지만, 역동역학이 **모든 것의 부품**이다:

- **B-03 M(q)**: 질량행렬의 j번째 열 = `RNEA(q, qd=0, qdd=e_j, gravity off)`. 즉 B-03은 이 함수를
  6번 호출하는 것으로 끝난다. 순동역학은 qdd = M⁻¹(τ − C(q,qd)qd − g(q))이고, 여기서
  C·qd = `RNEA(q, qd, 0, gravity off)`, g = `RNEA(q, 0, 0, gravity on)`이다.
- **B-06 토크 제어**: 중력 보상 토크가 feedforward 항으로 그대로 들어간다.
- **`MaxTorqueNm` 확정**: 최악 자세의 정지 중력 토크가 액추에이터 사양의 하한이다.

역동역학 하나를 제대로 검증해 두면 이후 단계는 재호출과 선형대수만 남는다. B-01이 "데이터만
추가하고 거동은 그대로"를 먼저 통과시킨 것과 같은 발상이다 — B-03에서 M(q)가 대칭 양정부호가
아니면, 원인이 RNEA가 아니라 추출 방식임을 확신할 수 있다.

## SI 단위 규약

B-01의 표를 그대로 따른다:

| 레이어 | 길이 | 각도 | 질량 | 관성 | 중력 | 토크 |
|---|---|---|---|---|---|---|
| FK/IK (STEP A) | **cm** | radian | — | — | — | — |
| 동역학 (STEP B) | **m** | radian | kg | kg·m² | m/s² | N·m |

**RNEA 내부는 100% SI다.** 토크가 N·m로 나와야 실제 산업용 로봇 스펙과 대조할 수 있고, 필수
제출물인 물리값 CSV 로그도 의미를 갖는다. 전부 cm로 두면 토크 단위가 kg·cm²/s²(= 1e-4 N·m)라는
기묘한 값이 되어 검증 기준 자체가 사라진다.

**변수명에 단위 접미사를 붙이는 것이 유일한 방어선이다** — `LinkOffsetM`, `ComAccelMPerSec2`,
`AngularVelRadPerSec`, `LinkForceN`, `TorqueNm`. 같은 함수 안에 cm 입력과 m 계산이 공존하기 때문이다.

## cm→m 경계 변환

B-01이 예고한 대로 **cm→m 변환이 이번 단계에서 처음 등장한다.** 설계 제약은 "변환을 한 함수
안에 가둔다"였고, 실제로 `SolveInverseDynamicsRNEA()` 진입부 한 블록에만 존재한다:

```cpp
// 이 블록이 파일 전체에서 유일한 cm→m 지점. 이후 코드는 전부 SI만 다룬다.
FVector LinkOffsetM[NumJoints + 1];
for (int32 i = 0; i < NumJoints; ++i)
{
    LinkOffsetM[i] = Model.LinkOffsets[i] * RobotCmToM;
}
LinkOffsetM[NumJoints] = Model.ToolOffset.GetTranslation() * RobotCmToM;
```

`RobotCmToM = 0.01`은 `RobotDynamicsRNEA.h`에 공개 상수로 둔다 — 테스트도 같은 상수를 써야
"테스트가 0.01을 하드코딩해서 우연히 맞는" 상황을 피할 수 있다.

**최대 함정: 같은 모델 안에서 단위가 섞여 있다.**

| 필드 | 단위 | RNEA에서 |
|---|---|---|
| `LinkOffsets[i]`, `ToolOffset` | **cm** | 변환한다 |
| `CenterOfMassLocalM` | **m** (B-01이 이미 SI로 저장) | **변환하지 않는다** |
| `InertiaDiagonalKgM2`, `MassKg`, `GravityMPerSec2` | SI | 변환하지 않는다 |

COM에 `RobotCmToM`을 한 번 더 곱하면 토크가 100배 작아진다. `LinkOffsets`에 곱하지 않으면 100배
커진다. 두 실수 모두 위치에너지 대조 테스트가 즉시 잡는다(100배 오차가 어떤 tolerance도 통과하지 못한다).

## 프레임 규약

**이 절이 B-02의 최대 리스크 지점이다.** 기존 FK가 교재의 어떤 관례에 대응하는지 확정해야 한다.

기존 FK는 이렇다:
```
JointLocal  = FTransform(FQuat(JointAxes[i], Q[i]), LinkOffsets[i])
Accumulated = JointLocal * Accumulated        // child-first
```

`FTransform(Rot, Trans)`가 `p ↦ Rot·p + Trans`이므로 `JointLocal`은 **프레임 i 좌표를 프레임 i−1
좌표로 옮기는 사상**이다. 여기서 네 가지가 따라 나온다:

1. **`R[i] = FQuat(JointAxes[i], Q[i])`는 프레임 i → 프레임 i−1 사상.** 교재(Craig)의 `R^{i−1}_i`에
   해당한다. 부모→자식은 `R[i].Inverse()`이고, forward pass에서 부모의 운동을 끌어올 때 이걸 쓴다.

2. **`LinkOffsets[i]`는 프레임 i 원점의 부모 프레임 좌표이며 `Q[i]`로 회전하지 않는다.**
   회전은 오프셋만큼 이동한 **뒤** 프레임 i 원점에서 일어난다. 따라서 forward pass의 원점 가속도
   전파는 `LinkOffsetM[i]`를 **회전 적용 전 부모 프레임에서** 계산한 다음 통째로 자식 프레임으로
   돌려야 한다. 순서를 바꾸면 틀린다.

3. **`JointAxes[i]`는 축 변환이 필요 없다.** 프레임 i 기준 벡터지만 회전이 자기 축을 불변으로
   두므로(`R[i]·axis = axis`) 프레임 i−1에서도 수치가 같다. 교재의 `z_i`와 정확히 일치한다.
   이 모델이 운 좋게 표준 RNEA에 그대로 얹히는 이유다.

4. **링크 i의 강체는 프레임 i 원점 → 프레임 i+1 원점 구간이다.** 즉 span은 `LinkOffsets[i]`가 아니라
   **`LinkOffsets[i+1]`**이고 i=5는 `ToolOffset`이다. B-01이 경고한 "한 칸 밀림" 함정이며,
   `CenterOfMassLocalM`은 이미 이 규약으로 저장돼 있다. backward pass에서 자식 반력의 모멘트 팔로
   `LinkOffsetM[i+1]`을 쓰는 것이 이 규약의 직접적 결과다.

**`LinkOffsets[0]`(베이스 기둥 40cm)은 재귀에서 자동으로 탈락한다.** 베이스가 정지 상태라
`ω[−1] = α[−1] = 0`이고, 원점 가속도 전파식의 두 교차항이 모두 0이 되어 `p[0]`이 곱해질 곳이
없어진다. "J0 피벗보다 아래에 있어 어떤 관절과도 회전하지 않는다"는 물리적 사실이 별도 처리 없이
수식에서 그대로 나온다.

**표현 프레임.** 모든 spatial 량(ω, α, a, f, n)은 **자기 링크 프레임 i 기준**이다. `a[i]`는 프레임 i
**원점**의 가속도이고, `n[i]`는 프레임 i **원점**에 대한 모멘트다. `τ[i] = Dot(n[i], z[i])`.

**`BaseTransform`은 무시한다.** `GravityMPerSec2`가 "베이스 프레임 기준"으로 정의돼 있으므로 RNEA는
베이스 프레임에서 닫힌다. 베이스가 월드에서 기울어졌다면 호출자가 중력을 미리 베이스 프레임으로
회전시켜 `GravityOverrideMPerSec2`로 넘겨야 한다(→ 한계).

**3D 벡터 재귀를 택한 이유.** spatial 6D 대수(Featherstone)가 더 일반적이지만, 6R 고정 모델에는
3D 벡터 revolute-only 재귀로 충분하고 위 프레임 대응을 눈으로 따라갈 수 있다. 검증되지 않은
프레임 규약 위에 추상화를 얹으면 틀렸을 때 원인을 좁히기 어렵다.

## 수식

`R[i]`=프레임 i→i−1, `p[i]`=`LinkOffsetM[i]`(프레임 i−1 기준), `z[i]`=`JointAxes[i]`,
`c[i]`=`CenterOfMassLocalM`(프레임 i), `I[i]`=`InertiaDiagonalKgM2`(COM 기준, 프레임 i 축 정렬).

**Forward pass** (i = 0…5) — 베이스에서 말단으로 속도/가속도를 전파:
```
초기값: ω[−1] = 0,  α[−1] = 0,  a[−1] = −g        ← 중력 트릭

ω[i]  = R[i]ᵀ·ω[i−1] + z[i]·qd[i]
α[i]  = R[i]ᵀ·α[i−1] + (R[i]ᵀ·ω[i−1]) × (z[i]·qd[i]) + z[i]·qdd[i]
a[i]  = R[i]ᵀ·( a[i−1] + α[i−1] × p[i] + ω[i−1] × (ω[i−1] × p[i]) )
ac[i] = a[i] + α[i] × c[i] + ω[i] × (ω[i] × c[i])          // COM 가속도
F[i]  = m[i] · ac[i]                                        // Newton
N[i]  = I[i]·α[i] + ω[i] × (I[i]·ω[i])                      // Euler (대각이라 성분곱)
```

**Backward pass** (i = 5…0) — 말단에서 베이스로 힘/모멘트를 역누적:
```
초기값: f[6] = 0,  n[6] = 0                       ← 말단 외력 없음

f[i] = R[i+1]·f[i+1] + F[i]
n[i] = R[i+1]·n[i+1] + c[i] × F[i] + p[i+1] × (R[i+1]·f[i+1]) + N[i]
τ[i] = Dot(n[i], z[i])                            // 나머지 성분은 베어링이 받는다
```

**옵션 항** (기본 전부 off):
```
if (bIncludeRotorInertia) τ[i] += RotorInertiaKgM2[i] · qdd[i]
if (bIncludeFriction)     τ[i] += ViscousFrictionNmsPerRad[i]·qd[i] + CoulombFrictionNm[i]·sign(qd[i])
```

## 중력 토크 계산의 의미

**`a[−1] = −g` 트릭.** 중력을 별도 항으로 더하지 않는다. 대신 베이스가 위로 `+9.81 m/s²`로
가속하는 비관성 좌표계인 척한다. 그러면 각 링크의 관성력 `m·ac`에 중력이 자동으로 섞여 나온다 —
등가원리(중력장 안에 정지 = 중력 없이 −g로 가속)를 그대로 쓰는 셈이다. 코드에 "중력 항"이
보이지 않는 이유이고, 링크마다 `m·g`를 더하는 방식보다 실수 여지가 적다.

**qd = qdd = 0이면 결과가 곧 중력 보상 토크다.** 로봇 동역학의 표준형
```
M(q)·qdd + C(q,qd)·qd + g(q) = τ
```
에서 qd = qdd = 0을 넣으면 `τ = g(q)`만 남는다. 그리고 `g(q) = ∂U/∂q` (U = 중력 위치에너지)이므로,
`ComputeGravityTorque()`의 출력은 **그 자세를 유지하는 데 각 관절이 버텨야 하는 정지 토크**다.
이 값이 `MaxTorqueNm`의 하한이자 B-06 feedforward 항이 된다.

**구조적 불변량 둘.** 이 모델에서 중력 토크가 **어떤 자세에서도 0인** 관절이 두 개 있다:

- **J0 (yaw, +Z)** — 중력이 −Z라 J0 축과 평행하고, 수직력은 수직축 둘레 모멘트를 만들지 못한다
  (`r × F`의 Z 성분 = `r_x·F_y − r_y·F_x = 0`). 위치에너지 쪽에서 보면 "Z축 둘레로 돌려도 높이가
  안 변하니 U가 q0에 무관"과 같은 말이다.
- **J5 (툴 roll, +X)** — `ToolOffset` = (10, 0, 0)이 `JointAxes[5]` = +X와 **동일선상**이라 툴의 COM
  (= span 중점 = (5, 0, 0))이 항상 자기 회전축 위에 놓인다. 모멘트 팔이 0이다. `CreateDefault()`가
  "J5 회전은 EE 위치를 바꾸지 않고 자세만 바꾼다"를 단위테스트 불변량으로 의도한 결과가
  동역학에도 그대로 나타난 것이다.

테스트 6(위치에너지)·8(리포트)이 이를 양쪽 경로로 확인한다. **두 관절의 토크 한계는 중력으로
산정할 수 없다** — B-03의 M(q) 대각(순수 가속 요구)에서 잡아야 한다.

**J3는 구조적 0이 아니다.** 아래 리포트의 자세 A에서 J3가 0으로 나오지만 이건 그 자세의 성질일
뿐이다 — 자세히 보면 손목을 꺾어야 부하가 걸린다(§권장 MaxTorqueNm 산정 방식).

## 마찰/로터 관성을 옵션으로 분리한 이유

**기본은 순수 강체 역동역학이다** (`bIncludeRotorInertia = false`, `bIncludeFriction = false`).

B-01 문서가 밝혔듯 로터 관성·마찰 값은 **CAD 근거 없는 자리잡기용 추정치**이고 B-06 토크 제어에서
튜닝할 대상이다. 검증되지 않은 항을 기본으로 켜면 강체 항의 정확성이 흐려진다 — 위치에너지 대조
테스트가 어긋났을 때 원인이 RNEA인지 마찰 모델인지 구분할 수 없게 된다. 켜면 강체 토크에 **가산**될
뿐이라 언제든 분리 검증할 수 있고, 테스트 7이 실제로 "옵션 on − off = 정확히 해당 항의 수식"임을
확인한다.

**마찰 부호 규약.** 마찰 토크 자체는 운동을 방해하는 방향(`τ_f = −b·qd − c·sign(qd)`)이지만,
**역동역학이 구하는 것은 모터가 내야 하는 토크**다. 모터는 마찰을 추가로 이겨내야 하므로 `+`로
가산한다 — 즉 마찰 항은 항상 `qd`와 **같은 부호**다. 테스트 7이 이 부호를 명시적으로 검증한다.

**정지 마찰(stiction)은 모델링하지 않는다.** `qd = 0`이면 `sign(qd) = 0`이라 Coulomb 항이 사라진다.
실제 로봇은 정지 상태에서 최대 정지 마찰까지 버티지만, 그 불연속을 다루려면 접촉 모델이 필요하고
이는 B-05 grasp의 영역이다.

## 변경/추가 파일

- 신규 `Public/Robot/RobotDynamicsRNEA.h` — `RobotCmToM`, `FRobotRNEAOptions`,
  `SolveInverseDynamicsRNEA()`, `ComputeGravityTorque()`
- 신규 `Private/Robot/RobotDynamicsRNEA.cpp` — forward/backward pass 구현
- 수정 `Public/Robot/RobotTypes.h` — 신규 plain struct `FRobot6DJointVelocity` /
  `FRobot6DJointAcceleration` / `FRobot6DJointTorque` 추가 (기존 타입은 불변)
- 신규 `Private/Tests/RobotRNEATests.cpp` — `RobotSim.Dynamics.RNEA.*`
- 신규 `Docs/Steps/STEP_B-02.md` (본 문서)

`Serial6DoFModel.h/.cpp`, `RobotConfig.h/.cpp`, `Serial6DoFRobotActor.*`는 **한 줄도 수정하지 않았다.**
기존 테스트 32종도 그대로다.

**qd/qdd/τ 타입을 `RobotTypes.h`에 둔 이유.** B-01은 이 타입들을 만들지 않았다(파라미터만 추가했다).
RNEA 헤더에 두면 B-03 순동역학이 타입을 쓰려고 RNEA 헤더를 include해야 하는 역의존이 생긴다.
관절 공간 상태 타입은 `FRobot6DJointState` 옆에 모아두는 편이 자연스럽다.

**중력 상수를 RNEA가 소유하지 않는다.** `Options.GravityOverrideMPerSec2`가 비어 있으면
`Model.GravityMPerSec2`(= B-01의 (0, 0, −9.81))를 읽는다. 표준 중력 9.80665와 0.05% 차이가 있지만,
CAD 근거 없는 잠정 질량 추정치의 오차에 비하면 무의미하고, B-01의 모델/에셋/테스트가 이미 −9.81로
정합해 있어 그쪽을 건드리면 회귀 추적이 더러워진다.

## 테스트 (`RobotSim.Dynamics.RNEA.*`)

| 테스트 | 검증 | 기대 |
|---|---|---|
| `ZeroMassOrGravityOffProducesZero` | 항등원 | 중력 off + 정지 → τ = 0, 질량 0 + 정지 → τ = 0 (근사가 아니라 정확히, tol 1e-12) |
| `FiniteTorques` | NaN 누출 | Q=0 중력 토크, 그리고 임의 자세 + 속도 + 가속도 + 옵션 전부 on에서 τ 유한 |
| `GravityTorqueChangesWithPose` | 자세 의존성 | J1을 30° 기울이면 J1 중력 토크가 1 N·m 넘게 달라진다 |
| `AccelerationTorqueSign` | 관성 항 부호 | 관절 j만 +1 rad/s² → τ_j > 0. M(q) 대각이 양수라 자세 무관하게 성립하므로 Q=0과 임의 자세 양쪽에서 6관절 전부 확인 |
| `FKRegressionUnaffected` | **회귀 가드** | RNEA를 온갖 조건으로 호출한 뒤에도 Q=0 EE = (105, 0, 120), 임의 자세 EE도 동일 |
| **`GravityTorqueMatchesEnergyGradient`** | **핵심 교차검증** | τ_g,j = ∂U/∂q_j를 3개 자세 × 6관절에서 확인 |
| `FrictionAndRotorInertiaAreSeparableTerms` | 항 분리 | 옵션 on − off = 정확히 `Ir·qdd` / `b·qd + c·sign(qd)`. 마찰이 qd와 같은 부호인지, qd=0에서 0인지도 확인 |
| `GravityTorqueReportForMaxTorqueSizing` | **비단정 진단** | 두 최악 자세의 중력 토크를 `LogRobotSim`으로 출력. 단정은 유한성 + 구조적 0(J0/J5) + "손목을 꺾으면 J3에 토크가 걸린다"까지만 — `MaxTorqueNm`과의 비교는 하지 않는다 |

### 위치에너지 대조가 왜 결정적인가

나머지 테스트는 전부 sanity 수준이라 **부호가 뒤집혀도, 링크 인덱스가 한 칸 밀려도 통과할 수 있다.**
"자세가 바뀌면 토크도 바뀐다"는 틀린 구현도 만족시킨다.

`GravityTorqueMatchesEnergyGradient`는 다르다:

```
τ_g,j = ∂U/∂q_j,   U(q) = −Σ mᵢ · g · p_comᵢ(q)
```

우변의 `p_comᵢ`를 **기존 `ComputeJointWorldTransform`**(Step A에서 이미 골든값으로 검증된 FK)만으로
구한다. RNEA의 forward/backward pass를 한 줄도 타지 않는 **완전히 독립된 경로**다. 두 경로가 3개
자세 × 6관절에서 모두 일치한다면 프레임 규약·링크 인덱스·cm→m 경계·중력 부호가 전부 맞았다는 뜻이다.
반대로 이게 깨지면 위 "프레임 규약" 절을 가장 먼저 의심해야 한다.

**tolerance를 넉넉하게 잡았다.** 중심차분(h = 1e-5 rad)은 절단오차와 반올림오차가 섞이므로
상대오차 1e-3에 절대 하한 1e-2 N·m을 둔다. 기본 모델의 중력 토크는 O(100) N·m 규모라 이 정도로도
잡으려는 실수(부호 뒤집힘, 인덱스 밀림, cm/m 혼동 = 100배)는 전부 걸리면서, 질량·COM이 잠정
추정치라는 사실 때문에 테스트가 brittle해지지 않는다.

## 권장 MaxTorqueNm 산정 방식

### 최악 자세는 관절마다 다르다

처음엔 "팔을 완전히 수평 전개한 자세 하나면 전부 커버된다"고 생각했지만 **틀렸다.** 리포트를
실제로 돌려보니 그 자세에서 J3가 0으로 나왔고, 원인을 따져보니 최악 자세가 관절마다 다르기
때문이었다. 그래서 자세를 두 개 둔다:

**자세 A — `q = (0, +90°, −90°, 0, 0, 0)`: J1/J2/J4의 최악 자세.**
J1 = +90°가 상완을 +Z에서 +X로 눕히고(+Z→+X), J2 = −90°가 그 뒤를 다시 +X로 편다. 팔 전체가
+X로 수평 전개되어(EE = (165, 0, 60)cm, Q=0의 105cm보다 멀다) 이 세 관절의 모멘트 팔이 최대가 된다.

**자세 B — `q = (0, +90°, −90°, +90°, +90°, 0)`: J3 부하용.**
자세 A에서 J3가 0인 이유는 팔 전체가 y = 0 평면에 눕기 때문이다. J3의 축 둘레 모멘트는
`(r × F)·x̂₃ = −r_z·F_y`인데, 자세 A에서는 중력이 프레임 3의 y축 성분을 갖지 않아 `F_y = 0`이다.
**J3 = 90°가 프레임 3의 y축을 수직으로 세워 `F_y ≠ 0`을 만들고, J4 = 90°가 하류 COM을 J3 축에서
떼어내 `r_z ≠ 0`을 만든다. 둘 다 있어야 토크가 걸린다.**

모든 각도가 관절 한계 안이다(J1 ±120°, J2 ±150°, J3 ±350°, J4 ±120°).

### 측정 결과 (기본 모델, `CreateDefault()`)

| 관절 | 중력토크 (N·m) | 최악 자세 | 현재 (잠정) | 여유배수 | 권장 (×2.5) | 판정 |
|---|---|---|---|---|---|---|
| J0 | **0** (구조적) | — | 400 | — | M(q)에서 산정 | 중력으로 산정 불가 |
| J1 | **−303.4** | A | 600 | **1.98** | **758** | **부족** |
| J2 | **−115.0** | A | 350 | 3.04 | 288 | 적절 |
| J3 | **−6.1** | B | 120 | 19.6 | 15 | 과대 |
| J4 | **−6.1** | A | 80 | 13.1 | 15 | 과대 |
| J5 | **0** (구조적) | — | 40 | — | M(q)에서 산정 | 중력으로 산정 불가 |

**J1이 여유 1.98배로 부족하다** — 중력을 버티는 데 이미 절반을 쓰므로 가속에 남는 힘이 부족하다.
600 → 758 N·m 이상이 필요하다. 반대로 J3/J4는 13~20배로 과대하지만, 이쪽은 관성 부하를 아직
모르므로 낮추기 전에 M(q)를 봐야 한다.

J3와 J4의 중력 토크가 6.1로 같은 것은 우연이 아니다. 두 관절 모두 하류가 (링크4 COM 0.075m ×
3kg) + (링크5 COM 0.20m × 2kg) = 0.625 kg·m로 같고, 자세 B의 roll이 이 기하를 보존하기 때문이다.

### 산정 절차

1. 각 관절의 최악 자세에서 정지 중력 토크 |τ_g,i|를 읽는다 (위 표).
2. `MaxTorqueNm[i] ≥ |τ_g,i| × 2.5`가 되도록 잡는다. 여유 2~3배는 중력을 버티고 **남는 힘으로
   가속**해야 하기 때문이다 — 중력 보상만 하면 로봇이 자세를 유지할 뿐 움직이지 못한다.
3. **J0/J5는 이 방식으로 값이 나오지 않는다** (구조적 0). 순수 가속 요구에서 잡아야 한다.
4. 정확한 값은 B-03의 M(q)와 B-04의 최대 가속도가 정해진 뒤 `|τ_g| + M(q)·qdd_max`로 재산정하는
   것이 옳다. ×2.5는 그 전까지의 대용품이다.

### 왜 이번 단계에서 자동 확정하지 않는가

B-01의 잠정값 `{400, 600, 350, 120, 80, 40}`과 `URobotConfig` 에셋 값을 **코드에서 자동으로 바꾸지
않는다.** 테스트도 `MaxTorqueNm`과의 비교를 단정하지 않고 로그만 남긴다.

이유는 **회귀 추적**이다. RNEA 자체가 아직 사람 손으로 검토되지 않은 상태에서 토크 한계를 그
출력으로 갱신하면, 두 값이 서로를 정당화하는 순환이 생긴다. 이후 어딘가에서 회귀가 났을 때
원인이 수식인지 한계값인지 분리할 수 없다. 또 RNEA가 100배 틀린 값을 내놓아도(cm/m 혼동)
한계값이 조용히 따라 움직여 테스트가 계속 초록으로 남는다.

위 표는 그 판단의 근거 자료일 뿐이며, 실제 값 확정은 **별도 커밋**에서 한다. 특히 J0/J5는 M(q)
없이는 산정할 수 없으므로, B-03 이후로 미루는 편이 자연스럽다.

## 한계

- **3D 벡터 revolute-only 재귀다.** prismatic 관절, 분기 트리, 폐루프를 지원하지 않는다. 6R 고정
  모델에는 충분하지만 일반 로봇으로 확장하려면 spatial 6D 대수(Featherstone)로 갈아타야 한다.
- **말단 외력을 지원하지 않는다.** backward pass의 `f[6] = n[6] = 0`이 고정이다. B-05 grasp에서
  물체 반력을 다루려면 이 초기값을 파라미터로 여는 것이 확장 지점이다 — 수식은 그대로다.
- **관성 텐서가 대각 근사다.** B-01의 축 정렬 균일 실린더 가정을 그대로 받는다. 비대각이 필요해지면
  `MultiplyDiagonalInertia()`를 행렬곱으로 교체하면 되고 RNEA 수식은 바뀌지 않는다.
- **`BaseTransform`을 무시한다.** 베이스 프레임에서 닫히므로 베이스가 월드에서 기울어진 경우
  호출자가 중력을 미리 회전시켜야 한다. 기본 모델은 `BaseTransform = Identity`라 현재는 무해하다.
- **정지 마찰을 모델링하지 않는다.** `qd = 0`에서 Coulomb 항이 0이다.
- **질량·관성·마찰이 여전히 CAD 근거 없는 추정치다.** B-01의 한계를 그대로 물려받는다. 절대
  토크값의 정확도가 아니라 **거동의 정성적 타당성**이 목표다. 위치에너지 대조는 "RNEA가 이 파라미터에
  대해 옳은가"를 볼 뿐 "파라미터가 실물과 같은가"를 보지 않는다.
- **`RobotCmToM`이 `Serial6DoFModel.cpp:79`의 파일 로컬 `CmToM`과 중복된다.** 통합하려면 B-01
  파일을 수정해야 해서 이번엔 보류했다. 값이 같은 두 상수가 공존한다.
- **최악 자세를 손으로 골랐다.** 리포트의 두 자세는 기하를 보고 추론한 것이지 탐색한 결과가
  아니다. J1/J2/J4는 "수평 전개가 모멘트 팔 최대"라는 근거가 명확하지만, J3는 자세 B가 정말
  최악인지 확인하지 않았다(부하가 걸리는 자세를 하나 찾았을 뿐이다). 엄밀히 하려면 관절 공간을
  샘플링해 `max_q |τ_g,i(q)|`를 관절별로 구해야 한다 — `MaxTorqueNm`을 실제로 확정하는 커밋에서
  다룰 일이다.
- **아직 어디에도 연결되지 않았다.** 액터/런타임 경로가 RNEA를 호출하지 않으므로 실제 시뮬레이션
  거동은 B-01과 동일하다.

## 다음 단계: B-03 (질량행렬 M(q)와 순동역학)

B-02의 함수를 재호출하는 것으로 대부분이 끝난다:

- **M(q) 추출**: j번째 열 = `SolveInverseDynamicsRNEA(Model, q, 0, e_j, {bEnableGravity=false})`.
  6번 호출하면 6×6 행렬이 완성된다.
- **중력 항** g(q) = `ComputeGravityTorque(Model, q)` — 이미 있다.
- **Coriolis/원심 항** C(q,qd)·qd = `SolveInverseDynamicsRNEA(Model, q, qd, 0, {bEnableGravity=false})`.
- **순동역학**: qdd = M(q)⁻¹·(τ − C(q,qd)·qd − g(q)). 6×6 역행렬은 A-03/A-04의 선형대수 유틸을
  재사용할 수 있는지 먼저 확인한다.

B-03의 검증 축: **M(q)가 대칭 양정부호인가** (물리적으로 반드시 성립 — 운동에너지 ½·qdᵀ·M·qd > 0).
대칭성이 깨지면 RNEA가 아니라 추출 방식을 의심한다. 그리고 역동역학 → 순동역학 왕복
(`τ = RNEA(q, qd, qdd)` → `qdd' = FD(q, qd, τ)` → `qdd' ≈ qdd`)이 닫히는가.
