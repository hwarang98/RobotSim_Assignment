# Step A-03: Numerical Jacobian

## 목표

STEP_A-02에서 구현한 6D pose error를 이용해, 6-DOF serial robot의 **numerical Jacobian J(6×6)** 를
유한차분(finite difference)으로 계산하는 순수 수학 레이어를 추가한다.
이 단계에서는 **DLS IK solve, target 이동 UI, pick/place FSM을 구현하지 않는다.** J 계산까지만 한다.

## 왜 analytic Jacobian 전에 numerical Jacobian을 먼저 구현했는가

Jacobian은 두 방식으로 구할 수 있다.

- **analytic/geometric Jacobian**: 각 관절 축과 링크 기하로부터 손으로 유도한 닫힌 형식. 빠르지만
  유도·부호·좌표계 실수가 들어가기 쉽다.
- **numerical Jacobian**: 검증된 FK를 미소 섭동해 수치 미분. 느리지만 **유도 오류가 원천적으로 없다.**

A-1에서 FK를, A-2에서 pose error를 먼저 검증 위에 고정한 것과 같은 전략이다.
numerical Jacobian은 이미 검증된 `FSerial6DoFModel`(FK source of truth)과
`FRobotPoseError`(A-02 quaternion-log pose error)를 **그대로 재사용**하므로, 새로운 수학이
거의 들어가지 않는다. 그래서 이후 analytic/geometric Jacobian을 구현하면 이 numerical J와
성분별로 비교해 검증할 수 있는 **gold reference** 가 된다.

즉 지금 J가 틀리면 원인은 FK 또는 pose error(이미 테스트로 고정됨)뿐이므로, A-04 DLS 디버깅은
순수하게 "수렴/스텝/특이점" 문제로 좁혀진다.

## 행/열 규약

Jacobian `M[row][col] = ∂(pose error 성분 row) / ∂(관절 col 각도)`.

- **행(row)** — pose error 6-벡터 `[px, py, pz, rx, ry, rz]`, 즉 `FRobot6DPoseError::ToArray6`와 동일 레이아웃:
  - row 0~2: **position error derivative, 단위 cm/rad**
  - row 3~5: **rotation error derivative(quaternion-log rotation vector), 단위 rad/rad**
- **열(col)** — col i = 관절 J_i (i = 0..5)
- 각 열의 의미: "현재 자세에서 관절 i를 미소 회전시켰을 때 EE pose가 어느 방향으로 얼마나 움직이는가."

행 레이아웃을 A-02 pose error 벡터와 일치시켰기 때문에, A-04에서 `Δq = f(J, Δx)` 형태로
pose error 벡터 Δx를 그대로 J와 결합할 수 있다(단위·순서 변환 불필요).

## Finite difference 수식 (전진차분)

```
current = FK(State)                              // ComputeEndEffectorTransform
for i in 0..5:
    StatePlus     = State                        // 로컬 복사본 (입력 State 불변)
    StatePlus.Q[i] += epsilon
    plus  = FK(StatePlus)
    delta = ComputePoseError(current, plus)      // = plus − current, 6-벡터
    for row in 0..5:
        J[row][i] = delta[row] / epsilon
```

`FRobotPoseError::ComputePoseError(Current, Target)` 는 `Target − Current` 규약이므로
`ComputePoseError(current, plus)` 는 정확히 `plus − current`(전진차분 분자)가 된다.
회전 성분은 A-02의 **quaternion-log rotation vector를 그대로** 쓴다 — Euler 각 차이를 쓰지 않는다.

전진차분(1차, 절단오차 O(ε))을 택한 이유: 검증 기준이면 충분한 정확도이고, 중심차분 대비 FK 호출이
절반(관절당 1회)이라 단순하다. 필요하면 이후 중심차분(O(ε²))으로 정밀도를 높일 수 있다.

## Epsilon 선택 근거

- 기본값 **1e-4 rad** (≈ 0.0057°).
- 유한차분 총 오차 ≈ (절단오차 ~O(ε)) + (반올림오차 ~O(machine_eps / ε))의 합이라, 너무 크면
  곡률에 의한 절단오차가, 너무 작으면 double 반올림 잡음이 지배한다. double(machine_eps ≈ 2.2e-16)
  기준으로 1e-4 부근이 두 오차의 균형점에 가깝다.
- 1e-4에서 위치 도함수 오차는 위치 tolerance(1e-3 cm)보다 충분히 작게 유지된다.
  (1e-2는 절단오차↑, 1e-8은 반올림 잡음↑.)
- **안전장치**: `EpsilonRad`가 0 이하이거나 지나치게 작은 값(≤ 1e-9)이면 기본값 1e-4로 대체한다.

## A-02 quaternion-log pose error와의 연결

회전 열을 만들 때 두 자세의 차이를 `ComputePoseError`에 위임한다. 이 함수는
`q_err = Target * Current⁻¹` 의 shortest-path axis-angle rotation vector(= quaternion log × 2)를
돌려주므로, 회전 도함수가 Euler wrap-around/gimbal 특이 없이 연속적이다.
따라서 J의 회전 행(3~5)은 자연스럽게 각속도 성분(rad/rad)의 의미를 가진다.

## 다음 STEP_A-04 DLS IK에서 J 사용 (예고)

A-04는 이 J와 A-02 pose error 벡터 Δx를 받아, Damped Least Squares로 관절 증분을 구한다:

```
Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ Δx
```

λ(damping)은 특이점 근처에서 해가 발산하지 않게 한다. J가 검증돼 있으므로 A-04의 초점은
λ 튜닝·수렴·관절 한계 처리에 맞춰진다.

## 한계

- **느림**: 관절당 FK를 1회씩(총 6회 + 기준 1회) 호출하는 수치 미분이라 analytic보다 느리다.
  실시간 IK 루프에서는 부담이 될 수 있다.
- **전진차분 편향**: 1차라 O(ε) 절단오차가 있다(중심차분이면 O(ε²)).
- 그러나 **검증 기준으로는 이상적**이다. 이후 analytic/geometric Jacobian을 구현하면 성분별로
  이 numerical J와 비교해 유도 정확성을 확인할 수 있다.

## 변경/추가 파일

- 신규 `Public/Robot/RobotJacobian.h` — `FRobotJacobian6x6`, `FRobotJacobian::ComputeNumericalJacobian`
- 신규 `Private/Robot/RobotJacobian.cpp`
- 신규 `Private/Tests/RobotJacobianTests.cpp` — `RobotSim.Jacobian.*` 테스트
- 신규 `Docs/Steps/STEP_A-03.md` (본 문서)
- 수정 `Public/Robot/Serial6DoFRobotActor.h` — CallInEditor `LogCurrentNumericalJacobian()` 선언
- 수정 `Private/Robot/Serial6DoFRobotActor.cpp` — 구현 + include

## 테스트 (`RobotSim.Jacobian.*`)

| 테스트 | 검증 | 기대값 |
|---|---|---|
| `ShapeAndFinite` | 36개 원소 모두 finite | — |
| `J0YawPositionDerivative` | Q=0, col0 위치 행 | ≈ (0, 105, 0) cm/rad = ẑ × (105,0,120) |
| `J5RollPositionInvariant` | Q=0, col5 위치 행 크기 | ≈ 0 (ToolOffset이 J5 축 +X와 공선) |
| `J5RollRotationDerivative` | Q=0, col5 회전 행 | ≈ (1, 0, 0) rad/rad (world +X 축) |
| `NoMutation` | 호출 전후 입력 State.Q 불변 | 불변 |

기존 `RobotSim.Kinematics.*`, `RobotSim.PoseError.*` 는 수정하지 않았다(회귀 없음).
