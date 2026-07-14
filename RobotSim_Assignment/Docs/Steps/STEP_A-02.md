# Step A-02: 6D Pose Error 순수 수학 레이어

## 목표

6-DOF FK 모델의 End Effector Transform과 목표 Transform 사이의 **6D pose error**를 계산하는
순수 수학 레이어를 추가한다. 이 단계에서는 IK / Jacobian / DLS / 마우스 드래그 UI / pick·place를
구현하지 않는다. 오차 표현과 그 계산만 분리해 테스트로 고정한다.

## 왜 IK 전에 pose error를 먼저 분리했는가

IK(A-03 Jacobian → A-04 DLS)는 "현재 EE 자세를 목표로 얼마나 움직여야 하는가"를 나타내는
6D 오차 벡터 Δx를 입력으로 받는다. IK 수렴 로직과 이 오차 표현을 한꺼번에 구현하면,
결과가 틀렸을 때 원인이 **오차 정의 오류**인지 **수렴 실패**인지 분리할 수 없다.

A-1에서 FK를 IK보다 먼저 검증한 것과 같은 이유다. FK가 검증된 위에 오차 표현을 먼저
독립적으로 고정해 두면, A-03 이후의 디버깅은 순수하게 "수렴/스텝" 문제로 좁혀진다.
pose error는 UObject/World에 의존하지 않는 순수 함수(`FRobotPoseError`)로 만들어,
액터 스폰 없이 단위테스트로 못박았다.

## 단위 정책: 위치 cm, 회전 radian

A-1의 규약(내부 계산은 radian/double, degree는 에디터 경계에서만)을 그대로 계승한다.

- **위치 오차: Unreal cm** — FK가 cm로 위치를 다루므로 오차도 cm. 별도 스케일 변환이 없어야
  단위 혼동 버그가 생기지 않는다.
- **회전 오차: radian** — 각속도·Jacobian의 자연 단위가 rad이며, axis-angle rotation vector의
  크기가 곧 회전각(rad)이 되어 물리적 의미가 분명하다. degree는 로그 출력에서만 병기한다.
- **가중치 적용 전/후 구분**: `ComputePoseError`는 raw(가중치 없음) 값을,
  `ComputeWeightedPoseError`는 위치·회전에 각 스칼라 가중치를 곱한 값을 반환한다.
  IK에서 위치 mm급 정밀도와 회전 도급 정밀도의 균형을 맞추려면 두 단위가 서로 다른 스케일이라
  가중치가 필요하지만, 이 단계에서는 그 값을 계산·구분만 하고 사용하지는 않는다.

## 왜 Euler 각 차이가 아니라 quaternion log / axis-angle인가

회전 오차를 `TargetEuler − CurrentEuler`로 계산하면 안 된다:

- **Gimbal lock / 축 순서 의존**: Euler(Roll/Pitch/Yaw) 성분 차이는 특정 자세에서 특이해지고,
  같은 회전이라도 표현이 여러 개라 성분별 차이가 불연속적으로 튄다.
- **Wrap-around**: 179° → −179°는 실제로 2°만 돈 것이지만 Euler 성분 차이는 358°로 나온다.

대신 두 회전의 **상대 회전을 quaternion으로 구한 뒤 axis-angle(= quaternion log × 2)**로 변환한다.
이는 회전 매니폴드 SO(3) 위의 최단 측지선을 매끄럽게 주며, 벡터의 방향 = 회전축,
크기 = 회전각(rad)이라 그대로 IK의 각속도 오차로 쓸 수 있다.

### 계산식

```
q_err   = q_target * q_current⁻¹        // 월드 프레임 상대 회전
if q_err.W < 0:  q_err = -q_err          // shortest path: W≥0 반구 선택 → 각도 ∈ [0, π]
sinHalf = |vec(q_err)|  (= sin(θ/2) ≥ 0)
cosHalf = q_err.W       (≥ 0)
if sinHalf < UE_SMALL_NUMBER:  r_err = 2·vec(q_err)   // θ≈0: 나눗셈 회피 small-angle 근사
else:
    θ     = 2·atan2(sinHalf, cosHalf)    // ∈ [0, π]
    r_err = (vec(q_err) / sinHalf) · θ   // axis(단위) × angle
```

- **월드 프레임 `q_target * q_current⁻¹`**: A-03 geometric Jacobian이 월드 프레임 각속도를
  다루는 것과 정합시키기 위한 선택.
- **Shortest path**: q와 −q는 같은 회전이지만 반구가 다르다. W≥0 반구를 고르면 회전각이
  [0, π]로 제한되어 항상 최단 경로가 된다. (예: 목표를 270°로 표현해도 −90°와 같은 오차)
- **180° 근처 안정성**: `atan2(sinHalf, cosHalf)`를 쓰면 `acos(W)`의 정의역 초과(부동소수 W>1)로
  인한 NaN이 원천 차단된다. 180° 근방에서는 sinHalf≈1, cosHalf≈0이라 분모(sinHalf)가 커서
  나눗셈이 오히려 안정적이다. θ≈0 특이점만 small-angle 근사(2·vec)로 처리한다.

## 다음 단계와의 연결 (A-03 / A-04 입력 벡터)

`FRobot6DPoseError::ToArray6()`는 오차를 `[px, py, pz, rx, ry, rz]` 6-벡터로 내보낸다.
이것이 곧 다음 단계의 입력이다:

- **A-03 Jacobian**: EE의 6×6 geometric Jacobian J를 구성하면, 관절 속도와 EE 속도의 관계는
  `Δx = J · Δq`. 여기서 `Δx`가 바로 이 6D pose error 벡터다.
- **A-04 DLS IK**: `Δq = Jᵀ(JJᵀ + λ²I)⁻¹ · Δx`로 관절 증분을 구할 때, 오른쪽 `Δx`에
  이 pose error(필요시 가중치 적용본)를 그대로 넣는다.

즉 이번 단계의 산출물은 IK 루프의 "오차 측정" 절반을 미리 완성해 고정한 것이며,
A-03/04에서는 J 구성과 수렴만 새로 다루면 된다.

## Skeletal Mesh visual과 수학 FK의 역할 분리 (재확인)

pose error는 **수학 FK(`FSerial6DoFModel`) 결과**로부터만 계산한다. SkeletalMesh 본 리타겟
비주얼은 수학 결과를 따라가 표시하는 레이어일 뿐이며 source of truth가 아니다. 따라서
오차 계산은 본 매핑 상태(미매핑 관절 유무 등)와 완전히 독립이다. 이번 단계는 본 매핑을
다시 건드리지 않았고, 오차의 기준은 언제나 수학 FK와 디버그 ToolTip이다.

## 구현 요약

### 변경 / 추가 파일

| 파일 | 내용 |
|---|---|
| `Public/Robot/RobotTypes.h` (수정) | `FRobot6DPoseError` 타입 추가 (plain struct, `ToArray6`/norm 헬퍼) |
| `Public/Robot/RobotPoseError.h` (신규) | 순수 수학 함수 선언 `FRobotPoseError` |
| `Private/Robot/RobotPoseError.cpp` (신규) | 회전 오차 axis-angle 계산 구현 |
| `Public/Robot/Serial6DoFRobotActor.h` (수정) | `TargetEndEffectorWorld` 프로퍼티 + CallInEditor 함수 선언 |
| `Private/Robot/Serial6DoFRobotActor.cpp` (수정) | `LogCurrentEndEffectorPoseErrorToTarget()` 구현 |
| `Private/Tests/RobotPoseErrorTests.cpp` (신규) | Automation Test 6종 |
| `Docs/Steps/STEP_A-02.md` (신규) | 본 문서 |

기존 `FSerial6DoFModel` FK, joint limit, link offset, tool offset, 골든 FK 테스트,
SkeletalMesh 본 매핑은 **변경하지 않았다**.

### API

```cpp
// 순수 수학 (UObject/World 비의존)
FVector           FRobotPoseError::ComputeRotationError(const FQuat& Current, const FQuat& Target);
FRobot6DPoseError FRobotPoseError::ComputePoseError(const FTransform& Current, const FTransform& Target);
FRobot6DPoseError FRobotPoseError::ComputeWeightedPoseError(
                      const FTransform& Current, const FTransform& Target,
                      double PositionWeight, double RotationWeight);

// 디버그 (액터, CallInEditor) — TargetEndEffectorWorld에 대한 현재 EE 오차를 로그
void ASerial6DoFRobotActor::LogCurrentEndEffectorPoseErrorToTarget();
```

## 검증 결과

Automation Test 6종 (`RobotSim.PoseError.*`):

| 테스트 | 검증 내용 |
|---|---|
| `RobotSim.PoseError.Identity` | 같은 Transform → 위치·회전 오차 모두 0 |
| `RobotSim.PoseError.PositionOnly` | 위치만 10cm 차이 → 위치 오차 (10,0,0), 회전 오차 0 |
| `RobotSim.PoseError.Rotation90` | +Z 90° 목표 → 회전 오차 크기 ≈ π/2, 벡터 ≈ (0,0,π/2), 위치 오차 0 |
| `RobotSim.PoseError.ShortestPath` | 270°/−90° 표현이 동일 오차 (0,0,−π/2), 크기 3π/2 아님 |
| `RobotSim.PoseError.NearPiStable` | 180°±ε 회전 → 성분 유한(NaN/Inf 없음), 크기 ≈ π |
| `RobotSim.PoseError.Weighted` | weighted = raw × 가중치 (위치·회전 독립) |

기존 `RobotSim.Kinematics.*` 5종은 회귀 없이 그대로 통과해야 한다 (FK 미변경).
