# Step A-05: Nullspace Joint-Limit Avoidance

## 목표

STEP_A-04의 DLS IK solver(`FRobotDlsIK::SolveDlsIK`)에 **nullspace joint-limit avoidance** 보조 항을
추가한다. End Effector 목표 pose를 그대로 추종하면서도, 관절이 가동 한계에 가까워질수록 중립(midpoint)
자세 쪽으로 되돌리는 2차 목표를 primary task의 **nullspace 방향으로만** 주입한다.
이 단계에서는 **마우스 드래그 UI, pick/place FSM, 경로·애니메이션 보간, Skeletal Mesh 리타겟 수정,
analytic IK, 모델 파라미터(축/오프셋/한계) 변경을 하지 않는다.** 기존 DLS solver에 선택 가능한 옵션을
얹고, 순수 수학 구현 + 최소 테스트 + 문서화까지만 한다. 옵션 기본값이 꺼져 있어 STEP_A-04 동작은 불변이다.

## 왜 nullspace가 필요한가

DLS 반복은 매 스텝 오차를 줄이는 방향으로 관절을 밀 뿐, 관절이 한계 쪽으로 몰리는지는 신경 쓰지 않는다.
`MaxStepRad` clamp와 numerical Jacobian 특성상 특정 관절이 한계에 붙기 쉽고, 한계에 붙으면
`ClampToLimits` 때문에 그 방향 자유도가 죽어 이후 수렴이 나빠진다.

해결책은 "EE pose를 지키는 것"을 **primary task**로, "관절을 중립으로 되돌리는 것"을 **secondary task**로
두고, secondary가 primary를 방해하지 않도록 **primary task의 nullspace(영공간)** 안에서만 관절을
움직이는 것이다. task-space에 영향이 0인 방향으로만 관절 자세를 재배치하므로 pose 정확도를 해치지 않는다.

## primary task와 secondary task의 차이

- **primary (task-space)**: EE 6D pose를 목표로 맞춘다. 항상 우선한다. `dq_task = Jwᵀ(Jw Jwᵀ+λ²I)⁻¹ e`.
- **secondary (joint-space)**: 관절을 midpoint로 되돌린다. primary가 남긴 자유도(nullspace)로만 실현된다.
  nullspace projector `N`을 통과한 성분만 살아남으므로, task 오차에 영향을 주지 않는 범위에서만 작동한다.

## DLS pseudo-inverse와 nullspace projector 수식

```
J#  = Jᵀ (J Jᵀ + λ² I)⁻¹        (DLS damped pseudo-inverse)
N   = I − J# J                    (nullspace projector, task-space에 영향 0인 방향)
dq  = dq_task + N · dq_null       (최종 관절 증분)
```

- `dq_task = J# e` (STEP_A-04에서 이미 `A = J Jᵀ + λ²I`, `A y = e`, `dq_task = Jᵀ y`로 계산 중).
- nullspace를 위해서는 `J#` 전체(6×6)가 필요하다. 6×6 역행렬을 명시적으로 만들지 않고, 각 basis
  vector `e_k`에 대해 `A z_k = e_k`를 STEP_A-04의 `SolveLinearSystem6`로 풀어 `A⁻¹`의 열을 채우고
  (`A`가 대칭이라 `z_k`가 그대로 유효), `J# = Jᵀ A⁻¹`로 구성한다. 그다음 `N = I − J# J`,
  `dq_null_proj = N · dq_null_desired`.
- basis 풀이가 하나라도 특이하면 `dq_null = 0`으로 두고 이번 반복의 nullspace 항을 건너뛴다(안전).

## joint limit normalized distance 정의

각 관절 `i`에 대해:

```
midpoint_i  = (min_i + max_i) / 2
halfRange_i = (max_i − min_i) / 2
normalized_i = (q_i − midpoint_i) / halfRange_i     // 0 = 중립, ±1 = 한계
```

`normalized`는 관절이 가동 범위의 어디쯤인지를 [−1, +1]로 정규화한 무차원 거리다. 관절마다 가동 범위가
크게 다르므로(예: J1 ±120° vs J3 ±350°), 정규화해야 서로 다른 관절을 같은 척도로 비교·회피할 수 있다.
진단값 `MaxAbsNormalizedJointDistance = max_i |normalized_i|`는 "가장 한계에 붙은 관절"의 정도를 한 숫자로 준다.

## midpoint 방향 gradient를 쓰는 이유

중립 자세로 돌아가려는 desired nullspace 관절 속도를 다음처럼 정의한다:

```
|normalized_i| < JointLimitActivationRatio  →  v_i = 0
그 이상                                       →  v_i = −NullspaceGain · normalized_i
```

- `−normalized` 부호는 항상 **midpoint를 향하는 방향**이다(상한 근처면 음수, 하한 근처면 양수).
  이는 관절 한계 회피 cost `c = ½ Σ normalized_i²`의 음의 gradient 방향과 일치한다.
- **activation ratio(기본 0.65)** 미만인 중앙부에서는 `v_i = 0`으로 둔다. 여유가 있는 관절까지 굳이
  당기면 nullspace 자유도를 낭비하고 task 수렴을 방해할 수 있으므로, **한계 근처에서만** 회피를 켠다.
- 한계에 가까울수록 `|normalized|`가 커져 되돌림 속도도 비례해 강해진다.

## weighting된 J와 nullspace projection의 일관성

STEP_A-04는 위치(cm)와 회전(rad) 스케일 차이를 보정하려고 오차 `e`와 Jacobian에 동일한 row scaling을
적용한 **`Jw`**(행 0~2 ×`PositionWeight`, 행 3~5 ×`RotationWeight`)를 쓴다. nullspace projection도
**같은 `Jw`와 `A = Jw Jwᵀ + λ²I`** 로 `J#`, `N`을 계산해야 primary task와 정의가 일관된다. 서로 다른
스케일의 J를 섞으면 nullspace가 실제 task-space와 어긋나 pose를 미세하게 밀 수 있다.
반면 `dq_null_desired(v)`는 **관절공간 속도**이므로 weighting을 적용하지 않는다.

## MaxStepRad와 joint limit clamp 유지

nullspace 항을 더한 뒤에도 STEP_A-04의 안전 정책을 그대로 지킨다:

```
dq = dq_task + N·dq_null   →  각 성분을 ±MaxStepRad로 clamp  →  State.Q += dq  →  (옵션) ClampToLimits
```

즉 합산된 최종 `dq`에 스텝 제한과 관절 한계 clamp가 모두 적용되어, nullspace가 켜져도 관절이 한계를
넘거나 선형 근사가 깨질 만큼 큰 스텝을 밟지 않는다.

## 추가 옵션값 (`FRobotDlsIKOptions`)

| 필드 | 기본값 | 의미 |
|---|---|---|
| `bUseNullspaceJointLimitAvoidance` | false | nullspace 항 활성화 여부 (false면 A-04와 동일) |
| `NullspaceGain` | 0.05 | 중립으로 되돌리는 보조 속도 크기 |
| `JointLimitActivationRatio` | 0.65 | 이 normalized 거리 미만이면 회피 0 |

## 결과 진단값 (`FRobotDlsIKResult`)

| 필드 | 의미 |
|---|---|
| `bNullspaceUsed` | 이번 solve에서 nullspace 항을 실제 적용했는지 |
| `MaxAbsNormalizedJointDistance` | 최종 상태 max \|normalized\| (0=중립, 1=한계) |
| `NullspaceStepNorm` | 마지막 반복의 `N·dq_null` 크기 |

## 변경/추가 파일

- 수정 `Public/Robot/RobotDlsIK.h` — 옵션 3필드, 결과 3필드, static 헬퍼 2개(`ComputeJointLimitAvoidanceVelocity`, `ComputeMaxAbsNormalizedJointDistance`) 선언
- 수정 `Private/Robot/RobotDlsIK.cpp` — anonymous namespace `ProjectNullspaceStep`, solve 루프에 nullspace 통합, 헬퍼 구현, 로그 확장
- 신규 `Private/Tests/RobotNullspaceTests.cpp` — `RobotSim.IK.Nullspace.*` 테스트
- 수정 `Public/Robot/Serial6DoFRobotActor.h` — 디버그 토글 `bUseNullspaceJointLimitAvoidance` UPROPERTY
- 수정 `Private/Robot/Serial6DoFRobotActor.cpp` — `SolveIKToTarget`에서 토글 반영 + nullspace 로그
- 신규 `Docs/Steps/STEP_A-05.md` (본 문서)

## 에디터에서 nullspace on/off 확인법

1. 레벨의 로봇 액터(`ASerial6DoFRobotActor`)를 선택한다.
2. `Robot|PoseError > TargetEndEffectorWorld`에 목표 EE Transform을 입력한다.
3. `Robot|IK > bUseNullspaceJointLimitAvoidance`를 **끈** 상태로 **SolveIKToTarget**을 눌러 로그의
   `nullspace=미사용, max관절편차 X.XXX`를 기록한다.
4. 관절을 한계 근처로 옮긴 뒤 같은 target에 대해 토글을 **켜고** 다시 눌러 `nullspace=사용, max관절편차`를
   비교한다. nullspace가 유효한 자세(특이점 근처 등)에서는 max 관절 편차가 줄어드는 것을 확인할 수 있다.

## 테스트 (`RobotSim.IK.Nullspace.*`)

| 테스트 | 검증 | 기대 |
|---|---|---|
| `DisabledByDefault` | 옵션 기본값 | `bUseNullspaceJointLimitAvoidance == false` |
| `GradientPointsTowardMidpoint` | J0 상/하한 근처 회피 속도 | 상한→음수, 하한→양수 |
| `NoActivationNearMidpoint` | 모든 관절 midpoint | 회피 속도 ≈ 0 |
| `SolverKeepsFinite` | nullspace on + 도달 불가 target | Q·오차·진단값 전부 finite |
| `JointLimitDistanceDoesNotWorsenOnRedundantOrWeakTask` | 한계 근처 초기 + 약한 task, on vs off | on의 max관절편차 ≤ off (악화 없음) |

방향성과 finite 안정성 위주로 느슨하게 검증한다. 기존 `RobotSim.Kinematics.*`, `RobotSim.PoseError.*`,
`RobotSim.Jacobian.*`, `RobotSim.IK.*`는 수정하지 않았다(회귀 없음).

## 한계

- **비여유(non-redundant) 로봇**: 6R 로봇 + 6D task는 정확 해 근처에서 `J`가 full-rank라 nullspace가
  사실상 비어(`N ≈ 0`) nullspace 항 효과가 작다. 실질 효과는 주로 **특이점 근처**(DLS damping이 유효
  nullspace를 만드는 영역)에서 나타난다.
- **계산량**: numerical Jacobian(매 반복 FK 7회)에 더해, nullspace를 켜면 `A⁻¹` 구성을 위해 6×6 선형계를
  6회 더 푼다. analytic 대비 무겁고 실시간 루프에 부담이다.
- **과도한 gain 위험**: `NullspaceGain`이 크면 nullspace step이 커져 task 수렴을 방해하거나 진동할 수 있다.
- **완전한 최적화 solver가 아님**: 이 항은 관절 한계 cost의 1차 gradient를 nullspace에 투영하는 **실시간
  데모용 안정화 항**이며, 전역 최적 자세를 보장하지 않는다.

## 다음 단계로 확장 가능한 것

- **target 조작 UI**: 뷰포트에서 목표 Transform을 gizmo로 드래그하며 실시간 IK.
- **analytic/geometric Jacobian 비교**: numerical Jacobian을 해석적 Jacobian으로 대체해 정확도·속도 비교.
- **가중 nullspace / adaptive gain**: 관절별 가중치나 오차 크기에 따른 동적 gain으로 회피 강도 조절.
