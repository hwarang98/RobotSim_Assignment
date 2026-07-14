# Step A-04: Damped Least Squares IK Solver

## 목표

STEP_A-02의 6D pose error와 STEP_A-03의 numerical Jacobian을 조합해, 6-DOF serial robot의
End Effector가 목표 Transform에 가까워지도록 관절 상태를 반복 갱신하는 **Damped Least Squares(DLS)
IK solver**를 순수 수학 레이어로 추가한다.
이 단계에서는 **마우스 드래그 UI, pick/place FSM, 경로·애니메이션 보간, Skeletal Mesh 리타겟 수정,
analytic IK를 구현하지 않는다.** 단일 목표에 대한 반복 solve와 에디터 디버그 함수까지만 한다.

## IK 문제 정의: e = target − current

관절 공간 상태 `q`에 대해 FK는 EE 자세 `x = f(q)`를 준다. 목표 자세 `x*`와의 오차를
A-02 pose error로 정의한다(부호 규약 `target − current`):

```
e = ComputePoseError(f(q), x*)   // 6-벡터 [Δpx,Δpy,Δpz (cm), Δrx,Δry,Δrz (rad)]
```

이 `e`를 0으로 만드는 `q`를 찾는 것이 IK다. 1차 근사로 `e ≈ J Δq` (J = A-03 Jacobian)이므로,
매 반복에서 `e`를 줄이는 관절 증분 `Δq`를 구해 `q += Δq`를 반복한다.

## DLS 수식

```
Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e
```

구현에서는 J가 square 6×6이므로, `A = J Jᵀ + λ² I` (6×6)를 만들고
`A y = e`를 6×6 Gaussian elimination(부분 피벗)으로 푼 뒤 `Δq = Jᵀ y`로 되돌린다.

## 왜 pseudo-inverse 대신 DLS인가

가장 단순한 해는 pseudo-inverse `Δq = J⁺ e`다. 하지만 로봇이 **singularity**(관절 정렬로 특정
방향 이동성이 사라지는 자세)에 가까워지면 `J`의 어떤 특이값이 0에 수렴하고, `J⁺`의 대응 항이
**무한대로 폭주**해 관절이 발산한다.

DLS는 `A = J Jᵀ + λ² I`에 damping `λ² I`를 더한다. `λ > 0`이면 `A`는 항상 **양의정부호에
가까워져** 역행렬이 안정적으로 존재하고, 특이 방향의 이득이 `1/σ` 대신 `σ/(σ² + λ²)`로 **유한하게
제한**된다. 즉 특이점 근처에서 정확도를 약간 희생하는 대신 수치적 폭주를 막는다. 이 안정성이
반복 solver의 신뢰성에 결정적이라 pseudo-inverse 대신 DLS를 택했다.

## position(cm)와 rotation(rad) 스케일이 달라 weight를 둔 이유

pose error 6-벡터는 위치(행 0~2, **cm**)와 회전(행 3~5, **rad**)이 **서로 다른 물리 단위**로 섞여
있다. 예를 들어 위치 10cm 오차와 회전 0.17rad(10°) 오차는 크기 숫자만 보면 위치가 압도적으로 커,
가중치 없이 최소제곱을 풀면 solver가 회전 정렬을 거의 무시한다.

이를 보정하려고 오차와 Jacobian **양쪽에 같은 방식으로 row scaling**을 적용한다:

```
e[0..2] *= PositionWeight        J.row[0..2] *= PositionWeight
e[3..5] *= RotationWeight         J.row[3..5] *= RotationWeight
```

기본값은 `PositionWeight = 1.0`, `RotationWeight = 30.0`이다. rad를 30배 키워 1rad ≈ 30cm 정도의
"가중 등가"로 맞춰, 위치와 방향 수렴 속도가 한쪽으로 치우치지 않게 한다. **수렴 판정은** 가중치를
적용하지 않은 원 단위(cm, rad)로 하므로, weight는 스텝 방향에만 영향을 주고 tolerance 의미를
왜곡하지 않는다.

## MaxStepRad를 둔 이유

`e ≈ J Δq`는 현재 자세 주변의 **1차(선형) 근사**라, `Δq`가 크면 근사가 깨져 오히려 오차가 늘거나
진동한다. `MaxStepRad`(기본 `8°`)로 각 관절 증분의 크기를 제한해 매 스텝을 선형 근사가 유효한
범위 안에 묶는다. 도달이 먼 목표에서는 여러 번 나눠 다가가고, 특이점 근처의 큰 `Δq`도 잘라내
안정성을 높인다.

## joint limit clamp 정책

`bClampJointLimits`(기본 true)이면 매 반복 끝에서 `Model.ClampToLimits(State)`로 모든 관절을
가동 범위로 잘라낸다. 물리적으로 도달 불가한 관절값을 IK가 만들어내지 않도록 보장하며, 결과
`Solution`은 항상 `IsWithinLimits`를 만족한다. 초기 상태도 solve 시작 전에 한 번 clamp한다.
clamp는 비파괴 함수라 입력 `InitialState`는 변경하지 않는다.

## 수렴/실패 로그를 남기는 이유

numerical Jacobian + DLS는 초기값·weight·λ에 민감해, "왜 안 됐는지"를 사후에 알 수 있어야 한다.
그래서 다음을 `LogRobotSim`에 남긴다.

- **수렴**: 반복 수, 최종 위치/회전 오차 (Verbose)
- **미수렴(반복 소진)**: 최대 반복 도달, 최종 오차 — 도달 불가/local minimum 신호 (Verbose)
- **선형계 풀이 실패(특이 행렬 추정)**: 피벗이 임계값 미만 → 안전 종료, λ 상향/target 조정 권고 (Warning)
- **Δq NaN/Inf**: 즉시 안전 종료, 마지막 유효 상태 반환 (Warning)

어떤 경우에도 solver는 예외 없이 마지막 유효 `State`와 재계산한 오차를 결과에 채워 반환한다.

## 안전장치 (특이점 / NaN·Inf)

- `A y = e`의 Gaussian elimination에서 부분 피벗 절댓값이 `1e-12` 미만이면 특이로 보고 실패 반환.
  (damping `λ² I` 덕분에 통상 자세에서는 피벗이 이 임계값보다 충분히 크다.)
- 풀이 결과 `y`, 증분 `Δq`에 `!FMath::IsFinite`가 하나라도 있으면 실패 처리하고 이전 유효 상태 유지.

## 기본 옵션값 (`FRobotDlsIKOptions`)

| 필드 | 기본값 | 의미 |
|---|---|---|
| `MaxIterations` | 80 | 최대 반복(무한 루프 방지) |
| `PositionToleranceCm` | 0.5 | 위치 수렴 허용 오차 (cm) |
| `RotationToleranceRad` | 2° | 회전 수렴 허용 오차 (rad) |
| `DampingLambda` | 0.15 | damping λ |
| `MaxStepRad` | 8° | 관절당 스텝 상한 (rad) |
| `PositionWeight` | 1.0 | 위치 오차/행 가중치 |
| `RotationWeight` | 30.0 | 회전 오차/행 가중치 |
| `JacobianEpsilonRad` | 1e-4 | numerical Jacobian 스텝 |
| `bClampJointLimits` | true | 매 반복 관절 한계 clamp |

## 수렴/실패 판정 기준

- **수렴(`bConverged = true`)**: `PositionErrorNorm ≤ PositionToleranceCm` **且** `RotationErrorNorm ≤ RotationToleranceRad`(원 단위, 가중치 미적용).
- **미수렴(`false`)**: 최대 반복 소진, 또는 선형계 특이/NaN으로 안전 종료. 결과에는 언제나 마지막 유효 상태·오차가 담긴다.

## 변경/추가 파일

- 신규 `Public/Robot/RobotDlsIK.h` — `FRobotDlsIKOptions`, `FRobotDlsIKResult`, `FRobotDlsIK::SolveDlsIK`
- 신규 `Private/Robot/RobotDlsIK.cpp` — solve 루프 + anonymous namespace 6×6 `SolveLinearSystem6`
- 신규 `Private/Tests/RobotDlsIKTests.cpp` — `RobotSim.IK.*` 테스트
- 신규 `Docs/Steps/STEP_A-04.md` (본 문서)
- 수정 `Public/Robot/Serial6DoFRobotActor.h` — CallInEditor `SolveIKToTarget()` 선언
- 수정 `Private/Robot/Serial6DoFRobotActor.cpp` — 구현 + include

## 에디터에서 SolveIKToTarget 사용법

1. 레벨에 배치된 로봇 액터(`ASerial6DoFRobotActor`)를 선택한다.
2. Details 패널의 `Robot|PoseError > TargetEndEffectorWorld`에 목표 EE Transform을 입력한다.
   (프레임 규약은 기존 `LogCurrentEndEffectorPoseErrorToTarget`와 동일 — 모델 Transform vs Target 직접 비교.)
3. Details 패널의 `Robot` 카테고리에서 **SolveIKToTarget** 버튼을 누른다.
4. Output Log에서 `LogRobotSim`을 필터링해 수렴/반복/최종 오차를 확인하고, 뷰포트에서 EE가
   목표로 접근하는지 관찰한다. 결과 관절 해는 자동 적용되고 `JointAnglesDeg`(도)도 동기화된다.

## 테스트 (`RobotSim.IK.*`)

| 테스트 | 검증 | 기대 |
|---|---|---|
| `AlreadyAtTarget` | Q=0 FK를 target으로 | 수렴 且 반복 ≤ 1 |
| `SmallReachablePositionOffset` | Q=0에서 Y +10cm | 수렴 또는 위치오차 절반 미만으로 감소 |
| `SmallReachableOrientationOffset` | Q=0에 tool roll 10° | 회전오차 감소 |
| `JointLimitClamp` | 먼 target solve | `IsWithinLimits` 만족 |
| `NoNaNOnDifficultTarget` | 아주 먼/도달 불가 target | Solution.Q·FinalError 전부 finite |

수렴을 빡빡하게 걸지 않고 "오차 감소" 위주로 검증한다(numerical Jacobian + DLS는 초기값/weight에 민감).
기존 `RobotSim.Kinematics.*`, `RobotSim.PoseError.*`, `RobotSim.Jacobian.*`는 수정하지 않았다(회귀 없음).

## 한계

- **느림**: 매 반복 numerical Jacobian(FK 7회)을 다시 계산하므로 analytic 대비 무겁다. 실시간 루프에는 부담.
- **local minimum**: 1차 gradient 계열이라 초기 자세에 따라 지역 최소에 갇힐 수 있다.
- **unreachable target**: 작업공간 밖 목표는 수렴하지 못하고 잔차가 남는다(발산 없이 안전 종료).
- **고정 damping**: λ가 상수라, 특이점 대응과 정확도의 트레이드오프가 고정돼 있다.

## 다음 단계로 확장 가능한 것

- **nullspace joint-limit avoidance**: `Δq = J⁺ e + (I − J⁺ J) z`의 null-space 항 `z`로 EE 자세를
  유지하면서 관절을 한계에서 멀어지게 하는 2차 목표를 추가.
- **adaptive damping**: 오차/조건수에 따라 λ를 동적으로 조절(SVD 기반 selectively damped least squares).
- **target 조작 UI**: 뷰포트에서 목표 Transform을 gizmo로 드래그하며 실시간 IK(별도 단계에서).
