# Step A-06.1: End Effector ToolOffset Calibration 검증

## 목표

디버그 EE / 기즈모 / 빨간선의 기준점을 **실제 KUKA 그리퍼 끝**에 맞춘다.
기구학(`LinkOffsets` / `JointAxes`)은 건드리지 않고 **ToolOffset만** 보정한다.

A-06에서 `CalibrateToolOffsetFromTarget()`으로 캘리브레이션 자체는 이미 구현했다.
이 단계는 **계산식을 바꾸지 않고**, 캘리브레이션이 실제로 맞았는지 로그만으로 확인할 수 있도록
**검증 로그**를 추가한다.

## 문제 현상

`SpawnOrAlignTargetToCurrentEndEffector` / `DrawDebugJointFrames` / `SolveIKToTarget`은 모두
`Model.ComputeEndEffectorTransform(CurrentState)`를 EE 기준으로 쓴다. 이 값은 정의상
**J5 frame + `Model.ToolOffset`**이다.

`ToolOffset` 기본값이 `FTransform(FQuat::Identity, FVector(10, 0, 0))` (`Serial6DoFModel.cpp:46`)
— 즉 J5에서 +X로 10cm — 인데 이는 KUKA 메시의 실제 집게 끝과 맞지 않는다. 그래서 기즈모/빨간선/
TargetActor 기준점이 실제 그리퍼 끝이 아니라 **디버그 링크 쪽**에 잡혀 보인다.

## 왜 기구학 문제가 아니라 ToolOffset 문제로 분리했는가

`JointAnglesDeg`를 조작하면 **관절과 SkeletalMesh는 정상적으로 움직인다.**

이는 다음이 모두 정상임을 뜻한다:
- 관절축 (`JointAxes`)
- FK 체인 (`ComputeJointWorldTransform`)
- IK solver (`FRobotDlsIK`)
- Input / interaction 레이어

증상은 **J5까지의 체인이 아니라 J5 이후의 툴 기준점**에만 국한된다.
따라서 수정 범위를 ToolOffset calibration으로 제한한다.

## 왜 LinkOffsets를 KUKA 메시 기준으로 바꾸지 않았는가

KUKA 메시를 기하학적 기준으로 삼아 `FSerial6DoFModel::LinkOffsets`를 바꾸는 방향은 **채택하지 않는다.**

- A-01~A-05의 FK/IK 수학 모델과 **골든 테스트가 이미 현재 LinkOffsets 기준으로 검증**되어 있다.
  특히 `X = 60 + 20 + 15 + 10 = 105` 상수가 `RobotKinematicsTests.cpp:59,85,98,122,130,177`과
  `RobotJacobianTests.cpp:80`에 하드코딩되어 있다 (끝의 `10`이 곧 `ToolOffset.X`).
- LinkOffsets 변경은 **로봇 전체 기구학 변경**이라 기존 테스트와 문서를 모두 다시 갱신해야 한다.
- 관절 움직임 자체는 정상이므로, 로봇 체인을 바꿀 근거가 없다.

**테스트 안전성**: 자동화 테스트는 `FSerial6DoFModel::CreateDefault()`를 직접 호출하고 `URobotConfig`를
거치지 않는다. 따라서 **RobotConfig 에셋의 ToolOffset을 캘리브레이션해도 테스트는 그대로 통과**한다.
반대로 `CreateDefault()`의 ToolOffset을 바꾸면 `RobotSim.Kinematics.*` / `RobotSim.Jacobian.*`이 깨진다 —
이번 단계에서 건드리지 않는 이유다.

## 계산식

FK 합성 규약은 UE의 child-first 순서를 따른다 (`Serial6DoFModel.cpp:66-69`):

```
EE_model = ToolOffset * J5_model
```

따라서 원하는 EE(= 사용자가 배치한 target)를 만족하는 ToolOffset은 역산으로 구한다:

```cpp
const FTransform TargetWorld = EndEffectorTargetActor->GetActorTransform();  // 월드
const FTransform TargetModel = TargetWorld.GetRelativeTransform(GetActorTransform());
const FTransform J5Model     = Model.ComputeJointWorldTransform(FSerial6DoFModel::NumJoints - 1, CurrentState);
const FTransform NewToolOffset = TargetModel.GetRelativeTransform(J5Model);
```

- `TargetModel` — 로봇 모델 공간에서 본 원하는 EE 위치/자세
- `J5Model` — 현재 관절 상태에서의 J5 프레임 (모델 공간, ToolOffset **미포함**)
- `NewToolOffset` — J5 프레임에서 실제 그리퍼 끝까지의 상대 변환

`ComputeJointWorldTransform`은 이름과 달리 **모델 공간**(`BaseTransform` 기준)을 반환하므로,
월드 target은 반드시 `GetRelativeTransform(GetActorTransform())`으로 내려야 한다.

## A-06 대비 이번 단계의 변경점

계산식과 적용 정책은 **A-06 그대로다.** 추가된 것은 검증 로그뿐이다.

| 항목 | A-06 | A-06.1 |
|---|---|---|
| 계산식 | `TargetModel.GetRelativeTransform(J5Model)` | 동일 (무수정) |
| RobotConfig 적용 정책 | `Modify` → `ToolOffset` → `MarkPackageDirty` → `RefreshFromConfig` | 동일 (무수정) |
| 로그 | 새 ToolOffset 1줄만 | 적용 전/후 전체 + 오차 검증 |
| 미리보기 | 없음 | `LogToolOffsetCalibrationState()` 신규 |

신규 함수 `CalibrateToolOffsetFromTargetActor()`는 **만들지 않았다.** 기존
`CalibrateToolOffsetFromTarget()`이 이미 동일한 계산식과 적용 정책을 갖고 있어, 새로 만들면
Details 패널에 같은 기능의 버튼이 두 개 생길 뿐이다.

## 적용 정책

**RobotConfig가 있으면** — DataAsset에 영구 기록:

```cpp
RobotConfig->Modify();
RobotConfig->ToolOffset = NewToolOffset;
RobotConfig->MarkPackageDirty();
RefreshFromConfig();
```

사용자가 **에셋을 저장(Ctrl+S)해야** 유지된다.

**RobotConfig가 없으면** — 런타임 모델에만 일시 적용:

```cpp
Model.ToolOffset = NewToolOffset;
MirrorModelToComponents();
ApplyJointState();
```

`OnConstruction`/`BeginPlay` 재구성 시 `CreateDefault()`로 되돌아가므로 Warning을 남긴다.
영구 반영하려면 RobotConfig 사용이 권장된다.

## 검증 로그

`CalibrateToolOffsetFromTarget()`이 `LogRobotSim`으로 출력하는 항목:

**적용 전** — 기존 ToolOffset / 새 ToolOffset / J5Model / TargetModel (각각 위치 cm + 회전 도)
**적용 후** — MathEEWorld / TargetWorld / EE↔Target 위치오차 cm / 회전오차 도

`LogToolOffsetCalibrationState()`는 위와 같은 값을 **적용 없이** 출력하며,
"적용했다면 어떤 ToolOffset이 될지"까지 미리 보여준다.

오차 계산은 직접 하지 않고 순수 수학 레이어 `FRobotPoseError::ComputePoseError`를 재사용한다 —
`LogCurrentEndEffectorPoseErrorToTarget()`과 동일한 경로다.

### 적용 후 오차가 0이 아닐 수 있는 이유

`RefreshFromConfig()`는 내부에서 `ApplyAnglesFromEditor()`를 호출해 `CurrentState`를 `JointAnglesDeg`
로부터 **clamp 포함해 재계산**한다 (`Serial6DoFRobotActor.cpp:443-452`). 캘리브레이션은 재계산 **전**
`CurrentState`로 ToolOffset을 역산하므로, 두 값이 어긋나 있으면 적용 후 오차가 0이 아니게 된다.

이 드리프트를 드러내는 것이 검증 로그의 핵심 목적이므로, **오차를 억지로 0으로 맞추지 않고 있는 그대로
출력**한다. 오차가 크게 나오면 관절 각도가 limit에 걸려 clamp되었는지 의심할 것.

## 캘리브레이션 절차

1. 로봇을 **홈 자세**(모든 `JointAnglesDeg` = 0)로 둔다.
   ToolOffset은 현재 자세 기준으로 역산되므로 자세가 재현 가능해야 한다.
2. Details → `SpawnOrAlignTargetToCurrentEndEffector` 로 TargetActor 생성.
3. TargetActor를 **실제 KUKA 그리퍼 끝**으로 이동하고, **회전도 실제 툴 방향에 맞춘다**
   — ToolOffset은 위치와 회전을 모두 담는다.
4. (선택) `LogToolOffsetCalibrationState` 실행 → 현재 어긋남과 적용될 값을 미리 확인.
5. `CalibrateToolOffsetFromTarget` 실행 → 로그에서 **적용 후 위치오차 ≈ 0cm, 회전오차 ≈ 0도** 확인.
6. 뷰포트에서 빨간선/디버그 EE가 그리퍼 끝으로 이동했는지 육안 확인.
7. RobotConfig 에셋 저장(Ctrl+S).

## 한계

- **ToolOffset은 J5 이후의 고정 툴 변환만 보정한다.** 그 앞의 링크 기하는 손대지 않는다.
- **로봇 링크 길이 전체를 KUKA 메시 비율에 맞추는 것은 별도 calibration 단계**다.
  현재 수학 모델의 링크 길이는 메시 실측이 아니라 A-01에서 정한 값이다.
- **SkeletalMesh가 수학 모델과 1:1 기구학이 아니면 모든 자세에서 완벽히 일치하지 않는다.**
  KUKA 메시는 순수 6R 체인이 아니고 J3이 unmapped 상태(A-1.5b)라, ToolOffset 하나로는
  **캘리브레이션한 자세에서 가장 정확**하고 다른 자세에서는 오차가 남는다.

## 변경 파일

- 수정 `Public/Robot/Serial6DoFRobotActor.h` — `LogToolOffsetCalibrationState()` 선언 추가,
  `CalibrateToolOffsetFromTarget()` 주석에 검증 로그 설명 보강
- 수정 `Private/Robot/Serial6DoFRobotActor.cpp` — 익명 네임스페이스에 `LogTransformLine` /
  `LogPoseErrorLine` 헬퍼 추가, `CalibrateToolOffsetFromTarget()`에 적용 전/후 검증 로그 추가,
  `LogToolOffsetCalibrationState()` 구현
- 신규 `Docs/Steps/STEP_A-06.1.md`

**무수정**: `FSerial6DoFModel`(LinkOffsets/JointAxes/JointLimits/CreateDefault), `Private/Tests/*` 전체,
IK solver, SkeletalMesh retarget, Input/PlayerController, TargetActor 드래그 로직, `STEP_A-06.md`
(함수명이 유지되므로 기존 참조가 그대로 유효).
