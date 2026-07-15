# RobotSim_Assignment — Step A: 수학으로 구현하는 역기구학(IK)

Unreal Engine **5.7** / C++ 프로젝트.
**Unreal 내장 IK 플러그인(IK Rig, FABRIK, Control Rig 등)을 일절 사용하지 않고**, 6-DOF 시리얼 로봇팔의
Forward Kinematics와 Jacobian 기반 Inverse Kinematics를 순수 수학 레이어로 직접 구현했다.

- **범위**: Step A (FK → pose error → Jacobian → DLS IK → nullspace → target 조작 → ToolOffset 보정)
- **범위 밖**: pick/place FSM, 컨베이어 연동 등 Step B 이후 항목

이 문서는 **진입점 겸 요약**이다. 각 단계의 설계 근거·수식 유도·검증 과정은 [`Docs/Steps/`](Docs/Steps/)의
단계별 문서에 있으며, 아래 각 섹션에서 해당 문서로 링크한다.

---

## 1. 실행 방법

### 1.1 빌드

1. `RobotSim_Assignment.uproject` 를 UE 5.7로 연다 (C++ 프로젝트이므로 최초 실행 시 모듈 빌드 프롬프트가 뜨면 수락).
2. 또는 `RobotSim_Assignment.sln` 을 Visual Studio에서 `Development Editor | Win64` 로 빌드한다.

### 1.2 맵

| 맵 | 경로 | 용도 |
|---|---|---|
| `L_FeatureTest` | `Content/Maps/L_FeatureTest.umap` | **Step A 기능 검증용 테스트 맵** |
| `RobotSimulation_Space` | `Content/Maps/RobotSimulation_Space.umap` | 과제 제공 환경 맵 |

`Config/DefaultEngine.ini` 에서 `L_FeatureTest` 가 이미 `EditorStartupMap` / `GameDefaultMap` 으로 지정돼 있다.
**에디터를 열면 바로 테스트 맵이 로드된다.**

### 1.3 설정 체크리스트

| 설정 항목 | 값 | 지정 위치 |
|---|---|---|
| 로봇 액터 | `ASerial6DoFRobotActor` (또는 `BP_6Robot`) | 맵에 배치 |
| `RobotConfig` | `Content/DataAsset/DA_RobotConfig` | 로봇 액터 Details → `Robot` |
| **GameMode Override** | `AEndEffectorGameMode` (또는 `BP_RobotGameMode`) | World Settings |
| `InputConfig` | `Content/DataAsset/DA_InputConfig` | PlayerController(`BP_RobotController`) Details |
| Target Actor | `SpawnOrAlignTargetToCurrentEndEffector` 실행으로 생성 | 로봇 액터 Details |
| PlayerStart | 로봇이 보이는 위치/방향 | 맵에 배치 (카메라가 여기 고정됨) |

- **GameMode Override** 를 지정하면 `AEndEffectorPlayerController` 와 `AEndEffectorViewPawn` 이 자동 배선된다.
- Enhanced Input 에셋(`IMC_RobotTargetControl`, `IA_MoveXY` / `IA_MoveZ` / `IA_Rotate` / `IA_SolveIK` /
  `IA_ResetTarget` / `IA_SelectTarget` / `IA_DragTarget`)은 `Content/Inputs/` 에 이미 authoring 돼 있다.
  키 매핑을 직접 재구성해야 한다면 [STEP_A-06 §Enhanced Input 에셋 생성 방법](Docs/Steps/STEP_A-06.md) 참고.
- InputConfig나 개별 Action이 비어 있어도 컴파일/실행은 된다. 컨트롤러가 Warning을 남기고 **해당 조작만** 비활성화한다.

### 1.4 PIE 실행 절차

1. 로봇 Details → **`SpawnOrAlignTargetToCurrentEndEffector`** 실행 → 현재 EE 위치에 Target Actor 생성/정렬.
2. **PIE 실행**.
3. Target을 마우스로 클릭·드래그하거나, 키보드로 이동/회전한다 (조작표는 [§3.2](#32-키보드--마우스-조작표)).
4. **Space / Enter** 로 IK solve.
5. Output Log의 `LogRobotSim` 카테고리에서 수렴 여부 · 반복 횟수 · 최종 위치/회전 오차를 확인한다.
   뷰포트의 EE↔Target 디버그 링크로도 시각적으로 확인 가능하다.

---

## 2. 구현 요약

### 2.1 레이어 분리 (핵심 설계 원칙)

| 레이어 | 책임 | 대표 타입 | UObject 의존 |
|---|---|---|---|
| **수학** | FK / pose error / Jacobian / IK 계산 | `FSerial6DoFModel`, `FRobotPoseError`, `FRobotJacobian`, `FRobotDlsIK` | **없음** (액터 스폰 없이 단위테스트) |
| **액터** | solve 오케스트레이션 + 비주얼 | `ASerial6DoFRobotActor` | 있음 |
| **interaction** | 키보드/마우스 입력, target 조작 | `AEndEffectorPlayerController`, `AEndEffectorTargetActor` | 있음 |

수학 레이어는 엔진 타입(`FTransform`/`FQuat`) 외에 UObject/World에 의존하지 않는다.
컨트롤러는 solver를 **직접 호출하지 않고** `Robot->SolveIKToTarget()` 에 위임한다.

### 2.2 단계별 수학

단위 규약: **위치 cm, 각도 radian**. Unreal 좌표계(LHS, Z-up), 합성 순서 `World = Local * Parent`.

| 항목 | 내용 | 파일 | 문서 |
|---|---|---|---|
| **FK source of truth** | `FSerial6DoFModel` — 6R, `EE = ToolOffset * J5 * … * J0 * Base`. Q=0에서 EE는 액터 공간 (105, 0, 120)cm | `Public/Robot/Serial6DoFModel.h` | [A-01](Docs/Steps/STEP_A-01.md) |
| **Pose Error** | `e = target − current`. 위치는 cm 차, 회전은 **quaternion-log(axis-angle) 벡터** (rad) — Euler 차가 아님 | `Public/Robot/RobotPoseError.h` | [A-02](Docs/Steps/STEP_A-02.md) |
| **Numerical Jacobian** | 6×6 전진 유한차분, `ε = 1e-4 rad`. 행 0~2 위치(cm/rad), 행 3~5 회전(rad/rad), 열 = J0~J5 | `Public/Robot/RobotJacobian.h` | [A-03](Docs/Steps/STEP_A-03.md) |
| **DLS IK** | `dq = Jᵀ (J Jᵀ + λ² I)⁻¹ e`, `q += clamp(dq, ±MaxStepRad)` | `Public/Robot/RobotDlsIK.h` | [A-04](Docs/Steps/STEP_A-04.md) |
| **Nullspace** | `J# = Jᵀ (J Jᵀ + λ² I)⁻¹`, `N = I − J#J`, `dq = dq_task + N·dq_null` | `RobotDlsIK.h` (`ComputeJointLimitAvoidanceVelocity`) | [A-05](Docs/Steps/STEP_A-05.md) |
| **ToolOffset 보정** | `TargetModel = TargetWorld.GetRelativeTransform(RobotActor)` → `ToolOffset = TargetModel.GetRelativeTransform(J5Model)` | `Private/Robot/Serial6DoFRobotActor.cpp` | [A-06.1](Docs/Steps/STEP_A-06.1.md) |

**DLS 반복식 상세** — 가중 오차와 row-scaled Jacobian을 쓴다:

```
e   = [Wp · Δpos(cm), Wr · Δrot(rad)]        // 가중 오차 6-벡터
J   = row-scaled Jacobian (행0~2 ×Wp, 행3~5 ×Wr)
dq  = Jᵀ (J Jᵀ + λ² I)⁻¹ e
q  += clamp(dq, ±MaxStepRad)
```

**기본 옵션값** (`FRobotDlsIKOptions`):

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `MaxIterations` | 80 | 도달 불가/local minimum에서 무한 루프 방지 |
| `PositionToleranceCm` | 0.5 | 위치 수렴 허용 오차 |
| `RotationToleranceRad` | 2° | 회전 수렴 허용 오차 |
| `DampingLambda` (λ) | 0.15 | singularity 근처 inverse 폭주 억제 |
| `MaxStepRad` | 8° | 선형 근사가 유효한 범위로 스텝 제한 |
| `PositionWeight` (Wp) | 1.0 | 위치 가중치 |
| `RotationWeight` (Wr) | 30.0 | cm ↔ rad 스케일 차 보정 |
| `bUseNullspaceJointLimitAvoidance` | `false` | 켜면 A-05 동작, 끄면 A-04와 완전히 동일 |
| `NullspaceGain` | 0.05 | 보조 항 크기 |
| `JointLimitActivationRatio` | 0.65 | 이 normalized 거리 이상에서만 회피 활성화 |

---

## 3. 사용법

### 3.1 Target Actor 생성/정렬

| 함수 (로봇 Details, CallInEditor) | 동작 |
|---|---|
| `SpawnOrAlignTargetToCurrentEndEffector` | Target Actor가 없으면 생성, 있으면 현재 EE 위치/자세로 정렬 |
| `CopyCurrentEndEffectorToTarget` | 기존 Target을 현재 EE로 덮어쓰기 |
| `SolveIKToTarget` | Target을 향해 DLS IK 1회 solve |
| `ResetJointAngles` | 모든 관절 0(홈 자세)으로 복귀 |

Target Actor의 `GetActorTransform()` 이 **곧 IK target** 이다. 로봇 수학에 어떤 영향도 주지 않는다.

> **프레임 규약**: solver는 **모델 공간**(로봇 액터 로컬) target을 받는다. `SolveIKToTarget()` 이 내부에서
> `TargetWorld.GetRelativeTransform(RobotActorTransform)` 으로 월드→모델 변환을 수행하므로, 로봇이 원점이
> 아닌 곳에 이동/회전돼 있어도 정확히 동작한다. 자세한 내용은 [STEP_A-06 §프레임 규약](Docs/Steps/STEP_A-06.md).

### 3.2 키보드 / 마우스 조작표

| 키 | 동작 | InputAction / 축 |
|----|------|------------------|
| **W / S** | target +X / −X | `MoveXY` (Axis2D, X) |
| **A / D** | target −Y / +Y | `MoveXY` (Axis2D, Y) |
| **Q / E** | target +Z / −Z | `MoveZ` (Axis1D) |
| **I / K** | pitch + / − | `Rotate` (Axis3D, X) |
| **J / L** | yaw + / − | `Rotate` (Axis3D, Y) |
| **U / O** | roll + / − | `Rotate` (Axis3D, Z) |
| **Space / Enter** | `SolveIKToTarget` 실행 | `SolveIK` (Bool) |
| **R** | target을 현재 EE로 리셋 | `ResetTarget` (Bool) |
| **LMB** | target 선택 + 드래그 | `SelectTarget` / `DragTarget` (Bool) |

- 이동/회전은 **로봇 액터(베이스) 프레임** 기준 — 로봇이 회전돼 있어도 "target +X"가 로봇 X를 향한다.
- 키를 누르는 동안 매 프레임 `속도 × Δt` 만큼 연속 이동/회전한다.
  속도는 `URobotTargetInputConfig` 의 `MoveSpeedCmPerSec`(기본 100) / `RotateSpeedDegPerSec`(기본 60)으로 조절.
- **드래그**: LMB로 target을 맞히면 선택(색 변경), 누른 채 움직이면 커서 ray를 **XY 수평면**(드래그 시작 시점의
  target Z 높이)과 교차시켜 X/Y를 갱신한다. **Z와 회전은 유지** (Z는 Q/E, 회전은 키보드 담당).
- **solve 타이밍**: `bAutoSolveOnTargetMove=true` → 드래그 중 매번 solve. `false`(기본) → 드래그를 놓는 순간
  `bSolveOnDragRelease` 에 따라 1회 solve, 또는 Space로 수동 solve.
- nullspace on/off는 별도 키가 없다. 로봇 Details의 `bUseNullspaceJointLimitAvoidance` 로 토글한다.

### 3.3 ToolOffset 캘리브레이션

수학 모델의 EE 기준점(ToolTip)을 **실제 KUKA 메시의 그리퍼 끝**에 맞추는 절차다.
로봇 체인/기구학(`LinkOffsets`/`JointAxes`/`JointLimits`)은 **건드리지 않고 ToolOffset만** 보정한다.

1. (권장) **홈 자세**로 둔다 — `ResetJointAngles` 실행. ToolOffset은 현재 자세 기준으로 역산되므로.
2. `SpawnOrAlignTargetToCurrentEndEffector` 로 Target Actor 생성.
3. Target Actor를 에디터 기즈모로 **실제 그리퍼 끝 위치·방향**에 맞춘다.
   위치뿐 아니라 **회전도** 실제 툴 방향에 맞춰야 한다 (ToolOffset이 위치+회전을 모두 담음).
4. (선택) **`LogToolOffsetCalibrationState`** 로 적용 없이 계산 결과를 미리 확인.
5. 로봇 Details → **`CalibrateToolOffsetFromTarget`** 실행.
6. `RobotConfig` 가 지정돼 있으면 그 **에셋의 `ToolOffset` 에 기록**된다 → **Ctrl+S로 에셋 저장 필수**.
   지정돼 있지 않으면 모델에 일시 적용되고 액터 재구성 시 초기화되므로, 로그에 찍힌 값을 RobotConfig에 수동 입력한다.

적용 후 로그(`적용후 EE↔Target`)에 잔차가 남을 수 있다 — 이는 버그가 아니라 메시/수학 비율 차이의 노출이다
([§6 한계](#6-현재-한계) 참고). 자세한 내용은 [STEP_A-06.1](Docs/Steps/STEP_A-06.1.md).

---

## 4. 테스트

수학 레이어는 UObject/World에 의존하지 않으므로 **액터 스폰 없이** 단위테스트한다.
총 **26개** Automation Test, 5개 그룹. 모두 `IMPLEMENT_SIMPLE_AUTOMATION_TEST` + `ProductFilter`.

### 4.1 실행 방법

- **Session Frontend**: 에디터 → `Tools` → `Session Frontend` → `Automation` 탭 → 필터에 `RobotSim` 입력 → `Start Tests`
- **콘솔 명령**: `Automation RunTests RobotSim`

> 테스트는 `FSerial6DoFModel::CreateDefault()` 를 사용한다. 따라서 `DA_RobotConfig` 나 ToolOffset 캘리브레이션
> 결과와 **무관하게** 항상 동일하게 통과한다 (골든 테스트).

### 4.2 테스트 그룹

**`RobotSim.Kinematics.*`** (5) — FK 정확성과 관절 한계 · `Private/Tests/RobotKinematicsTests.cpp`

| 테스트 | 검증 내용 |
|---|---|
| `ZeroPoseFK` | Q=0에서 EE가 링크 기하 합인 (105, 0, 120)cm에 위치 |
| `J0YawRotatesAboutZ` | J0 yaw가 EE를 +Z 축 둘레로 회전 (높이·반경 보존) |
| `J1PitchMovesEE` | J1 pitch가 어깨 피벗 (0,0,60) 기준 +Y 축 회전 |
| `ToolRollInvariance` | J5 roll 축이 ToolOffset (10,0,0)의 X축과 공선 → EE 위치 불변 |
| `JointLimitClamp` | 범위 밖 각도가 Min/Max로 정확히 clamp, `IsWithinLimits` 와 일치 |

**`RobotSim.PoseError.*`** (6) — 6D pose error · `Private/Tests/RobotPoseErrorTests.cpp`

| 테스트 | 검증 내용 |
|---|---|
| `Identity` | 동일 transform → 위치·회전 오차 모두 0 |
| `PositionOnly` | 10cm 평행이동 → 위치 오차만, 회전 오차 0 |
| `Rotation90` | 90° target → 회전 오차 크기 ≈ π/2, 축 방향 정확 |
| `ShortestPath` | −90° / 270° 표현이 **최단 경로** 회전 오차로 수렴 |
| `NearPiStable` | 180° 근처에서 NaN/Inf 없이 크기 ≈ π (quaternion-log 수치 안정성) |
| `Weighted` | 가중 오차 = 원 오차 × 가중치 (성분별) |

**`RobotSim.Jacobian.*`** (5) — numerical Jacobian · `Private/Tests/RobotJacobianTests.cpp`

| 테스트 | 검증 내용 |
|---|---|
| `ShapeAndFinite` | 6×6 = 36개 성분 전부 유한값 |
| `J0YawPositionDerivative` | Q=0에서 J0 위치 미분 ≈ (0, 105, 0) — **해석적 `ẑ × r` 과 일치** |
| `J5RollPositionInvariant` | Q=0에서 J5 roll 위치 미분 ≈ 0 (축이 ToolOffset과 공선) |
| `J5RollRotationDerivative` | Q=0에서 J5 roll 회전 미분이 +X (행3 ≈ 1, 행4/5 ≈ 0) |
| `NoMutation` | `ComputeNumericalJacobian` 이 입력 `FRobot6DJointState` 를 변경하지 않음 |

**`RobotSim.IK.*`** (5) — DLS IK solver · `Private/Tests/RobotDlsIKTests.cpp`

| 테스트 | 검증 내용 |
|---|---|
| `AlreadyAtTarget` | Q=0의 FK 결과를 target으로 주면 0~1 반복 내 수렴 |
| `SmallReachablePositionOffset` | +10cm Y 오프셋 target → 위치 오차 유의미하게 감소 |
| `SmallReachableOrientationOffset` | 10° 툴 회전 target → 회전 오차 유의미하게 감소 |
| `JointLimitClamp` | 해가 관절 가동 범위를 절대 벗어나지 않음 |
| `NoNaNOnDifficultTarget` | 도달 불가/난해 target에서도 결과가 유한 (발산 없음) |

**`RobotSim.IK.Nullspace.*`** (5) — nullspace joint-limit avoidance · `Private/Tests/RobotNullspaceTests.cpp`

| 테스트 | 검증 내용 |
|---|---|
| `DisabledByDefault` | 기본 옵션에서 nullspace off → **A-04와 완전 동일 동작 보장** |
| `GradientPointsTowardMidpoint` | 한계 근처에서 회피 속도가 관절 중점 방향을 가리킴 |
| `NoActivationNearMidpoint` | 중점 근처(활성화 임계 미만)에서 회피 속도 ≈ 0 → task 방해 없음 |
| `SolverKeepsFinite` | nullspace on + 난해 target에서도 결과 유한 |
| `JointLimitDistanceDoesNotWorsenOnRedundantOrWeakTask` | 한계 근처 시작 + 약한 task에서 nullspace on이 max normalized joint distance를 악화시키지 않음 |

---

## 5. 설계 선택 / 트레이드오프

### 5.1 수학 모델이 source of truth, SkeletalMesh는 approximate visual overlay

FK/IK 결과의 정답은 `FSerial6DoFModel` 이고, KUKA SkeletalMesh는 그 결과를 **보여주기만** 하는 레이어다.
비주얼을 수정해도 수학은 불변이며, 단위테스트는 메시 없이 돌아간다.
→ [A-01.5](Docs/Steps/STEP_A-01.5.md), [A-01.5b](Docs/Steps/STEP_A-01.5b.md)

### 5.2 KUKA 메시 ≠ 6R 수학 모델 → 일부 joint retarget skip

제공된 KUKA_R3200 메시의 본 계층은 우리 6R 수학 모델과 1:1 대응하지 않는다(예: J3 미매핑).
억지로 전부 매핑하면 메시가 뒤틀리므로, **부분 매핑 + delta retarget** 으로 대응되는 본만 회전시킨다.
"정확한 수학 + 근사 비주얼"이 "왜곡된 비주얼 + 왜곡된 수학"보다 낫다는 판단이다.
→ [A-01.5a](Docs/Steps/STEP_A-01.5a.md), [A-01.5b](Docs/Steps/STEP_A-01.5b.md)

### 5.3 LinkOffsets를 메시에 맞추지 않고 ToolOffset만 보정한 이유

메시 비율에 맞춰 `LinkOffsets` 를 바꾸면 FK 결과가 전부 바뀌어 **골든 테스트 5종이 모두 무효**가 된다
(`ZeroPoseFK` 의 (105,0,120) 등이 임의의 메시 의존 값으로 변함). 반면 ToolOffset은 체인 **말단 1개 변환**이라
기구학 구조를 건드리지 않고, 테스트는 `CreateDefault()` 를 쓰므로 영향받지 않는다.
"메시에 맞추기" 대신 **"수학을 보존하고 툴 끝만 맞추기"** 를 택했다.
→ [A-06.1](Docs/Steps/STEP_A-06.1.md)

### 5.4 Numerical Jacobian을 먼저 구현한 이유

Analytic Jacobian(`ẑᵢ × (p_ee − pᵢ)`)은 유도 과정에서 축 방향·부호·프레임을 틀리기 쉽고, 틀려도 IK가
"그럭저럭 수렴"해서 오류가 드러나지 않는다. Numerical Jacobian은 FK만 맞으면 **정의상 맞으므로**,
analytic 버전을 나중에 넣을 때 **검증 기준(ground truth)** 이 된다.
실제로 `J0YawPositionDerivative` 테스트는 수치 결과가 해석적 `ẑ × r` 과 일치함을 확인한다.
→ [A-03](Docs/Steps/STEP_A-03.md)

### 5.5 Pseudo-inverse 대신 DLS를 쓴 이유

Singularity 근처에서 `J Jᵀ` 가 특이해지면 `J⁺` 의 성분이 폭주해 관절이 튄다.
DLS는 `A = J Jᵀ + λ² I` 로 damping을 더해 λ>0인 한 항상 역행렬이 안정적으로 존재하게 만든다.
정확도를 조금 희생하는 대신 **특이점에서 발산하지 않는 안정성**을 얻는다 (`NoNaNOnDifficultTarget` 이 이를 검증).
→ [A-04](Docs/Steps/STEP_A-04.md)

### 5.6 Nullspace 항이 task를 보존하면서 joint limit을 회피하는 이유

`N = I − J#J` 는 task 공간에 **직교하는 방향**으로의 투영 연산자다. 따라서 임의의 `dq_null` 에 대해
`J · (N · dq_null) ≈ 0` 이 성립한다 — 즉 nullspace 항이 아무리 관절을 움직여도 **EE pose는 (1차 근사에서)
변하지 않는다**. 6-DOF에 6D task라 엄밀한 여유 자유도는 없지만, DLS damping과 task가 약한 방향에서 유효
여유가 생기며 그 안에서 관절을 중점 쪽으로 되돌린다.
`JointLimitActivationRatio=0.65` 로 **한계 근처에서만** 발동시켜 중앙부에서는 task를 전혀 방해하지 않는다.
→ [A-05](Docs/Steps/STEP_A-05.md)

### 5.7 Full transform gizmo 대신 Target Actor + XY drag + 키보드 회전을 택한 이유

PDF 요구사항은 "click/drag + keyboard movement로 EE target 조작"이다. Full transform gizmo는 축 선택 UI,
스크린 공간 투영, 회전 링 히트테스트 등 **UI 엔지니어링 비중이 IK 수학보다 커지는** 작업이고, 요구사항을
넘어선다. Target Actor를 월드에 실체화하면 `GetActorTransform()` 이 곧 target이 되어 개념이 단순해지고,
에디터에서는 기본 기즈모로 자유롭게 배치할 수 있으며(캘리브레이션에 활용), PIE에서는 드래그(XY) + 키보드(Z·회전)로
6-DOF 전부를 커버할 수 있다.
→ [A-06](Docs/Steps/STEP_A-06.md)

---

## 6. 현재 한계

| 한계 | 상세 |
|---|---|
| **Numerical Jacobian은 느리다** | 매 반복 6열 × FK 재평가 → analytic Jacobian 대비 연산량이 크다. 단일 target solve에는 충분하나 대량/실시간 다중 로봇에는 analytic 전환이 필요. |
| **Local minimum / 도달 불가 target** | DLS는 지역 반복법이라 전역 최적해를 보장하지 않는다. 도달 불가 target에서는 잔차가 남고 `bConverged=false` 로 로그에 노출된다(발산은 하지 않음). |
| **SkeletalMesh ↔ 수학 모델 1:1 아님** | KUKA 메시 본 구조가 6R 모델과 달라 일부 joint는 retarget에서 skip된다. 비주얼은 approximate overlay다. |
| **ToolOffset 보정 ≠ 전체 치수 보정** | 보정되는 것은 **툴 끝 기준점 하나**다. 캘리브레이션한 자세에서는 정확히 맞지만, 다른 자세에서는 J5 상류 링크 길이 차이만큼 잔차가 남는다(홈 자세 캘리브레이션을 권장하는 이유). 완전 일치는 기구학 자체를 메시에 맞춰야 하고, 그러면 골든 테스트가 무효가 된다. |
| **드래그 평면은 XY 고정** | 카메라 대면 평면 드래그 / 자유 3D 드래그 미지원. full transform gizmo 없음. |
| **관전 카메라 고정** | `AEndEffectorViewPawn` 은 이동 바인딩을 끈 전용 폰이다. 기본 `ADefaultPawn` 이 W/A/S/D·Q/E·마우스를 자유비행에 소비해 target 조작 키와 충돌하기 때문. 자유 카메라가 필요하면 별도 pawn/입력 추가 필요. |
| **pick/place FSM 없음** | Step A 범위 밖. target 추종까지만 구현. |

---

## 7. PDF Step A 요구사항 대응표

| PDF 요구사항 | 충족 | 근거 |
|---|:---:|---|
| 6-DOF 로봇팔 | ✅ | `FSerial6DoFModel` — 6R (J0 yaw / J1·J2 pitch / J3 roll / J4 pitch / J5 roll) |
| End Effector **위치** IK | ✅ | pose error 위치 성분(cm) + DLS 반복 → `SmallReachablePositionOffset` |
| End Effector **자세** IK | ✅ | quaternion-log 회전 오차(rad) + DLS 반복 → `SmallReachableOrientationOffset` |
| **클릭** 으로 target 조작 | ✅ | LMB + `GetHitResultUnderCursor` 로 Target Actor 선택 |
| **드래그** 로 target 조작 | ✅ | LMB 드래그 → XY 수평면 교차로 target 이동 |
| **키보드** 로 target 이동 | ✅ | W/S/A/D/Q/E 이동 + I/K/J/L/U/O 회전 |
| Unreal 내장 IK 플러그인 **미사용** | ✅ | `Build.cs` 에 IK 관련 플러그인/모듈 없음. FK·Jacobian·IK 전부 자체 C++ 구현 |
| **Jacobian 기반** IK | ✅ | `FRobotJacobian::ComputeNumericalJacobian` (6×6 유한차분) |
| **DLS** 특이점 회피 | ✅ | `dq = Jᵀ(JJᵀ + λ²I)⁻¹ e`, λ=0.15 |
| **Nullspace** 관절 한계 회피 | ✅ | `N = I − J#J`, `bUseNullspaceJointLimitAvoidance` + 전용 테스트 5종 |
| **단위 테스트** | ✅ | Automation Test **26개** / 5개 그룹 |
| **수학·물리 정확성 설명** | ✅ | `Docs/Steps/` 단계별 문서 11편 + 본 README |

---

## 8. 문서 인덱스

| 문서 | 내용 |
|---|---|
| [STEP_A-01](Docs/Steps/STEP_A-01.md) | 6축 FK 수학 모델, 관절 계층/한계, 디버그 비주얼 |
| [STEP_A-01.5](Docs/Steps/STEP_A-01.5.md) | 시각화 레이어 — SkeletalMesh 본 동기화 + 링크별 StaticMesh |
| [STEP_A-01.5a](Docs/Steps/STEP_A-01.5a.md) | Bone Mapping Probe — KUKA 본 매핑 조사 도구 및 결과 |
| [STEP_A-01.5b](Docs/Steps/STEP_A-01.5b.md) | 부분 본 매핑 — SkeletalMesh를 approximate overlay로 확정 |
| [STEP_A-02](Docs/Steps/STEP_A-02.md) | 6D Pose Error — 위치 cm + quaternion-log 회전 rad, 단위/부호 규약 |
| [STEP_A-03](Docs/Steps/STEP_A-03.md) | Numerical Jacobian — 전진차분, epsilon 선택 근거, 행·열 규약 |
| [STEP_A-04](Docs/Steps/STEP_A-04.md) | Damped Least Squares IK — 수식, 가중치, 스텝 제한, 수렴 판정 |
| [STEP_A-04.1](Docs/Steps/STEP_A-04.1.md) | `URobotConfig` DataAsset 통합, BoneProbe 제거 (housekeeping) |
| [STEP_A-05](Docs/Steps/STEP_A-05.md) | Nullspace Joint-Limit Avoidance — projector 수식, gradient, 활성화 정책 |
| [STEP_A-06](Docs/Steps/STEP_A-06.md) | End Effector Target Interaction — 프레임 규약, 조작표, Enhanced Input 에셋 |
| [STEP_A-06.1](Docs/Steps/STEP_A-06.1.md) | ToolOffset Calibration 검증 — 계산식, 절차, 잔차 해석 |

---

## 9. 소스 구조

```
Source/RobotSim_Assignment/
├─ Public/Robot/ · Private/Robot/
│   ├─ RobotTypes.h                 // FRobotJointLimit, FRobot6DPose, FRobot6DJointState, FRobot6DPoseError
│   ├─ Serial6DoFModel.h/.cpp       // [수학] FK source of truth
│   ├─ RobotPoseError.h/.cpp        // [수학] 6D pose error (quaternion-log)
│   ├─ RobotJacobian.h/.cpp         // [수학] 6×6 numerical Jacobian
│   ├─ RobotDlsIK.h/.cpp            // [수학] DLS IK + nullspace
│   ├─ RobotConfig.h/.cpp           // [에셋] 로봇 정의 DataAsset (deg→rad 변환 지점)
│   ├─ RobotTargetInputConfig.h/.cpp// [에셋] Enhanced Input 배선/튜닝 DataAsset
│   ├─ Serial6DoFRobotActor.h/.cpp  // [액터] solve 오케스트레이션 + 비주얼 + CallInEditor 디버그
│   ├─ EndEffectorTargetActor.h/.cpp    // [액터] 월드 target
│   ├─ EndEffectorPlayerController.h/.cpp // [입력] 키보드/마우스
│   ├─ EndEffectorGameMode.h/.cpp   // [입력] PC/Pawn 배선
│   ├─ EndEffectorViewPawn.h/.cpp   // [입력] 이동 바인딩 제거한 고정 뷰 폰
│   └─ RobotSimLog.h/.cpp           // LogRobotSim 카테고리
└─ Private/Tests/                   // 26 automation tests (5 files)
```

수학 레이어(`Serial6DoFModel` / `RobotPoseError` / `RobotJacobian` / `RobotDlsIK`)는 UObject에 의존하지 않으며,
`static` 함수만 가진 plain struct 패턴으로 통일돼 있다.
