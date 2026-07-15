# Step A-06: End Effector Target Interaction

## 목표

PDF Step A의 **"End Effector click/drag + keyboard movement"** 요구사항을 구현한다.
사용자가 월드의 target을 마우스/키보드로 조작하면 로봇 팔이 DLS IK로 그 target을 따라간다.

STEP A-01~A-05의 순수 수학 레이어(FK / pose error / numerical Jacobian / DLS IK / nullspace)와
비주얼 액터는 그대로 유지하고, 그 위에 **interaction 레이어**만 얹는다.

이 단계에서는 다음을 **하지 않는다**:
- FK/Jacobian/DLS/nullspace 수학식, `FSerial6DoFModel` axis/offset/limit, 기존 테스트, SkeletalMesh 리타겟 수정
- pick/place FSM 구현
- full transform gizmo / 대규모 UI 시스템 구현
- Enhanced Input의 IA/IMC를 코드에서 생성 (에셋으로 authoring)

## 왜 Target Actor / interaction 레이어를 분리했는가

이 프로젝트의 핵심 원칙은 **수학 solver ↔ UI/interaction 분리**다.

| 레이어 | 책임 | 대표 타입 |
|---|---|---|
| 수학 | FK/IK 계산 (UObject 비의존, 단위테스트) | `FSerial6DoFModel`, `FRobotDlsIK` |
| 액터 | solve 오케스트레이션 + 비주얼 | `ASerial6DoFRobotActor` |
| interaction | 키보드/마우스 입력, target 조작 | `AEndEffectorPlayerController`, `AEndEffectorTargetActor` |

`AEndEffectorTargetActor`는 월드에 보이는 **목표 Transform 그 자체**다. 액터의 `GetActorTransform()`이 곧
IK target이며, 로봇 수학에는 어떤 영향도 주지 않는다. 컨트롤러는 입력만 담당하고 수학은 직접 호출하지
않으며, 로봇에 명령을 위임한다(`Robot->SolveIKToTarget()`). 이렇게 하면 수학 레이어는 A-05까지의 상태를
그대로 유지하면서 조작 UI만 독립적으로 붙였다 뗐다 할 수 있다.

## 프레임 규약 (월드 ↔ 모델 공간) — 핵심

`FRobotDlsIK::SolveDlsIK`는 **로봇 모델 공간**(액터 로컬, `BaseTransform=Identity` 기준) target을 받는다.
그런데 `AEndEffectorTargetActor`는 월드에 배치되므로 `GetActorTransform()`이 **월드** 트랜스폼이다.
따라서 `SolveIKToTarget()`은 target을 solver에 넘기기 전에 로봇 액터 기준 상대 트랜스폼으로 내린다:

```cpp
const FTransform TargetWorld = EndEffectorTargetActor->GetActorTransform();
const FTransform TargetModel = TargetWorld.GetRelativeTransform(GetActorTransform());
FRobotDlsIK::SolveDlsIK(Model, CurrentState, TargetModel, Options);
```

- **월드 트랜스폼을 그대로 넘기면 안 된다.** 로봇 액터가 원점/Identity일 때만 우연히 맞고,
  로봇이 이동/회전된 맵에서는 target이 틀어진다.
- 기존 `TargetEndEffectorWorld`(Target Actor 미지정 시 폴백)는 이미 모델 공간 값이므로 그대로 사용한다
  (기존 동작 100% 보존).
- 역방향(현재 EE로 target 정렬)은 모델 공간 → 월드로 올린다:
  `Model.ComputeEndEffectorTransform(CurrentState) * GetActorTransform()`.

## 키보드 조작표

| 키 | 동작 | InputAction / 축 |
|----|------|------------------|
| W / S | target +X / −X | `MoveXY` (Axis2D, X) |
| A / D | target −Y / +Y | `MoveXY` (Axis2D, Y) |
| Q / E | target +Z / −Z | `MoveZ` (Axis1D) |
| I / K | pitch + / − | `Rotate` (Axis3D, X) |
| J / L | yaw + / − | `Rotate` (Axis3D, Y) |
| U / O | roll + / − | `Rotate` (Axis3D, Z) |
| Space / Enter | `SolveIKToTarget` 실행 | `SolveIK` (Bool) |
| R | target을 현재 EE로 리셋 | `ResetTarget` (Bool) |
| 왼쪽 마우스 | target 선택 + 드래그 | `SelectTarget` / `DragTarget` (Bool) |

- 이동은 **로봇 액터(베이스) 프레임** 기준이라 로봇이 회전돼 있어도 "target +X"가 로봇 X를 향한다.
- 키를 누르는 동안 매 프레임 `속도 × Δt`만큼 연속 이동/회전한다(cm/초, 도/초). 속도는 `URobotTargetInputConfig`
  의 `MoveSpeedCmPerSec`(기본 100) / `RotateSpeedDegPerSec`(기본 60)로 조절한다.
- 회전은 내부적으로 `FQuat`로 합성한다. Euler(pitch/yaw/roll)는 입력 경계의 step 값에서만 쓴다.

## 마우스 클릭/드래그 동작

1. **선택**: 왼쪽 마우스 누름 → `GetHitResultUnderCursor(ECC_Visibility)`로 Target Actor를 맞히면 선택
   (색이 `SelectedColor`로 바뀜). Target Actor의 마커 메시만 QueryOnly 컬리전을 가져 커서에 잡힌다
   (로봇 링크는 NoCollision).
2. **드래그**: 누른 상태로 마우스를 움직이면 커서 world ray를 **XY 수평면**(드래그 시작 시점의 target Z 높이)과
   교차시켜 target의 X/Y를 갱신한다. **Z와 회전은 유지**(Z 이동은 Q/E, 회전은 키보드가 담당).
3. **solve 타이밍**: `bAutoSolveOnTargetMove=true`면 드래그 중 매번 solve, `false`(기본)면 드래그를 놓는 순간
   `bSolveOnDragRelease`에 따라 1회 solve(또는 Space로 수동 solve).

> 회전 드래그/full transform gizmo는 구현하지 않는다. 드래그는 XY 위치 이동 중심이고, 회전은 키보드로 처리한다.

## SolveIKToTarget 흐름

```
[입력]  키보드 Space/Enter · 마우스 드래그 릴리스 · auto-solve
   ↓  (AEndEffectorPlayerController → 위임)
Robot->SolveIKToTarget()
   ↓  target 소스 선택
        EndEffectorTargetActor 있음 → TargetWorld.GetRelativeTransform(RobotActorTransform)  (월드→모델)
        없음                        → TargetEndEffectorWorld  (기존 모델 공간 값)
   ↓
FRobotDlsIK::SolveDlsIK(Model, CurrentState, TargetModel, Options)   // 순수 수학 (A-04/A-05 그대로)
   ↓
SetJointAngles(Result.Solution)  → 비주얼 반영 + JointAnglesDeg 동기화
   ↓
UE_LOG: 수렴/미수렴 · 반복수 · FinalPositionErrorCm · FinalRotationErrorRad · nullspace
```

## 수학 solver와 UI 레이어 분리

- 컨트롤러는 `SolveDlsIK`를 **직접 호출하지 않는다**. `Robot->SolveIKToTarget()` /
  `Robot->CopyCurrentEndEffectorToTarget()`만 호출한다.
- 조작 대상 target은 로봇이 소유(`Robot->GetEndEffectorTargetActor()`)한다 → 컨트롤러/로봇이 단일 소스를 공유.
- Enhanced Input 배선(IMC/IA)과 튜닝값은 `URobotTargetInputConfig` DataAsset 하나에 모은다 → 컨트롤러에
  입력 설정을 개별 UPROPERTY로 흩뿌리지 않는다.

## ToolOffset 캘리브레이션 (EE 기준점을 실제 그리퍼 끝에 맞추기)

디버그 매시(수학 정답)와 KUKA 메시는 비율이 달라 완벽히 겹치지 않는데, 특히 **EE 기준점(수학 ToolTip)이
실제 메시 그리퍼 끝과 어긋나** 보일 수 있다. 이를 **ToolOffset만** 보정해 맞춘다(로봇 체인/기구학은 불변).

- **수식**: FK 합성이 `EE_model = ToolOffset * J5_model` 이므로,
  `ToolOffset = TipModel.GetRelativeTransform(J5_model)` 로 역산한다.
  여기서 `TipModel = TargetWorld.GetRelativeTransform(RobotActorTransform)`(월드→모델),
  `J5_model = ComputeJointWorldTransform(5, CurrentState)`.
- **절차**:
  1. (권장) 로봇을 **홈 자세**(모든 관절 0)로 둔다 — ToolOffset은 현재 자세 기준으로 계산되므로.
  2. `SpawnOrAlignTargetToCurrentEndEffector`로 Target Actor 생성.
  3. Target Actor를 에디터에서 **실제 그리퍼 끝 위치·방향**으로 옮긴다(이동 기즈모/드래그). 위치뿐 아니라
     **회전도 실제 툴 방향에 맞춘다** — ToolOffset이 위치+회전을 모두 담는다.
  4. 로봇 Details → **`CalibrateToolOffsetFromTarget`** 실행.
  5. RobotConfig가 있으면 그 에셋의 `ToolOffset`에 기록됨 → **에셋 저장(Ctrl+S)**. 없으면 모델에 일시 적용(재구성
     시 초기화되므로 로그에 찍힌 값을 RobotConfig에 수동 입력).
- **한계**: 메시 비율 ≠ 수학 비율이라 ToolOffset 하나로는 **캘리브레이션한 자세에서만 정확히** 일치하고, 다른
  자세에서는 J5 상류 링크 길이 차이만큼 잔차가 남는다(홈 자세 캘리브레이션 권장 이유). 완전 일치는 기구학 자체를
  메시에 맞춰야 하는데, 그건 FK/IK 골든 테스트를 바꾸므로 이번 범위 밖이다.
- **수학 불변**: `FSerial6DoFModel`의 LinkOffsets/JointAxes/JointLimits와 `CreateDefault()`는 건드리지 않는다.
  테스트는 `CreateDefault()`를 쓰므로 ToolOffset 보정(RobotConfig/actor Model)과 무관하게 그대로 통과한다.

## 변경/추가 파일

- 신규 `Public/Robot/EndEffectorTargetActor.h` / `Private/Robot/EndEffectorTargetActor.cpp` — 월드 target 액터(마커 + 디버그 축/색, QueryOnly 컬리전)
- 신규 `Public/Robot/RobotTargetInputConfig.h` / `Private/Robot/RobotTargetInputConfig.cpp` — Enhanced Input 배선/튜닝 DataAsset
- 신규 `Public/Robot/EndEffectorPlayerController.h` / `Private/Robot/EndEffectorPlayerController.cpp` — 키보드/마우스 입력 소유 컨트롤러
- 신규 `Public/Robot/EndEffectorGameMode.h` / `Private/Robot/EndEffectorGameMode.cpp` — PlayerController 배선용 GameMode
- 신규 `Public/Robot/EndEffectorViewPawn.h` / `Private/Robot/EndEffectorViewPawn.cpp` — 이동 바인딩을 끈 고정 뷰 폰(WASD 충돌 방지)
- 수정 `Public/Robot/Serial6DoFRobotActor.h` — `EndEffectorTargetActor`/`bDrawTargetLink` UPROPERTY, getter, 2개 CallInEditor 함수 선언
- 수정 `Private/Robot/Serial6DoFRobotActor.cpp` — `SolveIKToTarget` target 소스 분기(월드→모델 변환), `SpawnOrAlignTargetToCurrentEndEffector`/`CopyCurrentEndEffectorToTarget`/`CalibrateToolOffsetFromTarget`, target 디버그 링크, PIE `BeginPlay` 초기화(SkeletalMesh 포즈 버퍼 재할당)
- 수정 `RobotSim_Assignment.Build.cs` — `EnhancedInput` 모듈 추가
- 신규 `Docs/Steps/STEP_A-06.md` (본 문서)

수학 파일(`FSerial6DoFModel`/`FRobotJacobian`/`FRobotDlsIK`/`FRobotPoseError`)과 기존 테스트,
SkeletalMesh 리타겟 코드는 **수정하지 않았다**.

## Enhanced Input 에셋 생성 방법 (에디터)

IA/IMC는 코드에서 만들지 않으므로 에디터에서 다음을 한 번 생성한다.

1. **InputAction 에셋 생성** (Content Browser → Input → Input Action)

   | 에셋 이름 | Value Type | 키 매핑 시 축 |
   |---|---|---|
   | `IA_MoveXY` | Axis2D (Vector2D) | X=W/S, Y=A/D |
   | `IA_MoveZ` | Axis1D (float) | Q/E |
   | `IA_Rotate` | Axis3D (Vector) | X=I/K, Y=J/L, Z=U/O |
   | `IA_SolveIK` | Digital (bool) | Space/Enter |
   | `IA_ResetTarget` | Digital (bool) | R |
   | `IA_SelectTarget` | Digital (bool) | LMB |
   | `IA_DragTarget` | Digital (bool) | LMB |

2. **InputMappingContext 생성** (`IMC_RobotTargetControl`) 후 각 키를 매핑하며 modifier를 붙인다:

   - `IA_MoveXY`: `W`(+X, 무수정) / `S`(Negate) / `D`(Swizzle Input Axis Values **YXZ**) / `A`(Swizzle **YXZ** + Negate)
   - `IA_MoveZ`: `Q`(무수정) / `E`(Negate)
   - `IA_Rotate`: `I`(무수정=X) / `K`(Negate) / `J`(Swizzle **YXZ**) / `L`(Swizzle **YXZ**+Negate) / `U`(Swizzle **ZYX**) / `O`(Swizzle **ZYX**+Negate)
   - `IA_SolveIK`: `Space`, `Enter`
   - `IA_ResetTarget`: `R`
   - `IA_SelectTarget`: `Left Mouse Button`
   - `IA_DragTarget`: `Left Mouse Button`

3. **`URobotTargetInputConfig` DataAsset 생성** (Miscellaneous → Data Asset → RobotTargetInputConfig).
4. 위 IMC/IA 레퍼런스를 DataAsset의 대응 필드에 할당하고, 필요하면 속도 튜닝값을 조정한다.
5. `AEndEffectorPlayerController`(또는 그 Blueprint)의 `InputConfig`에 이 DataAsset을 할당한다.

> 에셋을 아직 안 만들었어도 컴파일/실행은 된다. config나 개별 Action이 비어 있으면 컨트롤러가 Warning을
> 남기고 해당 조작만 비활성화한다.

## 실행 방법

1. 맵에 `ASerial6DoFRobotActor`를 배치한다.
2. 맵에 **PlayerStart**를 두고(카메라 시점), World Settings → **GameMode Override** = `AEndEffectorGameMode`.
   (또는 이 GameMode를 상속한 Blueprint를 만들어 `InputConfig`/`Robot`을 지정.)
3. 로봇 Details에서 **SpawnOrAlignTargetToCurrentEndEffector**를 실행 → 현재 EE 위치에 Target Actor 생성/정렬.
4. **PIE 실행**.
5. target을 클릭·드래그하거나 W/S/A/D/Q/E(이동)·I/K/J/L/U/O(회전)로 움직인다.
6. **Space/Enter**로 IK solve(또는 auto-solve). Output Log(`LogRobotSim`)의 최종 오차와 EE↔target 디버그
   링크로 결과를 확인한다.

## 수동 검증 체크리스트

Interaction은 자동화 테스트 대신 수동으로 확인한다.

- [ ] `SpawnOrAlignTargetToCurrentEndEffector`로 target actor 생성/정렬 (직후 solve 시 오차 ≈ 0)
- [ ] W/S/A/D/Q/E로 target 위치 이동 확인
- [ ] I/K/J/L/U/O로 target 회전(좌표축 방향 변화) 확인
- [ ] Space/Enter로 IK solve, 로그에 최종 위치/회전 오차 출력
- [ ] target 클릭 시 색 변경(선택) 확인
- [ ] 드래그로 XY 평면 위치 이동 확인 (Z·회전 유지)
- [ ] target 변경 후 solve → final position/rotation error 감소 확인
- [ ] R로 target이 현재 EE로 리셋되는지 확인

## 기존 수학 테스트 회귀

수학 파일을 수정하지 않았으므로 아래 그룹은 전부 그대로 통과해야 한다(회귀 없음).
Session Frontend 또는 `Automation RunTests`로 실행한다.

`RobotSim.Kinematics.*`, `RobotSim.PoseError.*`, `RobotSim.Jacobian.*`, `RobotSim.IK.*`, `RobotSim.IK.Nullspace.*`

## PDF Step A 요구사항 충족

| PDF 요구 | 충족 방식 |
|---|---|
| End Effector **click** | LMB로 Target Actor 선택(`GetHitResultUnderCursor`) |
| End Effector **drag** | LMB 드래그로 XY 수평면 위 target 이동 |
| **keyboard movement** | W/S/A/D/Q/E 이동 + I/K/J/L/U/O 회전 |
| target을 따라가는 IK | target 변경 → `SolveIKToTarget` → DLS IK로 관절 갱신 |

## 한계

- **full transform gizmo가 아니다.** 드래그는 XY 위치 이동 중심이고, Z 이동·회전은 키보드로 처리한다.
- **드래그 평면은 XY 수평면(Z 고정)** 한 종류다. 카메라 대면 평면/자유 3D 드래그는 지원하지 않는다.
- **관전 카메라는 고정 시점**이다. 기본 `ADefaultPawn`은 W/A/S/D·Q/E·마우스를 자유비행 카메라 이동에
  소비해 target 조작 키와 충돌하므로, 이동 바인딩을 끈 전용 `AEndEffectorViewPawn`을 DefaultPawnClass로
  쓴다(카메라는 PlayerStart 위치/방향에 고정). 자유 카메라가 필요하면 별도 pawn/입력을 추가해야 한다.
- **pick/place FSM은 없다.** target 추종까지만 구현한다.
- **nullspace 토글 키는 없다.** on/off는 로봇 Details의 `bUseNullspaceJointLimitAvoidance`로 조작한다
  (InputConfig의 정의된 7개 액션 범위 유지).

## 다음 단계로 확장 가능한 것

- **full transform gizmo**: 위치·회전을 뷰포트에서 직접 드래그.
- **카메라 대면 평면 드래그 / 3D 드래그 토글**: 드래그 평면 선택 옵션화.
- **연속 실시간 IK 추종**: 드래그 중 매 프레임 solve를 기본화하고 속도 제한/보간 추가.
- **pick/place FSM**: target 도달 후 grasp/release 상태기계로 확장(Step B).
