# Decision Log

구현 단계별 범위 설정과 의사결정 근거를 기록한다.

---

## Step A-1: 6축 FK 모델과 관절 계층 검증

### 목표

6-DOF IK 구현 전에 6축 serial robot의 FK, 관절 계층, 단위계, joint limit을 먼저 검증한다.

### 포함 범위

- 6개 revolute joint 모델 (J0 yaw-Z, J1/J2/J4 pitch-Y, J3/J5 roll-X)
- Forward Kinematics (직접 구현, 엔진 IK 기능 미사용)
- End Effector pose(위치 + 쿼터니언) 계산 및 로그
- joint limit clamp
- visual/math consistency check (컴포넌트 트랜스폼 vs 수학 FK 런타임 비교)
- Automation Test (액터 스폰 없는 순수 수학 검증)

### 제외 범위

- IK
- Jacobian
- DLS (Damped Least Squares)
- nullspace limit avoidance
- End Effector 조작 UI (클릭 드래그 / 키보드)
- Pick & Place
- Gripper
- StaticMesh 시각화 (현재는 엔진 기본 도형 디버그 메시만 사용)

### 범위 설정 근거

IK는 FK의 역문제이므로, FK와 좌표계가 검증되지 않은 상태에서 IK를 구현하면 오차의 원인이 FK 모델 정의 오류인지 IK 수렴 문제인지 분리할 수 없다. 따라서 Step A-1에서는 비주얼 완성도보다 수학 모델과 테스트 가능한 FK 기반을 우선했다.

같은 이유로 실제 KUKA 메시 정합에는 시간을 쓰지 않았다. 이후 단계의 Jacobian/DLS/nullspace는 모두 FK와 6D pose 표현 위에서 동작하므로, 이 단계의 정확성이 Step A 전체의 기반이 된다.

### 구현 요약

**구조 — 수학 모델과 비주얼의 분리:**

| 클래스 | 역할 |
|---|---|
| `FSerial6DoFModel` | 순수 수학 FK 모델. UObject/World 비의존이라 액터 스폰 없이 단위테스트 가능 |
| `ASerial6DoFRobotActor` | 6개 revolute joint SceneComponent 부모-자식 계층. 모델을 미러링하는 비주얼 레이어 |
| `FRobotJointLimit` / `FRobot6DPose` / `FRobot6DJointState` | 관절 한계, 6D 자세, 관절 공간 상태 타입 |

**핵심 결정:**

- **파라미터 단일 정의(single source of truth)**: joint axis, link offset, joint limit, tool offset은 `FSerial6DoFModel::CreateDefault()`에만 정의한다. 액터 생성자는 이 값을 그대로 컴포넌트 계층에 복사하므로, 수학과 비주얼이 서로 다른 파라미터를 갖는 오류를 구조적으로 차단한다.
- **단위계 규약**: 내부 계산은 radian/double로 통일하고, degree는 에디터 프로퍼티 경계에서만 사용한다. 변환 지점을 한 함수(`ApplyAnglesFromEditor`)로 고정해 단위 혼동을 방지한다.
- **FK 정의**: 각 관절의 로컬 변환을 `FTransform(FQuat(Axis, Q), LinkOffset)`으로 두고 `World = Local * Parent`(UE child-first 규약)로 누적, 마지막에 tool offset을 합성한다. 이 정의는 SceneComponent의 RelativeLocation/RelativeRotation 의미와 정확히 일치하며, 이것이 시각=수학 일치의 수학적 근거다.
- **런타임 자기 검증**: `CheckVisualMatchesMath()`가 관절 각도 적용 시마다 ToolTip 컴포넌트의 월드 트랜스폼과 수학 FK 결과를 비교해, 위치 0.1cm / 각도 1e-3 rad 초과 시 Warning을 남긴다. 미러링이 깨지면 즉시 로그로 드러난다.
- **joint limit**: `SetJointAngles()` 진입 시 항상 클램프한다. 한계를 벗어난 에디터 입력은 클램프 결과가 Details 패널로 되돌려 써져 실제 적용 값을 보여준다.

**기준 자세**: Q=0에서 EE는 actor space 기준 **(105, 0, 120)cm** (X = 60+20+15+10, Z = 40+20+60 — 링크 오프셋의 단순 합으로 손 계산 가능하도록 설계).

### 검증 결과

Automation Test 5종 **all success** (허용 오차: 위치 1e-3 cm, 각도 1e-4 rad).

| 테스트 | 검증 내용 |
|---|---|
| `RobotSim.Kinematics.ZeroPoseFK` | Q=0에서 EE = (105, 0, 120), 자세 identity — golden pose |
| `RobotSim.Kinematics.J0YawRotatesAboutZ` | J0=90°에서 (0, 105, 120); 임의 각도에서 XY 반경 105, 높이 120 불변 |
| `RobotSim.Kinematics.J1PitchMovesEE` | J1=90°에서 (60, 0, −45); 어깨 피벗으로부터의 거리 보존 |
| `RobotSim.Kinematics.ToolRollInvariance` | J5 회전 시 EE 위치 불변(툴 오프셋이 J5 축과 동일선상), 자세만 `FQuat(X, θ)` |
| `RobotSim.Kinematics.JointLimitClamp` | 한계 초과 입력이 정확히 Min/Max로 클램프, 범위 내 값 불변 |

단일 관절 테스트는 golden 값 일치뿐 아니라 **불변량(반경/거리/위치 보존)**을 함께 검증하므로, FK 합성 순서나 회전 컨벤션 오류가 있으면 즉시 실패하도록 설계했다. 에디터에서는 관절별 수동 각도 변경, EE pose 주기 로그, 디버그 좌표축(관절은 컴포넌트에서, EE는 수학 FK에서 그려 불일치가 화면에서도 보임)으로 확인했다.

### 다음 단계

Step A-1.5에서 StaticMesh 시각화 레이어를 추가한다. 단, 수학 FK 모델(`FSerial6DoFModel`)은 변경하지 않는다 — 시각화는 관절 프레임의 형제 컴포넌트로만 붙여 FK에 영향을 주지 않는 현재 구조를 유지한다.
