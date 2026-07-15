# 테스트 실행 방법

이 프로젝트의 수학 레이어(FK/IK/동역학)는 전부 **Automation Test**로 검증한다. 액터도 월드도
스폰하지 않는 순수 계산 테스트라 PIE 없이 몇 초 만에 끝난다. 스텝 작업 후 회귀 확인은 이 문서의
절차만 따르면 된다.

엔진: **UE 5.7** / 프로젝트: `RobotSim_Assignment.uproject`

## 사전 준비 — 빌드

**새 `.cpp`/`.h` 파일을 추가한 스텝이면 Live Coding으로는 안 된다.** Live Coding(`Ctrl+Alt+F11`)은
기존 파일 수정은 잘 잡지만 새로 추가된 파일은 놓치는 경우가 많다. 이때는:

1. 에디터를 닫는다
2. `.uproject` 우클릭 → **Generate Visual Studio project files**
3. 솔루션 열고 **Development Editor / Win64** 빌드
4. 에디터 실행

기존 파일만 고쳤다면 Live Coding으로 충분하다.

> 테스트 코드는 `#if WITH_DEV_AUTOMATION_TESTS`로 감싸여 있다. **Development** 또는 **DebugGame**
> 구성에서만 컴파일되며 Shipping에는 들어가지 않는다.

## 방법 1 — Session Frontend (GUI)

**Tools → Session Frontend → Automation 탭**

- 왼쪽 세션 목록에서 **실행 중인 에디터 인스턴스를 먼저 선택**해야 테스트 트리가 나타난다.
  여기서 자주 막힌다 — 트리가 비어 보이면 세션 선택을 확인할 것.
- 트리에서 원하는 노드를 체크하고 **Start Tests**
- 실패 항목을 클릭하면 테스트가 출력한 한국어 실패 메시지(기대값·실제값·오차)가 그대로 보인다.

메뉴에 없으면 아래 콘솔 방식을 쓰면 된다.

## 방법 2 — 콘솔 (제일 빠름)

에디터에서 **`~`**로 콘솔을 열고:

```
Automation RunTests RobotSim.Dynamics.RNEA
```

**부분 문자열 매칭**이라 범위를 자유롭게 조절할 수 있다:

```
Automation List                            // 등록된 테스트 이름 전부 확인
Automation RunTests RobotSim               // 전체 40종 — 스텝 완료 시 회귀 확인용
Automation RunTests RobotSim.Kinematics    // Step A 기구학 5종
Automation RunTests RobotSim.Dynamics      // Params 6종 + RNEA 8종
Automation RunTests RobotSim.Dynamics.RNEA.GravityTorqueMatchesEnergyGradient   // 하나만
```

결과는 Output Log에 `LogAutomationController`로 찍힌다.

> `RobotSim.IK`는 부분 문자열이라 `RobotSim.IK.Nullspace.*`도 함께 잡힌다 (5 + 5 = 10종).

## 방법 3 — 커맨드라인 (에디터 없이)

```
"C:\Program Files\Epic Games\UE_5.7\Engine\Binaries\Win64\UnrealEditor-Cmd.exe" ^
  "C:\Github\RobotSim_Assignment\RobotSim_Assignment\RobotSim_Assignment.uproject" ^
  -ExecCmds="Automation RunTests RobotSim;Quit" ^
  -TestExit="Automation Test Queue Empty" -unattended -nopause -nosplash -log
```

수학 테스트라 월드가 필요 없어 이게 가장 빠르다. 엔진 경로는 설치 위치에 맞게 고칠 것.

## 현재 테스트 목록 (40종)

| 카테고리 | 개수 | 스텝 | 파일 |
|---|---|---|---|
| `RobotSim.Kinematics.*` | 5 | A-01 | `RobotKinematicsTests.cpp` |
| `RobotSim.PoseError.*` | 6 | A-02 | `RobotPoseErrorTests.cpp` |
| `RobotSim.Jacobian.*` | 5 | A-03 | `RobotJacobianTests.cpp` |
| `RobotSim.IK.*` | 5 | A-04 | `RobotDlsIKTests.cpp` |
| `RobotSim.IK.Nullspace.*` | 5 | A-05 | `RobotNullspaceTests.cpp` |
| `RobotSim.Dynamics.Params.*` | 6 | B-01 | `RobotDynamicsParamsTests.cpp` |
| `RobotSim.Dynamics.RNEA.*` | 8 | B-02 | `RobotRNEATests.cpp` |

전부 `Source/RobotSim_Assignment/Private/Tests/` 아래에 있다.

## 회귀가 났을 때 먼저 볼 것

**골든 불변량은 `RobotSim.Kinematics.ZeroPoseFK`다** — Q=0에서 EE = **(105, 0, 120)cm**. Step A
전체가 이 값 위에 서 있으므로, 이게 깨지면 다른 실패는 볼 필요도 없이 기구학부터 고쳐야 한다.

동역학 쪽 핵심 신호는 **`RobotSim.Dynamics.RNEA.GravityTorqueMatchesEnergyGradient`**다. RNEA와
완전히 독립된 경로(신뢰된 FK만으로 계산한 위치에너지 기울기)로 같은 값을 구해 대조하므로,
이게 통과하면 프레임 규약·링크 인덱스·cm→m 경계·중력 부호가 전부 맞은 것이다. 깨지면
`Docs/Steps/STEP_B-02.md`의 "프레임 규약" 절부터 의심할 것.

## 로그 읽기 — `LogRobotSim`

Output Log에서 **`LogRobotSim`으로 필터**하면 로봇 관련 로그만 모아볼 수 있다. 테스트 중에는
B-02의 토크 사이징 리포트가 여기로 나온다:

```
[RNEA] MaxTorqueNm 산정 리포트 — 자세 A: 수평 전개 Q=(0, +90, -90, 0, 0, 0) — J1/J2/J4 최악
[RNEA]   EE = (165.0, 0.0, 60.0)cm
[RNEA]   관절 | 중력토크(N·m) | 현재 MaxTorqueNm(잠정) | 여유배수 | 권장(|τ|×2.5)
[RNEA]     J1  |    -303.374  |       600.0  |      1.98  |       758.4
```

**숫자를 눈으로 검산하는 요령** (B-02 기준):

- J1이 **−303 N·m 근처**여야 한다. 100배 어긋난 값(−3 또는 −30000)이면 cm↔m 경계가 틀린 것이고,
  부호가 +면 중력 방향이 뒤집힌 것이다.
- **J0과 J5는 어떤 자세에서도 정확히 0**이다 (구조적 — 자세한 이유는 STEP_B-02.md 참조).
  0이 아니면 수식이 틀렸다.

## 새 테스트를 추가할 때 (관례)

기존 파일들이 전부 같은 형태를 지킨다. 새 스텝에서도 맞출 것:

```cpp
#include "Misc/AutomationTest.h"
// ... Robot/ 헤더들

#if WITH_DEV_AUTOMATION_TESTS

namespace Robot<이름>TestUtils
{
    constexpr double SomeToleranceCm = 1e-3;
    static void TestXxxNear(FAutomationTestBase& Test, const FString& What, ...);
}

//~============================================================================
// 1. <이 테스트가 무엇을 왜 검증하는지 한국어로>
//~============================================================================
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FRobot<이름><거동>Test,
    "RobotSim.<영역>.<거동>",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter)

bool FRobot<이름><거동>Test::RunTest(const FString& Parameters)
{
    using namespace Robot<이름>TestUtils;
    // ...
    return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
```

- 플래그는 **항상** `EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::ProductFilter`
- 실패 메시지는 한국어 `FString::Printf`로 **기대값·실제값·오차를 전부 인라인**에 넣는다.
  Session Frontend에서 그 한 줄만 보고 원인을 알 수 있어야 한다.
- 액터/월드를 쓰지 않는다. `UObject`가 꼭 필요하면(`URobotConfig` 등)
  `TStrongObjectPtr<T> X(NewObject<T>())`로 만든다 — **CDO(`GetMutableDefault`)를 쓰지 말 것.**
  값을 변조하는 테스트가 CDO를 오염시키면 같은 세션의 다른 테스트에 영향을 준다.
- 새 스텝의 테스트는 **기존 테스트를 한 줄도 수정하지 않고** 추가한다. 기존 테스트를 고쳐야
  통과한다면 그건 회귀다.
