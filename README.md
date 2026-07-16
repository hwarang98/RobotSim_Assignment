# RobotSim_Assignment — 자동창고 물류 셀 (6-DOF 로봇팔 시뮬레이션)

Unreal Engine **5.7** / C++ 중심 프로젝트. **엔진 내장 IK 플러그인과 물리 엔진 기반 로봇 구동을
사용하지 않고**, 6-DOF 시리얼 로봇팔의 기구학·동역학·작업 제어를 순수 수학 레이어로 직접 구현했다.

> ⚠️ **UE 프로젝트와 전체 문서는 [`RobotSim_Assignment/`](RobotSim_Assignment/) 하위 폴더에 있다.**
> `.uproject`·소스·에셋·`Docs/`가 모두 그 안에 있으므로, 열 때는 하위 폴더의
> `RobotSim_Assignment.uproject` 를 연다. 이 파일은 저장소 랜딩 요약이고,
> **상세 문서는 → [`RobotSim_Assignment/README.md`](RobotSim_Assignment/README.md)** 이다.

## 구현 범위 (STEP A~D)

| STEP | 내용 | 상태 |
|---|---|---|
| **A** | 6R FK · 6D pose error(quaternion-log) · numerical Jacobian · DLS IK · nullspace 관절한계 회피 · EE 타깃 클릭/드래그+키보드 · ToolOffset 캘리브레이션 | ✅ 전부 |
| **B** | 링크 질량·관성·토크한계 파라미터(B-01) · **RNEA 역동역학**(B-02) | ✅ 부분 |
| **C** | 픽앤플레이스 FSM · 고정 타임스텝 · **물리값 CSV 로깅**(C-01) · **멀티로봇 작업 할당 dispatcher**(C-02) | ✅ |
| **D** | UMG 텔레메트리 + RNEA 토크 게이지(D-01) · **모션 CSV 기록↔재생 대칭**(D-02) | ✅ |

과제 안내가 "전부 완성 불필요 — 범위 우선순위 문서화 필수, 판단력도 평가"라 명시하므로, **무엇을 왜
구현/포기했는가**를 [`Docs/SCOPE_AND_PRIORITY.md`](RobotSim_Assignment/Docs/SCOPE_AND_PRIORITY.md)가 답한다
(토크 제어 구동·시간최적 S-curve·마찰 grasp·N층 적재·Sim2Real 실기는 근거와 함께 의도적으로 포기).

## 필수 심화 항목 대응

| 필수 항목 | 충족 | 근거 |
|---|:---:|---|
| **멀티로봇(2대+) 작업 할당** | ✅ | `APickPlaceDispatcher` — 도달성 캐시 기반 결정론 배급 + 슬롯 상호배제 (C-02) |
| **고정 타임스텝 결정론 재현** | ✅ | accumulator 고정 스텝 + double 정밀도 + 프레임레이트 독립 궤적 |
| **물리값 CSV 로깅** | ✅ | 매 스텝 관절각·각속도·EE 자세·**RNEA 토크** → `Saved/RobotSim/*.csv` |
| **범위 우선순위 문서화** | ✅ | [SCOPE_AND_PRIORITY.md](RobotSim_Assignment/Docs/SCOPE_AND_PRIORITY.md) |

## 빠른 시작

1. [`RobotSim_Assignment/RobotSim_Assignment.uproject`](RobotSim_Assignment/) 를 UE 5.7로 연다 (C++ 모듈 빌드 프롬프트 수락).
2. 멀티로봇 씬 맵에서 **PIE 실행** → dispatcher가 박스를 두 로봇에 배분, UMG 대시보드에 FSM 상태·관절각·토크 게이지·할당 현황 표시.
3. 사이클 종료 시 `Saved/RobotSim/` 에 로봇별 물리값 CSV + `Dispatch.csv`(할당 이벤트) 생성.
4. 대시보드 재생 컨트롤로 저장된 모션 CSV를 되돌려 **결정론적 재현** 확인 (D-02).

> 전체 실행 절차·조작표·테스트·설계 트레이드오프는 하위
> **[프로젝트 README](RobotSim_Assignment/README.md)** 에 있다.

## 주요 문서

| 문서 | 내용 |
|---|---|
| [프로젝트 README](RobotSim_Assignment/README.md) | 전체 실행법·수학 상세·테스트(40종)·PDF 요구사항 대응표(A~E) |
| [범위 우선순위 근거](RobotSim_Assignment/Docs/SCOPE_AND_PRIORITY.md) | 무엇을 왜 구현/포기했는가 (필수 문서) |
| [단계별 문서 17편](RobotSim_Assignment/Docs/Steps/) | STEP A-01 ~ D-02 설계 근거·수식 유도·검증 |

## 테스트

수학 레이어는 UObject/World 비의존 plain struct라 액터 스폰 없이 단위테스트한다.
총 **40종** Automation Test (7그룹). 에디터 → `Tools` → `Session Frontend` → `Automation` →
필터 `RobotSim` → `Start Tests`, 또는 콘솔 `Automation RunTests RobotSim`.

## AI 사용 범위

이 프로젝트는 **AI 페어 프로그래밍(Anthropic Claude Code)** 으로 진행했다.

- **AI 관여**: 단계별 구현 계획·C++ 코드·주석·문서 초안, 수식 유도 정리와 골든 테스트 값 손계산 교차검증, 과제 PDF 판독.
- **사람 소유**: **모든 빌드·PIE·테스트 실행과 결과 검증**, 범위 우선순위 결정, 에디터 작업 전부(에셋·UMG·레벨 배치·데모 녹화), 실환경 제약의 발견.
- **방식**: 각 단계를 프롬프트(스펙)로 정리 → 사람이 제약을 덧붙여 확정 → 구현. 저장소의 모든 코드는 사람이 빌드·테스트로 검증한 상태로 커밋됐다.

자세한 구분은 [프로젝트 README §10](RobotSim_Assignment/README.md#10-ai-사용-범위).

## 제출 전 점검

- 소스·설정에 API 키·비밀번호·개인정보 없음. `Config/DefaultEngine.ini` 의 `SecurityToken` 은 UE가 자동 생성하는 UDP 메시징 세션 ID로 보안 위협이 아니다.
- zip 제출 시 `Saved/`·`Intermediate/`·`Binaries/`·`DerivedDataCache/` 제외 권장.
