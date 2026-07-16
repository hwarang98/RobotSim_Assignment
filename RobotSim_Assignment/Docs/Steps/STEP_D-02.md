# Step D-02: PickPlace 모션 CSV 재생 (고정 타임스텝 대칭 재현)

## 목표

D-01 한계 4번("모션 로드가 없다")을 지운다. 저장 방향(`RecordCsvRow` / `WriteCsvToDisk`)의 **정반대** —
CSV를 읽어 팔을 고정 타임스텝으로 재생하는 방향 — 을 만든다. 기록↔재생이 대칭이 되어
**결정론적 재현**이 눈으로 증명된다.

## 범위 밖 (이번 단계에서 구현하지 않음)

박스 재생, dispatcher 전체 동기 재생, `time_s` 기반 리샘플링, 신규 자동화 테스트,
수학 레이어 / `ASerial6DoFRobotActor` 수정. 기존 테스트 40종 무수정.

## 무엇을 재생하는가 — 팔 관절각만

CSV의 `q0_deg`~`q5_deg`를 프레임마다 `ActiveState`에 되넣고 `SetJointAngles`로 민다.
**박스는 재생 대상에서 뺀다.**

이유: `box_index` / `box_held`는 "붙었다/떨어졌다"는 표시일 뿐, **어느 박스인지·그게 지금 어디
있는지를 CSV가 담지 못한다.** 억지로 재생하면 저장 때와 다른 박스 상태(개수·위치·dispatcher 배급)
에서 순간이동이 일어나 **거짓 재현**이 된다. 팔 궤적만 재생하는 것이 정직하다.

이것이 이 단계의 핵심 한계이며, 데모에서 "재생 중 박스는 따라오지 않는다"는 점을 그대로 보여준다.

## 컬럼은 이름으로 찾는다 (인덱스 하드코딩 금지)

`RecordCsvRow`의 헤더는
`time_s,phase,box_index,q0_deg,...,q5_deg,qd...,ee...,box_held,tau0_nm,...`이고,
**tau는 끝에 붙인다**는 규약이 기록부 주석에 있다(중간 삽입 여지가 있다). 지금은 `q0_deg`가
인덱스 3이지만, 컬럼 순서가 한 번만 바뀌면 고정 인덱스는 **조용히 틀린 열**을 읽는다.

→ `LoadMotionCsv`는 **0행(헤더)을 파싱해 `q0_deg`~`q5_deg`의 실제 인덱스를 찾고** 그 열만 읽는다.
헤더에 여섯 이름이 다 없으면 로드 실패로 처리한다. **기록↔재생 대칭의 계약은 컬럼 이름이지
위치가 아니다.**

## degree → radian 역변환

기록은 `FMath::RadiansToDegrees(ActiveState.Q[i])`로 degree를 쓴다. 재생은 반드시
`FMath::DegreesToRadians`로 되돌려 `ActiveState.Q[i]`에 넣는다. 빠뜨리면 팔이 **57배 꺾인다.**
원칙은 "기록이 쓴 것과 정확히 역으로"다.

## bReplayActive — FSM·dispatcher와의 공존

재생은 **FSM 단계가 아니라 그 위에 얹는 별도 모드다.** `EPickPlacePhase` enum은 건드리지 않는다.

- `Tick` 앞에서 `bReplayActive`면 **재생 스테퍼만 돌리고 FSM 전진 블록을 통째로 건너뛴다.**
  그 뒤 공통 경로(`SetJointAngles`)로 로봇에 밀어 로봇 Tick의 홈 자세 되쓰기를 이긴다.
- `IsIdle()`이 재생 중 **false**를 반환한다 → dispatcher가 이 로봇에 배급하지 않는다.
  (Phase는 Idle이어도 팔은 재생 중이므로, 배급하면 두 모드가 같은 관절을 두고 다툰다.)
- 마지막 프레임을 지나면 `bReplayActive=false` + Idle 복귀.
- `PlayReplay`는 진행 중이던 파지/배급을 먼저 정리한다(박스 놓기, dispatcher에 반납) — 재생과
  FSM이 같은 관절을 다투지 않도록.

## 타임스텝

CSV 한 행 = 고정 스텝 하나이므로, 재생도 `FixedTimeStepSec`마다 프레임 하나를 전진시킨다
(FSM과 같은 누적기 패턴, 단 별도 누적기 `ReplayTimeAccumulatorSec` — 두 모드는 공존하지 않으므로 분리).

**한계**: 다른 `FixedTimeStepSec`로 기록된 파일은 재생 속도가 달라진다. 파일은 스텝 간격이 아니라
프레임 나열만 담으므로, 현재 액터의 `FixedTimeStepSec`로 재생된다. `time_s` 컬럼 기반 정밀
리샘플링은 gold-plating이라 뺐다 — 같은 설정으로 기록·재생하는 통상 경로에서는 불필요하다.

## 파싱

`FFileHelper::LoadFileToStringArray` → 0행 헤더로 컬럼 인덱스 해석 → 각 데이터 행을 콤마 분리해
여섯 `q` 열 파싱. 숫자 파싱 실패/컬럼 부족 행은 스킵하되 **경고는 한 번만**(스팸 금지).
파일 없음 / 프레임 0개 / 헤더 불일치면 로드 실패 로그 후 재생하지 않는다. 로그 접두사 `[PickPlaceTask]`.

## 노출 표면 (D-01과 대칭)

```cpp
UPROPERTY(EditAnywhere, Category="PickPlace|Replay")
FString ReplayMotionFileName;      // 비어 있으면 이 액터의 CsvFileName 사용

UFUNCTION(CallInEditor, BlueprintCallable, Category="PickPlace")
void LoadMotionCsv();              // 파싱만 → ReplayFrames 채움 (여러 번 재생 가능하게 분리)
void PlayReplay();                 // 처음부터 재생. 프레임 비었으면 LoadMotionCsv 자동 1회
void StopReplay();                 // 중단 후 Idle

UFUNCTION(BlueprintPure, Category="PickPlace|UI")
bool IsReplaying() const;
float GetReplayProgress() const;   // 0~1
```

로드와 재생을 분리한 이유: 같은 파일을 여러 번 재생할 때 매번 파싱하지 않기 위해서다. 다만
`PlayReplay`가 빈 프레임이면 로드를 자동 호출해 "버튼 한 번" 편의는 남긴다.

## 신규 테스트를 만들지 않은 근거

D-01과 동일하다. 재생은 월드/액터(`SetJointAngles`, 로봇 컴포넌트)에 묶여 있어 순수 단위테스트가
되지 않고, 그 하네스를 만들 시간이 없다. 검증은 **PIE에서 눈으로**: 사이클을 한 번 돌려 CSV를
만든 뒤 `PlayReplay`로 같은 팔 궤적이 재현되는지 확인한다. 기록↔재생 대칭 자체가 이미 계약이므로,
CSV의 `q` 열이 그대로 관절각으로 되돌아온다는 것이 재현의 증거다.

## 변경/추가 파일

- `Public/Robot/PickPlaceTaskActor.h`, `Private/Robot/PickPlaceTaskActor.cpp`
  — `ReplayMotionFileName` 프로퍼티, `LoadMotionCsv`/`PlayReplay`/`StopReplay`,
    `IsReplaying`/`GetReplayProgress`, `StepReplay` + `Tick`/`IsIdle`/`ResetCycle` 재생 분기.
- `Docs/Steps/STEP_D-02.md` (이 문서), `Docs/Steps/STEP_D-01.md` 한계 4번 갱신.

`ASerial6DoFRobotActor`, 수학 레이어, 테스트 40종은 무수정.

## 여러 사이클 누적 재생

한 번의 PIE 실행 안에서는 이미 여러 박스가 한 파일에 누적된다 (StartCycle이 첫 배급 전 한 번만
버퍼를 비우므로). 그래서 "여러 사이클 누적"이 새 기능이 되려면 **여러 실행(PIE 재시작)에 걸쳐**
누적해야 한다.

`bAccumulateMotionAcrossRuns`(기본 꺼짐)를 켜면 새 사이클이 기존 CSV **뒤에 이어붙는다.** 끄면 매
실행이 파일을 덮어써 항상 "가장 최근 사이클"만 남는다(기존 동작). 켜두면 PIE를 여러 번 돌린 궤적이
한 파일에 시간순으로 쌓이고, `PlayReplay`가 그 전체를 이어서 재생한다 — 재생 코드는 **그대로**다
(파일에 있는 걸 통째로 트니까). 누적을 처음부터 다시 시작하려면 `ClearMotionCsv`를 누른다.

**구현은 디스크 append가 아니라 "기존 파일을 읽어 앞에 붙여 다시 쓰기"다.** append는 `ForceUTF8`이
붙이는 BOM이 파일 중간에 박혀 파서를 깨뜨린다. 매번 `[헤더] + [과거 행] + [이번 행]` 전체를
덮어쓰면 BOM은 항상 맨 앞 한 번뿐이고, 사이클 도중 여러 번 flush해도 파일이 온전하다.

**기존 파일의 헤더가 현재 헤더와 다르면**(컬럼 스키마 변경, 예: tau 추가로 25→31) 섞지 않고 경고 후
새로 시작한다 — 스키마가 다른 행이 섞이면 재생/분석이 조용히 틀어지기 때문이다. D-02의 "컬럼 이름
계약"과 같은 원칙이다.

## 한계

1. **박스는 재생되지 않는다** (위 참조). 팔만 재현되고 파지/적재는 재생 중 일어나지 않는다.
2. **다른 FixedTimeStepSec로 기록된 파일은 재생 속도가 달라진다.** `time_s` 기반 리샘플링 없음.
3. **단일 로봇 재생이다.** dispatcher가 여러 로봇을 동시에 시간 정렬해 재생하지는 않는다 —
   각 태스크 액터가 자기 CSV를 독립적으로 재생한다.

## 다음 단계

기록↔재생 대칭이 완성됐으므로, 남은 것은 STEP B 동역학 축(B-03 순동역학 → B-06 computed torque)이다.
그쪽이 들어오면 재생 대상에 토크/가속도가 의미를 갖게 되어 이 표면을 확장할 수 있다.
