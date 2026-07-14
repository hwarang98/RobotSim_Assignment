# Step A-1.5a: Bone Mapping Probe (본 매핑 조사 도구)

## 목표

SkeletalMesh를 FK에 바로 동기화하기 전에, **각 본이 실제로 어떤 파츠를 움직이는지 조사하는 Bone Probe 기능**을 추가한다. 조사 결과로 `JointBoneNames` 매핑을 확정하는 것이 이 단계의 산출물이다. 이번 단계에서는 FK와 SkeletalMesh를 자동 동기화하지 않는다.

## 범위

**포함:**
- `DumpSkeletonBones()`: 본 인덱스/이름/부모를 로그로 출력 (CallInEditor)
- `ApplyBoneProbe()`: 지정한 본 하나만 로컬 축 기준으로 회전 (CallInEditor)
- `ResetBoneProbe()`: ref pose로 복원 (CallInEditor)
- Probe 프로퍼티: `ProbeBoneName`, `ProbeAxis`(X/Y/Z enum), `ProbeAngleDeg`, `bShowProbeMesh`
- Probe 모드(`bShowProbeMesh`) 동안: 본 매핑 미완성이어도 메시 표시, FK 자동 동기화 중지, 매핑 Warning 억제

**제외:** FK 수학 모델/joint axis/link offset/테스트 변경, IK/Jacobian/DLS/PickPlace/Gripper.

## 근거

현재 에셋의 본 구조는 다음과 같다:

```
Bone
└─ Bone_001
   ├─ Bone_002
   │  └─ Bone_003 → Bone_004 → Bone_005 → Bone_006 → Bone_007
   ├─ Bone_008
   └─ Bone_009
```

- `Bone_008`, `Bone_009`는 메인 체인 밖의 보조 본이므로 J0~J5 후보에서 제외한다.
- 본 이름이 의미를 담고 있지 않으므로(`Bone_00N`), 이름 순서만 보고 J0~J5에 기계적으로 매핑하면 틀릴 수 있다. 각 본을 하나씩 돌려 실제로 움직이는 파츠를 눈으로 확인한 뒤 매핑을 확정해야, A-1.5의 본 동기화(그리고 이후 IK 시각 검증)가 올바른 전제 위에 선다.
- Probe는 ref pose에서 시작해 **한 번에 본 하나만** 회전시키므로 (누적 없음) 관찰이 명확하다.

## 구현 요약

| 항목 | 내용 |
|---|---|
| `ApplyBoneProbe()` | 전체 ref pose 복원 → 대상 본의 컴포넌트 공간 변환(ref pose 부모 체인 누적)에 로컬 축 델타 회전 합성 → `SetBoneTransformByName`. 본이 없으면 Warning. Probe 모드 자동 켜기 |
| `ResetBoneProbe()` / `ResetPoseToRefPose()` | 모든 본 `ResetBoneTransformByName` + `RefreshBoneTransforms` |
| `DumpSkeletonBones()` | `FReferenceSkeleton` 순회, `[인덱스] 이름 (parent: 부모)` 형식 출력 |
| `bShowProbeMesh` | 켜면: 매핑 미완성이어도 메시 표시 + `SyncSkeletalPoseToMath()` 중지(Probe 포즈 보호) + 매핑 Warning 억제. 매핑 확정 후 끄면 기존 A-1.5 동기화 경로로 복귀 |
| 에셋 재적용 최적화 | `ApplySkeletalMeshVisual()`이 에셋이 실제로 바뀐 경우에만 reinit — 무관한 프로퍼티 편집이 Probe 포즈를 초기화하지 않도록 |

FK 수학 모델과 디버그 링크는 변경 없음. Automation Test 5종은 순수 수학 검증이므로 영향 없음.

## 검증 (본 매핑 조사 절차)

1. 빌드 후 로봇 액터 선택 → Robot|Visual → `SkeletalMeshAsset` 할당.
2. Robot|BoneProbe → `bShowProbeMesh` 켜기 → 메시가 ref pose로 표시되는지 확인.
3. `DumpSkeletonBones` 버튼 → Output Log에서 본 계층 확인.
4. `ProbeBoneName`을 `Bone_001` ~ `Bone_007`로 바꿔가며 `ApplyBoneProbe` 실행 (필요 시 `ProbeAxis`/`ProbeAngleDeg` 조정) → **어떤 파츠가 움직이는지 본별로 기록**.
5. 기록을 바탕으로 J0(베이스 yaw)~J5(툴 roll) 순서에 맞는 본을 `JointBoneNames[0..5]`에 입력.
6. `ResetBoneProbe` → `bShowProbeMesh` 끄기 → A-1.5 동기화 경로가 동작하며 관절 각도에 본이 따라오는지 확인.
7. Automation Test 5종(`RobotSim.Kinematics.*`) all success 확인.

## 조사 결과 (본 매핑 확정)

Probe로 본별 움직임을 관찰한 결과, 구조는 파츠/링크 교대 형태로 확인됐다: 파츠(001/003/005/007)는 제자리 회전(roll·yaw), 파츠 사이의 연결 링크(002/004/006)는 굽힘(pitch)을 담당한다. 이는 6R 모델의 관절 회전 타입(J0=Z yaw, J1·J2·J4=Y pitch, J3·J5=X roll)과 정확히 일치한다.

| 관절 | 축 | 본 | 담당 |
|---|---|---|---|
| J0 | Z (yaw) | `Bone_001` | 베이스 상부 제자리 회전 |
| J1 | Y (pitch) | `Bone_002` | 001→003 링크 (어깨 굽힘) |
| J2 | Y (pitch) | `Bone_004` | 003→005 링크 (팔꿈치 굽힘) |
| J3 | X (roll) | `Bone_005` | 전완 파츠 제자리 (비틀림) |
| J4 | Y (pitch) | `Bone_006` | 005→007 링크 (손목 까딱) |
| J5 | X (roll) | `Bone_007` | 툴/플랜지 파츠 제자리 (비틀림) |

**제외**: `Bone_003`(상완 파츠 — J1 링크에 종속, 독립 관절 없음), `Bone_008`/`Bone_009`(체인 밖 보조 본).

> **개정 주의 (→ Step A-1.5b):** 위 표의 J3→`Bone_005` 매핑은 이후 폐기됐다. 실제 구동 결과 Bone_005는 상하 pitch만 담당하는 구조라 roll(J3)을 얹으면 외형이 비틀린다. 따라서 **J3는 None(미매핑)으로 변경**됐고, 부분 매핑을 허용하도록 코드를 수정했다. 최종 매핑과 근거는 `STEP_A-01.5b.md` 참조.

## 다음 단계

전완 roll(J3) 구조 불일치가 확인됨 → Step A-1.5b에서 부분 매핑(J3 미매핑) + approximate overlay로 정리한다.
