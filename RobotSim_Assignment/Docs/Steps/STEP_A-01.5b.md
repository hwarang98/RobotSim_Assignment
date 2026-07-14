# Step A-1.5b: 부분 본 매핑 (Skeletal Mesh를 approximate overlay로)

## 목표

KUKA SkeletalMesh가 수학 모델(`FSerial6DoFModel`)의 6R 구조와 1:1로 일치하지 않는 문제를 시각화 레이어에서만 해결한다. **수학 모델과 Automation Test는 절대 수정하지 않는다.** SkeletalMesh는 외형 안정성을 위한 approximate overlay로만 사용하고, 6-DOF 전체의 정답 표시는 계속 디버그 링크/ToolTip(수학 FK)이 담당한다.

## 배경 — 구조 불일치

Bone Probe(A-1.5a) 조사와 실제 구동 확인 결과, 이 메시에는 **전완 roll(J3) 자유도가 없다**. J3 후보였던 `Bone_005`는 구조상 상하 pitch만 담당하며, 여기에 roll을 얹으면 로봇 외형이 좌우로 비틀려 기존 구조를 벗어난다.

- 수학 모델의 관절 축: J0=Z(yaw), J1=Y, J2=Y, **J3=X(roll)**, J4=Y, J5=X(roll) — 정통 6R.
- 메시의 실제 자유도: yaw + pitch들 + 툴 roll (전완 roll 없음).

두 로봇의 관절 위상이 다르므로, "메시가 절대 안 깨짐"과 "6R 수학을 완벽히 시각화"를 동시에 만족할 수 없다. 수학 모델이 source of truth이고 테스트가 그 위에 서 있으므로, **포기하는 쪽은 메시의 시각적 fidelity**로 정한다.

## 변경 (visual retarget only)

1. **J3 → None 매핑**. 생성자 기본값에서 `JointBoneNames[3] = NAME_None`. Bone_005에 roll을 얹지 않는다.
2. **부분 매핑 허용**. 이전에는 `JointBoneNames` 중 하나라도 비면 메시 전체를 숨겼으나(`bSkeletalBoneMapValid`), 이제 에셋만 할당되면 메시를 표시하고(`bSkeletalMeshActive`) **미매핑 관절만 개별 skip**한다.
3. **명시적 로그**:
   - None 관절 → `Log`: "J%d: visual-only unmapped joint — SkeletalMesh 동기화에서 skip합니다".
   - 이름은 있으나 에셋에 없는 관절 → `Warning`: 오타 가능성 안내 후 그 관절만 skip (메시는 계속 표시).
4. **디버그 링크/ToolTip/CheckVisualMatchesMath 불변** — 수학 FK 기준 6-DOF 전체를 계속 표시·검증한다. SkeletalMesh는 검증 대상이 아니다.
5. **포즈 버퍼 크래시 가드** (J3=None 허용으로 드러난 잠복 버그): `ResetPoseToRefPose()`가 `GetNumBones()`(ref skeleton 기준, nonzero)로 순회하다 아직 할당되지 않은 `BoneSpaceTransforms`(size 0)를 인덱싱해 assert 크래시했다. 이제 `BoneSpaceTransforms`를 ref pose 크기와 대조 후 직접 덮어쓰고, `ApplySkeletalMeshVisual()`은 버퍼가 비어 있으면 `SetSkinnedAssetAndUpdate`로 재할당한다.

## 확정 매핑

| 관절 | 축 | 본 | 비고 |
|---|---|---|---|
| J0 | Z (yaw) | `Bone_001` | 베이스 제자리 회전 |
| J1 | Y (pitch) | `Bone_002` | 어깨 굽힘 |
| J2 | Y (pitch) | `Bone_004` | 팔꿈치 굽힘 |
| J3 | X (roll) | **None** | 전완 roll — 메시 미지원, 시각화 skip |
| J4 | Y (pitch) | `Bone_006` | 손목 까딱 |
| J5 | X (roll) | `Bone_007` | 툴 제자리 회전 |

제외 본: `Bone_003`(상완, J1에 종속), `Bone_005`(상하 전용 — J3 roll 미적용), `Bone_008`/`Bone_009`(체인 밖 보조 파츠).

## 트레이드오프 (명시)

- **얻는 것**: 메시가 어떤 자세에서도 외형이 깨지지 않음. J3에 roll이 강제되지 않는다.
- **잃는 것**: IK가 J3≠0 해를 쓸 때 메시의 손목·툴 방향이 수학과 어긋나 보일 수 있다. 이때도 **디버그 링크는 항상 수학 FK 그대로** 6-DOF를 그리므로 진짜 기구학은 링크로 검증 가능하다.
- 완벽한 시각 fidelity가 필요해지면, 전완 roll 본을 가진 메시로 교체하거나 J3용 보조 본을 리깅하는 것이 향후 개선 경로다.

## 검증

1. 리빌드 후 로봇 액터 선택 → `SkeletalMeshAsset` 할당.
2. Output Log에 `J3: visual-only unmapped joint ...` 정보 로그 1회 출력, 나머지 관절 Warning 없음 확인.
3. `bShowProbeMesh` 끈 상태에서 메시가 정상 표시(숨김 아님) 확인.
4. `JointAnglesDeg`로 J0/J1/J2/J4/J5를 각각 움직여 대응 본이 디버그 링크와 같은 방향으로 회전, **J3를 움직여도 메시는 비틀리지 않음**(디버그 링크만 forearm roll 반영) 확인.
5. 메시 할당 전후 EE pose 로그 동일, `RobotSim.Kinematics.*` 5종 all success 확인 (수학/테스트 무변경).

## 다음 단계

Step A-2: Jacobian 기반 IK (DLS, nullspace joint limit avoidance). 수학 모델 위에서 구현하며 이 시각화 레이어는 변경하지 않는다.
