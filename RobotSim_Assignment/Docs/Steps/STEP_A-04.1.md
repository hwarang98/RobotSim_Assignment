# Step A-04.1: 액터 정리 — 로봇 정의 DataAsset로 통합 + BoneProbe 제거 (housekeeping)

## 목표

`ASerial6DoFRobotActor`가 커지는 것을 막기 위한 정리 단계. 기능 추가가 아니라 구조 개선이다.
1. 로봇 정의(기구학 + SkeletalMesh 배선)를 코드 하드코딩 대신 **하나의 DataAsset**으로 통합한다.
   → 에셋 하나 = 로봇 하나. 복제/교체/리뷰가 쉬워지고, 액터는 얇은 뷰어가 된다.
2. 본 매핑이 확정되어 역할이 끝난 **Bone Probe 조사 도구**를 제거해 액터를 슬림하게 한다.

## 1. URobotConfig (신규 DataAsset)

- 신규 `Public/Robot/RobotConfig.h`, `Private/Robot/RobotConfig.cpp`
- `USTRUCT FRobotJointConfig` — 관절 하나의 authoring 표현: `Axis`, `LinkOffsetCm`, `MinDeg/MaxDeg/MaxVelDegPerSec`(도 단위).
- `UCLASS URobotConfig : public UDataAsset`
  - 기구학: `BaseTransform`, `Joints[6]`, `ToolOffset` + `FSerial6DoFModel ToModel() const`
  - 시각화 배선: `SkeletalMeshAsset`, `JointBoneNames[6]`
- 각도 도→radian 변환은 `ToModel()` 경계에서만. Axis는 정규화(0 벡터는 기본 축으로 대체), Min/Max 뒤집힘 방어.
- 기본값은 생성자에서 `CreateDefault()` + 확정된 KUKA 본 매핑으로 채운다 → **새 에셋은 SkeletalMesh만 지정하면 기본 로봇**. 하드코딩 숫자를 두 곳에 두지 않는다.

### 순수 수학 레이어 불변식 유지
`FSerial6DoFModel`은 여전히 UObject/World에 의존하지 않는다. DataAsset은 값만 보관하고 `ToModel()`로
순수 struct를 "생성"만 하므로, 수학 레이어는 DataAsset을 **참조하지 않는다**. 따라서 단위테스트
(`RobotSim.*`)는 그대로 `CreateDefault()`를 쓰며 **수정·회귀 없음**.

## 2. 액터 연동 (RobotConfig)

- `ASerial6DoFRobotActor`에 `UPROPERTY TObjectPtr<URobotConfig> RobotConfig` 추가 (Category `Robot|Config`).
- 액터에서 `SkeletalMeshAsset`/`JointBoneNames` UPROPERTY 제거 → RobotConfig가 소유. 액터는 private
  `GetConfiguredSkeletalMesh()`/`GetConfiguredBoneName(i)`로 해석(RobotConfig가 None이면 각각 nullptr/None).
- 신규 private `RefreshFromConfig()`: `Model = RobotConfig ? RobotConfig->ToModel() : CreateDefault()` → `MirrorModelToComponents()` → `ApplyAnglesFromEditor()`.
- 신규 private `MirrorModelToComponents()`: 모델의 `LinkOffsets`/`ToolOffset`을 관절 컴포넌트 체인에 재반영(회전은 `ApplyJointState`가 담당).
- `OnConstruction`이 `RefreshFromConfig()` → `ApplyLinkVisuals()` → `ApplySkeletalMeshVisual()` 호출.
  `PostEditChangeProperty`의 `RobotConfig` 분기는 `RefreshFromConfig()` + `ApplySkeletalMeshVisual()`를 함께 호출.
- **폴백 규약**: `RobotConfig`가 None이면 기구학은 코드 기본값(기존과 동일), SkeletalMesh 시각화는 비활성(디버그 링크만 표시).
- **표시 토글** `bShowSkeletalMesh`는 인스턴스별 디버그 프리퍼런스라 액터에 남긴다(에셋/본 이름은 config).

### 마이그레이션 주의 (1회 재지정)
`SkeletalMeshAsset`/`JointBoneNames`를 액터에서 제거했으므로, 배치된 액터에 저장돼 있던 메시 할당은
로드 시 사라진다. **URobotConfig 에셋을 만들어 SkeletalMesh를 지정하고 액터의 RobotConfig에 물리는
1회 재지정**이 필요하다(본 이름 기본값은 에셋에 이미 들어 있다).

## 3. Bone Probe 제거

확정된 본 매핑(J0=Bone_001 … J5=Bone_007, J3=None; STEP_A-01.5b 참조) 이후 역할이 끝난 조사 도구를 삭제:
- 제거: `EProbeAxis` enum, `ApplyBoneProbe`/`ResetBoneProbe`/`DumpSkeletonBones` CallInEditor, `ProbeBoneName`/`ProbeAxis`/`ProbeAngleDeg`/`bShowProbeMesh` 프로퍼티, `ProbeAxisToVector`.
- 유지: `ResetPoseToRefPose()` — 동기화 전 미매핑 본 초기화에 계속 쓰인다(더 이상 Probe 전용 아님).
- `ApplySkeletalMeshVisual`/`SyncSkeletalPoseToMath`에서 `bShowProbeMesh` 분기 제거 → 항상 매핑 보고 + 동기화하는 단순 경로.

## 한계 / 주의

- **디버그 도형 메시**(Cube/Cylinder)의 오프셋·스케일은 기본 링크 길이에 맞춰 하드코딩돼 있다.
  RobotConfig로 링크 길이를 크게 바꾸면 이 도형들은 어긋날 수 있다(순수 시각용). **좌표 프레임/FK는 정확**하며,
  `CheckVisualMatchesMath()`가 계속 검증한다.
- DataAsset **내부** 값을 편집해도 액터의 `PostEditChangeProperty`는 호출되지 않는다. 반영하려면 액터를
  다시 선택/이동해 `OnConstruction`을 트리거한다.

## 변경/추가 파일

- 신규 `Public/Robot/RobotConfig.h`, `Private/Robot/RobotConfig.cpp`
- 신규 `Docs/Steps/STEP_A-04.1.md` (본 문서)
- 수정 `Public/Robot/Serial6DoFRobotActor.h`, `Private/Robot/Serial6DoFRobotActor.cpp`

## 검증

- `Content`에 `URobotConfig` 에셋 생성(우클릭 → Miscellaneous → Data Asset → RobotConfig) → 기구학 기본값이 기본 로봇과 같은지 확인.
- 에셋에 KUKA SkeletalMesh 지정 → 액터 `RobotConfig`에 에셋 물리기 → 기존과 동일하게 동작(각도/디버그 링크/SkeletalMesh, 본 매핑 기본값 Bone_001…007·J3 None) 확인.
- 기구학 값(예: 링크 길이)을 바꿔 로봇 형태가 데이터로 바뀌는지 확인. 에셋 내부 편집 후엔 액터 재선택/이동으로 `OnConstruction` 트리거.
- 자동화 테스트 `RobotSim.Kinematics/PoseError/Jacobian/IK.*` 전부 그대로 통과(수학 레이어 무변경).
