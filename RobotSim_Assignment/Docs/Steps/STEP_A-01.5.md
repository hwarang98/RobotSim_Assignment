# Step A-1.5: 시각화 레이어 (SkeletalMesh 본 동기화 + 링크별 StaticMesh)

## 목표

Step A-1의 FK 관절 계층 위에 **순수 비주얼 레이어**를 추가한다. 핵심은 6축 FK 수학 모델과 실제 로봇 SkeletalMesh의 본 체인을 동기화하는 것으로, 이후 IK 단계에서 수학 모델과 화면 시각화가 일치하는지 실제 로봇 외형으로 확인할 수 있게 한다. 수학 모델은 유지하고 SkeletalMesh는 결과를 따라가기만 한다.

## 포함 범위

- SkeletalMesh 시각화 (`SkeletalVisualComponent`, PoseableMesh): FK 관절 회전각을 J0~J5 대응 본에 델타 회전 리타겟으로 동기화
- Details에서 SkeletalMesh 에셋 할당 + J0~J5 본 이름 매핑 (`SkeletalMeshAsset`, `JointBoneNames`)
- 링크별 StaticMesh 할당 슬롯 7개 (`LinkVisuals`: Base + Link1~6) — 링크 분리 메시/디버그용
- 표시 토글 (`bShowSkeletalMesh`, `bShowStaticMeshes`, `bShowDebugLinks`)
- Details 변경 즉시 반영 (`OnConstruction` / `PostEditChangeProperty`)
- 본 매핑 실패 Warning 로그

## 제외 범위

- FK 수학 모델, joint axis, link offset, joint limit 변경 (일절 없음)
- IK / Jacobian / DLS / nullspace
- Pick & Place, Gripper
- AnimBP/리타게팅 (본 트랜스폼을 코드로 직접 쓰므로 불필요)

## 핵심 결정

- **수학 모델이 source of truth**: `SyncSkeletalPoseToMath()`는 `FSerial6DoFModel::ComputeJointWorldTransform()` 결과를 본에 **복사만** 한다. 본 트랜스폼이 FK 계산에 입력되는 경로는 존재하지 않으므로 EE pose/FK 결과는 구조적으로 불변이다. 기존 `CheckVisualMatchesMath()` 런타임 검증 유지.
- **PoseableMeshComponent 채택**: USkeletalMeshComponent는 AnimInstance가 포즈를 소유하므로 코드에서 본을 직접 쓰려면 AnimBP/커스텀 AnimNode가 필요하다. UPoseableMeshComponent는 `SetBoneTransformByName()`으로 본을 직접 제어할 수 있어 "수학 → 본 단방향 동기화"에 정확히 맞는다.
- **델타 회전 리타겟 동기화**: 처음에는 각 관절의 수학 월드 변환(위치+회전)을 본에 통째로 강제하는 방식을 썼으나, 메시 고유의 링크 길이/본 축이 수학 모델(105/120cm 체계)과 달라 스키닝이 찌그러지는 문제가 확인됐다. 최종 방식은 **관절 회전각 Q[i]만 본의 바인드 로컬 회전 위에 델타로 얹는 리타겟**이다: 관절 축(수학 Q=0에서 컴포넌트 공간 = `JointAxes[i]`)을 부모 본의 바인드 프레임으로 옮겨 `FQuat(축, Q)`를 합성하고, 본의 바인드 평행이동(메시 고유 링크 길이)은 유지한다. 메시가 찌그러지지 않는 대신 본 위치는 메시 비율을 따르므로 디버그 링크와 위치가 어긋나는 것은 **의도된 동작**이며, 수치 검증은 계속 디버그 링크와 `CheckVisualMatchesMath()`가 담당한다. 전제: 바인드 포즈 = 수학 Q=0 자세, 본 체인 순서 = J0→J5.
- **매핑 실패 시 안전한 강등**: 본 이름이 비었거나 에셋에 없으면 Warning을 남기고 SkeletalMesh를 숨긴 채 디버그 링크만 표시한다. 반쪽 동기화된 메시는 수학 검증에 방해가 되기 때문이다.
- **디버그 링크 유지**: 관절 구조의 수학 검증은 기존 기본 도형 디버그 링크와 디버그 좌표축으로 진행한다. SkeletalMesh는 어디까지나 외형 확인용 레이어다.
- **링크별 StaticMesh 슬롯 병행 유지**: 링크 분리된 메시 에셋이 생기면 `LinkVisuals`에 할당해 사용할 수 있다 (관절 프레임 자식 부착, FK 불변, 미할당 슬롯은 디버그 도형 유지).
- **충돌 배제**: 모든 비주얼 컴포넌트는 `NoCollision` — 이후 단계의 물리/트레이스에 간섭하지 않는다.

## 구현 요약

| 항목 | 내용 |
|---|---|
| `SkeletalMeshAsset` / `JointBoneNames[6]` / `bShowSkeletalMesh` | Details 노출 프로퍼티 (Robot\|Visual) |
| `SkeletalVisualComponent` | `UPoseableMeshComponent`, Root 직속, NoCollision |
| `ApplySkeletalMeshVisual()` | 에셋 적용 + 본 매핑 유효성 검증(실패 시 Warning + 숨김) + 가시성 갱신 |
| `SyncSkeletalPoseToMath()` | 관절 회전각을 본 바인드 로컬 회전에 델타로 합성(바인드 평행이동 유지). `ApplyJointState()`에서 매 적용 시 호출 |
| `FLinkVisualConfig` + `LinkVisuals` 7슬롯 | 링크별 StaticMesh 슬롯 (Base + Link1~6, `EditFixedSize`) |
| 반영 시점 | `OnConstruction` + `PostEditChangeProperty` (`GetMemberPropertyName()` 기준) |

## 검증

- Automation Test 5종(`RobotSim.Kinematics.*`)은 액터를 스폰하지 않는 순수 수학 검증이므로 이번 변경과 무관하며 그대로 통과해야 한다 — all success 확인.
- 에디터/PIE: SkeletalMesh 에셋 + 본 이름 매핑 → 메시가 찌그러짐 없이 표시, J0~J5 각도 변경 시 대응 본이 디버그 링크와 같은 방향으로 회전(위치는 메시 비율만큼 차이 가능), 메시 할당 전후 EE pose 로그 동일, 매핑 오류 시 Warning 출력, 토글 동작 확인.

## 다음 단계

Step A-2: Jacobian 기반 IK (DLS, nullspace joint limit avoidance). 수학 모델(`FSerial6DoFModel`) 위에서 구현하며 이번 비주얼 레이어는 변경하지 않는다.
