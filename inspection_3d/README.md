# Pointcloud Package README

이 패키지는 볼트와 멀티탭을 여러 시점에서 스캔한 뒤 후처리하고,
기준 PCD와 비교해서 3D 검사 결과를 만든다.

현재 프로젝트에서는 HMI/음성이 직접 이 패키지를 호출하지 않는다.
`robot_control/pointcloud_inspector_task.py`가 reset/capture/finalize/compare
서비스를 순서대로 호출하는 "검사 모션 실행기" 역할을 맡는다.

## 현재 역할

- `pipeline_node`
  - PointCloud2 캡처
  - 캡처할 때마다 누적 ICP 병합
  - ROI crop, outlier 제거, DBSCAN
  - 최종 `filtered_dbscan_*.pcd` 생성
- `comparison_node`
  - 기준 PCD와 voxel 비교
  - 결과 PCD와 `metrics.json` 저장

## 현재 실행 구조

1. `pipeline_with_comparison.launch.py`로 `pipeline_node`와 `comparison_node`를 실행한다.
2. 스캔 실행기는 캡처를 여러 번 요청한다.
3. 각 capture 요청 시 새 점군을 현재 누적 점군에 바로 ICP 병합한다.
4. `finalize`로 누적 점군의 후처리 결과 PCD를 만든다.
5. `compare`로 기준 PCD와 비교한다.

현재 음성 연동에서는 `robot_control/pointcloud_inspector_task.py`가
스캔 실행기 역할을 담당한다.

## 실행 명령

멀티탭 검사:

```bash
ros2 launch inspection_3d pipeline_with_comparison.launch.py \
  object_type:=multitap
```

볼트 검사:

```bash
ros2 launch inspection_3d pipeline_with_comparison.launch.py \
  object_type:=bolt
```

## object_type

현재 지원:

- `bolt`
- `multitap`

이 값에 따라 ROI와 기준 PCD가 바뀐다.

## 기준 PCD

기본 기준 PCD:

- `resource/good_bolt.pcd`
- `resource/good_multitap.pcd`

## 핵심 파일

- `inspection_3d/inspection_3d/pipeline_node.py`
  - 캡처, 병합, 후처리, finalize 서비스
- `inspection_3d/inspection_3d/comparison_node.py`
  - 비교 서비스
- `inspection_3d/inspection_3d/occupancy_compare.py`
  - 실제 voxel 비교 알고리즘
- `launch/pipeline_with_comparison.launch.py`
  - 실행 launch

## 서비스

`pipeline_node.py`:

- `/pointcloud_pipeline/reset`
- `/pointcloud_pipeline/capture`
- `/pointcloud_pipeline/finalize`

`comparison_node.py`:

- `/pointcloud_comparison/compare`

비교 서비스 타입:

- `od_msg/srv/SrvPointCloudCompare.srv`

## 주요 출력

- `src/cobot2_ws/operator_ui/pointclouds/<bolt|outlet>/captures/filtered_dbscan_*.pcd`
- `data/pipeline/comparison/.../metrics.json`

현재 저장 정책:

- `capture_*.pcd` 저장 안 함
- `merged_icp_*.pcd` 저장 안 함
- `filtered_dbscan_*.pcd`는 HMI가 읽는 captures 폴더에 저장
- comparison 결과 저장

## git에 같이 올릴 파일

- `inspection_3d/inspection_3d/pipeline_node.py`
- `inspection_3d/inspection_3d/comparison_node.py`
- `inspection_3d/inspection_3d/occupancy_compare.py`
- `inspection_3d/launch/pipeline_with_comparison.launch.py`
- `inspection_3d/resource/good_bolt.pcd`
- `inspection_3d/resource/good_multitap.pcd`
- `inspection_3d/setup.py`
- `inspection_3d/package.xml`
- `inspection_3d/README.md`
- `od_msg/srv/SrvPointCloudCompare.srv`

보통 올리지 않는 파일:

- `data/pipeline/captures/`
- `data/pipeline/merged/`
- `data/pipeline/comparison/`
- `build/`
- `install/`
- `log/`
