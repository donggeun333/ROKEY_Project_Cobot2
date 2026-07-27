# Pointcloud Package README

이 패키지는 볼트/멀티탭을 여러 시점에서 스캔한 뒤 point cloud를 후처리하고,
정상 기준 PCD와 비교해서 3D 검사 결과를 만드는 역할을 담당한다.

## 역할 요약

- `pipeline_node`
  - PointCloud2 입력을 받아 캡처 저장
  - 여러 캡처를 ICP로 병합
  - ROI crop + outlier 제거 + DBSCAN 수행
  - 최종 `filtered_dbscan_*.pcd` 생성
- `comparison_node`
  - 후처리된 test PCD를 정상 기준 PCD와 voxel 비교
  - 공통/누락/추가 영역 PCD와 `metrics.json` 저장

## 현재 실행 구조

수동 테스트 방식:

1. robot bringup
2. realsense node
3. 필요 시 static TF publisher
4. `pipeline_with_comparison.launch.py`
5. 별도 스캔 실행기
   - 예전 수동 방식: `scan_test.py`
   - 현재 음성 연동 방식: `robot_control/pointcloud_inspector_task.py`

음성 연동 방식에서는 `scan_test.py`를 직접 실행하지 않고,
`robot_control` 패키지의 pointcloud inspection task가 같은 역할을 대신한다.

## 기본 실행 명령

멀티탭 검사:

```bash
ros2 launch pointcloud pipeline_with_comparison.launch.py \
  object_type:=multitap
```

볼트 검사:

```bash
ros2 launch pointcloud pipeline_with_comparison.launch.py \
  object_type:=bolt
```

필요 시 static TF:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.02 \
  --y 0.075 \
  --z 0.04 \
  --roll -1.5708 \
  --pitch -1.5708 \
  --yaw 0.0 \
  --frame-id link_6 \
  --child-frame-id camera_link
```

## object_type

현재 지원:

- `bolt`
- `multitap`

이 값은 pointcloud 노드 내부 ROI, 비교 기준 PCD, 스캔 포즈 선택에 영향을 준다.

## 기준 PCD

기본 기준 PCD는 패키지 resource를 사용한다.

- `resource/good_bolt.pcd`
- `resource/good_multitap.pcd`

`comparison_node.py`는 기본적으로 이 두 파일을 reference로 사용한다.
launch에서 다른 경로를 넘기면 그 값을 우선 사용한다.

## 주요 파일 역할

- `pointcloud/pipeline_node.py`
  - point cloud 캡처, 병합, 후처리, finalize 서비스 제공
- `pointcloud/comparison_node.py`
  - 기준 PCD와의 비교 서비스 제공
- `pointcloud/occupancy_compare.py`
  - voxel occupancy 비교 알고리즘 구현
- `pointcloud/scan_test.py`
  - 수동 movej 기반 스캔 테스트 스크립트
- `launch/pipeline_with_comparison.launch.py`
  - `pipeline_node` + `comparison_node` 실행용 launch
- `resource/good_bolt.pcd`
  - 기본 볼트 기준 PCD
- `resource/good_multitap.pcd`
  - 기본 멀티탭 기준 PCD

## 서비스 구조

`pipeline_node.py`가 제공:

- `/pointcloud_pipeline/reset`
- `/pointcloud_pipeline/capture`
- `/pointcloud_pipeline/finalize`

`comparison_node.py`가 제공:

- `/pointcloud_comparison/compare`

비교 서비스 타입:

- `../od_msg/srv/SrvPointCloudCompare.srv`

request:

- `test_path`

response:

- `success`
- `message`
- `output_dir`
- `result_text`
- `similarity`
- `missing_ratio`
- `added_ratio`

## 출력 파일

pipeline 출력:

- `data/pipeline/captures/capture_*.pcd`
- `data/pipeline/merged/merged_icp_*.pcd`
- `data/pipeline/filtered/filtered_dbscan_*.pcd`

comparison 출력:

- `data/pipeline/comparison/<test_name>_<timestamp>/common_voxels.pcd`
- `data/pipeline/comparison/<test_name>_<timestamp>/missing_voxels.pcd`
- `data/pipeline/comparison/<test_name>_<timestamp>/added_voxels.pcd`
- `data/pipeline/comparison/<test_name>_<timestamp>/union_voxels.pcd`
- `data/pipeline/comparison/<test_name>_<timestamp>/metrics.json`

## 현재 남겨둔 파일과 참고 파일

현재 음성 연동에서 직접 쓰는 핵심 파일:

- `pointcloud/pipeline_node.py`
- `pointcloud/comparison_node.py`
- `pointcloud/occupancy_compare.py`
- `launch/pipeline_with_comparison.launch.py`

참고/실험용 파일:

- `pointcloud/scan_test.py`
- `pointcloud/capture_node.py`
- `pointcloud/merger_node.py`
- `pointcloud/merge_saved_node.py`
- 기타 mesh/smoothing/test 스크립트

## git에 같이 올릴 파일

- `pointcloud/pointcloud/pipeline_node.py`
- `pointcloud/pointcloud/comparison_node.py`
- `pointcloud/pointcloud/occupancy_compare.py`
- `pointcloud/launch/pipeline_with_comparison.launch.py`
- `pointcloud/resource/good_bolt.pcd`
- `pointcloud/resource/good_multitap.pcd`
- `pointcloud/setup.py`
- `pointcloud/package.xml`
- `pointcloud/README.md`
- `od_msg/srv/SrvPointCloudCompare.srv`
- `od_msg/CMakeLists.txt`
- `od_msg/package.xml`

## git에 올리지 않는 파일

- `data/pipeline/captures/`
- `data/pipeline/merged/`
- `data/pipeline/filtered/`
- `data/pipeline/comparison/`
- `build/`
- `install/`
- `log/`
