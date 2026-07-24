# PointCloud Pipeline README

이 패키지는 멀티탭/볼트 스캔 결과를 point cloud로 저장하고, 후처리한 뒤, 정상 기준 PCD와 비교하는 작업을 담당한다.

## 전체 흐름

1. `pipeline_node.py`
   - 카메라 PointCloud2를 받아 캡처
   - 여러 캡처를 ICP로 병합
   - ROI crop 수행
   - DBSCAN으로 대상 cluster만 추출
   - `filtered_dbscan_*.pcd` 저장

2. `comparison_node.py`
   - `pipeline_node`가 만든 `filtered_dbscan_*.pcd` 경로를 전달받음
   - 정상 기준 PCD와 voxel 단위로 비교
   - 공통/누락/추가 영역 PCD와 `metrics.json` 저장

## 핵심 파일

- `pointcloud/pipeline_node.py`
  - 스캔, ICP 병합, ROI crop, DBSCAN 후처리 담당
- `pointcloud/comparison_node.py`
  - 후처리 결과 PCD와 기준 PCD 비교 담당
- `pointcloud/occupancy_compare.py`
  - voxel 비교 알고리즘 구현
- `launch/pipeline_with_comparison.launch.py`
  - `pipeline_node` + `comparison_node` 실행용 launch
- `setup.py`
  - `pipeline_node`, `comparison_node` console script 등록
- `package.xml`
  - `pointcloud` 패키지 의존성 정의

## 커스텀 서비스

비교 단계로 넘어갈 때는 ROS 기본 `Trigger` 대신 커스텀 서비스 사용.

- 파일: `../od_msg/srv/SrvPointCloudCompare.srv`
- 역할:
  - request: `test_path`
  - response: `success`, `message`, `output_dir`, `result_text`, `similarity`, `missing_ratio`, `added_ratio`

이 서비스가 필요한 이유:

- `pipeline_node`가 방금 만든 `filtered_dbscan_*.pcd` 경로를
- `comparison_node`에 정확히 전달하기 위해서

즉, "최신 파일 추측"이 아니라 "이 파일을 비교하라"를 직접 넘긴다.

## 관련 인터페이스 패키지

커스텀 서비스는 `od_msg` 패키지에 정의되어 있다.

- `../od_msg/srv/SrvPointCloudCompare.srv`
- `../od_msg/CMakeLists.txt`
- `../od_msg/package.xml`

git 업로드 시에는 `pointcloud` 패키지 변경분과 함께 `od_msg`의 위 3개 파일도 같이 포함해야 한다.

## 출력 파일

`pipeline_node.py` 출력:

- `data/pipeline/captures/capture_*.pcd`
- `data/pipeline/merged/merged_icp_*.pcd`
- `data/pipeline/filtered/filtered_dbscan_*.pcd`

`comparison_node.py` 출력:

- `data/pipeline/comparison/<test_name>_<timestamp>/common_voxels.pcd`
- `data/pipeline/comparison/<test_name>_<timestamp>/missing_voxels.pcd`
- `data/pipeline/comparison/<test_name>_<timestamp>/added_voxels.pcd`
- `data/pipeline/comparison/<test_name>_<timestamp>/union_voxels.pcd`
- `data/pipeline/comparison/<test_name>_<timestamp>/metrics.json`

## 실행에 필요한 구성

- PointCloud2 입력 토픽
- 필요 시 TF (`camera frame -> save_frame`)
- 정상 기준 PCD 경로
  - `multitap_reference_path`
  - 필요 시 `bolt_reference_path`

## 최소 실행 노드

- `pointcloud_pipeline`
- `pointcloud_comparison`

필요하면 여기에 `static_transform_publisher`를 launch에 같이 포함한다.

## git에 올릴 때 보통 포함할 파일

- `pointcloud/pointcloud/pipeline_node.py`
- `pointcloud/pointcloud/comparison_node.py`
- `pointcloud/pointcloud/occupancy_compare.py`
- `pointcloud/launch/pipeline_with_comparison.launch.py`
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
