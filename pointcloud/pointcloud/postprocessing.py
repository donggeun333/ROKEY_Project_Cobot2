#!/usr/bin/env python3

from pathlib import Path
import sys

import numpy as np
import open3d as o3d


# =========================================================
# 1. 경로 설정
# =========================================================
script_dir = Path(__file__).resolve().parent

input_path = (
    script_dir
    / "/home/dg/cobot_ws/data/merged_icp/multitap_filled_icp2.pcd"
)

output_path = (
    script_dir
    / "/home/dg/cobot_ws/data/dbscan/multitap_filled_icp2_dbscan2.pcd"
)


# =========================================================
# 2. 입력 파일 확인
# =========================================================
if not input_path.exists():
    print(f"[ERROR] Input file not found: {input_path}")
    sys.exit(1)


# =========================================================
# 3. PCD 읽기
# =========================================================
pcd = o3d.io.read_point_cloud(str(input_path))

original_points = len(pcd.points)

print(f"[INFO] Input file: {input_path}")
print(f"[INFO] Original points: {original_points}")

if original_points == 0:
    print("[ERROR] Point cloud is empty.")
    sys.exit(1)


# =========================================================
# 4. ROI 적용
# 멀티탭 위치에 맞게 조정
# =========================================================
bbox = o3d.geometry.AxisAlignedBoundingBox(
    min_bound=(0.28, 0.01, -0.03),
    max_bound=(0.46, 0.20, 0.1),
)

roi_cloud = pcd.crop(bbox)

roi_points = len(roi_cloud.points)

print(f"[INFO] ROI points: {roi_points}")

if roi_points == 0:
    print("[ERROR] No points found inside ROI.")
    sys.exit(1)


# =========================================================
# 5. Voxel Downsampling
# =========================================================
voxel_size = 0.002  # 2 mm

downsampled_cloud = roi_cloud.voxel_down_sample(
    voxel_size=voxel_size
)

downsampled_points = len(downsampled_cloud.points)

print(f"[INFO] Downsampled points: {downsampled_points}")

if downsampled_points == 0:
    print("[ERROR] No points remain after voxel downsampling.")
    sys.exit(1)


# =========================================================
# 6. Statistical Outlier Removal
# =========================================================
filtered_cloud, _ = downsampled_cloud.remove_statistical_outlier(
    nb_neighbors=30,
    std_ratio=1.5,
)

filtered_points = len(filtered_cloud.points)

print(f"[INFO] After outlier removal: {filtered_points}")

if filtered_points == 0:
    print("[ERROR] No points remain after outlier removal.")
    sys.exit(1)


# =========================================================
# 7. DBSCAN 군집화
# =========================================================
eps = 0.012       # 12 mm
min_points = 20

labels = np.asarray(
    filtered_cloud.cluster_dbscan(
        eps=eps,
        min_points=min_points,
        print_progress=True,
    )
)

if labels.size == 0:
    print("[ERROR] DBSCAN returned no labels.")
    sys.exit(1)


# =========================================================
# 8. 유효 군집 확인
# -1은 DBSCAN noise
# =========================================================
valid_labels = labels[labels >= 0]
noise_count = int(np.sum(labels == -1))

print(f"[INFO] DBSCAN noise points: {noise_count}")

if valid_labels.size == 0:
    print("[ERROR] All points were classified as DBSCAN noise.")
    print("[TIP] Increase eps or decrease min_points.")
    sys.exit(1)


# =========================================================
# 9. 군집별 포인트 수 출력
# =========================================================
cluster_ids, cluster_counts = np.unique(
    valid_labels,
    return_counts=True,
)

print(f"[INFO] Number of clusters: {len(cluster_ids)}")

for cluster_id, count in zip(cluster_ids, cluster_counts):
    print(f"[INFO] Cluster {cluster_id}: {count} points")


# =========================================================
# 10. 가장 큰 군집 선택
# ROI 안에서 가장 큰 군집을 멀티탭으로 가정
# =========================================================
largest_cluster_id = cluster_ids[
    np.argmax(cluster_counts)
]

largest_cluster_indices = np.where(
    labels == largest_cluster_id
)[0]

multitap_cloud = filtered_cloud.select_by_index(
    largest_cluster_indices
)

multitap_points = len(multitap_cloud.points)

print(f"[INFO] Selected cluster: {largest_cluster_id}")
print(f"[INFO] Multitap points: {multitap_points}")

if multitap_points == 0:
    print("[ERROR] Selected cluster is empty.")
    sys.exit(1)


# =========================================================
# 11. 결과 저장
# =========================================================
success = o3d.io.write_point_cloud(
    str(output_path),
    multitap_cloud,
)

if not success:
    print(f"[ERROR] Failed to save: {output_path}")
    sys.exit(1)

print(f"[INFO] Saved: {output_path}")
print(f"[INFO] Saved points: {multitap_points}")