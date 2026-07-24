#!/usr/bin/env python3

from pathlib import Path
import csv
import sys

import numpy as np
import open3d as o3d


# =========================================================
# 1. 경로 설정
# =========================================================
input_path = Path(
    "/home/dg/cobot_ws/data/pipeline/filtered/good_multitap.pcd"
)

output_dir = Path(
    "/home/dg/cobot_ws/data/dbscan/"
)

output_dir.mkdir(
    parents=True,
    exist_ok=True,
)


# =========================================================
# 2. 고정 파라미터
# =========================================================
NB_NEIGHBORS = 30

EPS = 0.012
MIN_POINTS = 20

# 너무 작은 군집 제거
MIN_CLUSTER_SIZE = 100


# =========================================================
# 3. 실험 조건
# =========================================================
experiments = [
    {
        "name": "voxel_1p5mm",
        "voxel_size": 0.0015,
        "std_ratio": 1.5,
    },
    {
        "name": "voxel_2p0mm",
        "voxel_size": 0.0020,
        "std_ratio": 1.5,
    },
    {
        "name": "voxel_3p0mm",
        "voxel_size": 0.0030,
        "std_ratio": 1.5,
    },
]


# =========================================================
# 4. 입력 파일 확인
# =========================================================
if not input_path.exists():
    print(f"[ERROR] Input file not found: {input_path}")
    sys.exit(1)


# =========================================================
# 5. PCD 읽기
# =========================================================
pcd = o3d.io.read_point_cloud(
    str(input_path)
)

original_points = len(pcd.points)

print("=" * 70)
print(f"[INFO] Input file: {input_path}")
print(f"[INFO] Original points: {original_points}")
print("=" * 70)

if original_points == 0:
    print("[ERROR] Point cloud is empty.")
    sys.exit(1)


# =========================================================
# 6. ROI 적용
# =========================================================
bbox = o3d.geometry.AxisAlignedBoundingBox(
    min_bound=(0.28, 0.01, -0.03),
    max_bound=(0.46, 0.20, 0.10),
)

roi_cloud = pcd.crop(bbox)

roi_points = len(roi_cloud.points)

print(f"[INFO] ROI points: {roi_points}")

if roi_points == 0:
    print("[ERROR] No points found inside ROI.")
    sys.exit(1)


# =========================================================
# 7. 실험 반복
# =========================================================
summary_rows = []

for experiment in experiments:
    name = experiment["name"]
    voxel_size = experiment["voxel_size"]
    std_ratio = experiment["std_ratio"]

    print()
    print("=" * 70)
    print(f"[EXPERIMENT] {name}")
    print(f"[PARAM] voxel_size = {voxel_size:.4f} m")
    print(f"[PARAM] nb_neighbors = {NB_NEIGHBORS}")
    print(f"[PARAM] std_ratio = {std_ratio:.1f}")
    print(f"[PARAM] eps = {EPS:.3f} m")
    print(f"[PARAM] min_points = {MIN_POINTS}")
    print("=" * 70)

    # -----------------------------------------------------
    # 7-1. Voxel Downsampling
    # -----------------------------------------------------
    downsampled_cloud = roi_cloud.voxel_down_sample(
        voxel_size=voxel_size
    )

    downsampled_points = len(
        downsampled_cloud.points
    )

    print(
        f"[INFO] Downsampled points: "
        f"{downsampled_points}"
    )

    if downsampled_points == 0:
        print("[WARNING] No points after downsampling.")

        summary_rows.append({
            "name": name,
            "voxel_size_m": voxel_size,
            "std_ratio": std_ratio,
            "eps_m": EPS,
            "min_points": MIN_POINTS,
            "downsampled_points": 0,
            "sor_points": 0,
            "sor_removed_points": 0,
            "sor_removed_ratio": 0.0,
            "cluster_count": 0,
            "noise_points": 0,
            "noise_ratio": 0.0,
            "selected_cluster_count": 0,
            "selected_points": 0,
            "selected_ratio": 0.0,
            "output_file": "",
            "status": "downsample_failed",
        })

        continue

    # -----------------------------------------------------
    # 7-2. Statistical Outlier Removal
    # -----------------------------------------------------
    filtered_cloud, _ = (
        downsampled_cloud.remove_statistical_outlier(
            nb_neighbors=NB_NEIGHBORS,
            std_ratio=std_ratio,
        )
    )

    filtered_points = len(
        filtered_cloud.points
    )

    sor_removed_points = (
        downsampled_points - filtered_points
    )

    sor_removed_ratio = (
        sor_removed_points / downsampled_points
        if downsampled_points > 0
        else 0.0
    )

    print(
        f"[INFO] After SOR: {filtered_points}"
    )
    print(
        f"[INFO] SOR removed: "
        f"{sor_removed_points} "
        f"({sor_removed_ratio * 100:.2f}%)"
    )

    if filtered_points == 0:
        print("[WARNING] No points remain after SOR.")

        summary_rows.append({
            "name": name,
            "voxel_size_m": voxel_size,
            "std_ratio": std_ratio,
            "eps_m": EPS,
            "min_points": MIN_POINTS,
            "downsampled_points": downsampled_points,
            "sor_points": 0,
            "sor_removed_points": sor_removed_points,
            "sor_removed_ratio": sor_removed_ratio,
            "cluster_count": 0,
            "noise_points": 0,
            "noise_ratio": 0.0,
            "selected_cluster_count": 0,
            "selected_points": 0,
            "selected_ratio": 0.0,
            "output_file": "",
            "status": "sor_failed",
        })

        continue

    # -----------------------------------------------------
    # 7-3. DBSCAN
    # -----------------------------------------------------
    labels = np.asarray(
        filtered_cloud.cluster_dbscan(
            eps=EPS,
            min_points=MIN_POINTS,
            print_progress=False,
        )
    )

    if labels.size == 0:
        print("[WARNING] DBSCAN returned no labels.")

        summary_rows.append({
            "name": name,
            "voxel_size_m": voxel_size,
            "std_ratio": std_ratio,
            "eps_m": EPS,
            "min_points": MIN_POINTS,
            "downsampled_points": downsampled_points,
            "sor_points": filtered_points,
            "sor_removed_points": sor_removed_points,
            "sor_removed_ratio": sor_removed_ratio,
            "cluster_count": 0,
            "noise_points": filtered_points,
            "noise_ratio": 1.0,
            "selected_cluster_count": 0,
            "selected_points": 0,
            "selected_ratio": 0.0,
            "output_file": "",
            "status": "dbscan_failed",
        })

        continue

    # -----------------------------------------------------
    # 7-4. 군집 통계
    # -----------------------------------------------------
    noise_count = int(
        np.sum(labels == -1)
    )

    noise_ratio = (
        noise_count / labels.size
        if labels.size > 0
        else 0.0
    )

    valid_labels = labels[
        labels >= 0
    ]

    print(
        f"[INFO] DBSCAN noise: "
        f"{noise_count} "
        f"({noise_ratio * 100:.2f}%)"
    )

    if valid_labels.size == 0:
        print("[WARNING] All points are noise.")

        summary_rows.append({
            "name": name,
            "voxel_size_m": voxel_size,
            "std_ratio": std_ratio,
            "eps_m": EPS,
            "min_points": MIN_POINTS,
            "downsampled_points": downsampled_points,
            "sor_points": filtered_points,
            "sor_removed_points": sor_removed_points,
            "sor_removed_ratio": sor_removed_ratio,
            "cluster_count": 0,
            "noise_points": noise_count,
            "noise_ratio": noise_ratio,
            "selected_cluster_count": 0,
            "selected_points": 0,
            "selected_ratio": 0.0,
            "output_file": "",
            "status": "all_noise",
        })

        continue

    cluster_ids, cluster_counts = np.unique(
        valid_labels,
        return_counts=True,
    )

    print(
        f"[INFO] Number of clusters: "
        f"{len(cluster_ids)}"
    )

    for cluster_id, count in zip(
        cluster_ids,
        cluster_counts,
    ):
        print(
            f"    Cluster {int(cluster_id)}: "
            f"{int(count)} points"
        )

    # -----------------------------------------------------
    # 7-5. 일정 크기 이상의 군집 유지
    # -----------------------------------------------------
    keep_cluster_ids = cluster_ids[
        cluster_counts >= MIN_CLUSTER_SIZE
    ]

    if keep_cluster_ids.size == 0:
        print(
            "[WARNING] No cluster passed "
            f"MIN_CLUSTER_SIZE={MIN_CLUSTER_SIZE}"
        )

        summary_rows.append({
            "name": name,
            "voxel_size_m": voxel_size,
            "std_ratio": std_ratio,
            "eps_m": EPS,
            "min_points": MIN_POINTS,
            "downsampled_points": downsampled_points,
            "sor_points": filtered_points,
            "sor_removed_points": sor_removed_points,
            "sor_removed_ratio": sor_removed_ratio,
            "cluster_count": len(cluster_ids),
            "noise_points": noise_count,
            "noise_ratio": noise_ratio,
            "selected_cluster_count": 0,
            "selected_points": 0,
            "selected_ratio": 0.0,
            "output_file": "",
            "status": "no_valid_cluster",
        })

        continue

    keep_indices = np.where(
        np.isin(
            labels,
            keep_cluster_ids,
        )
    )[0]

    result_cloud = (
        filtered_cloud.select_by_index(
            keep_indices
        )
    )

    selected_points = len(
        result_cloud.points
    )

    selected_ratio = (
        selected_points / filtered_points
        if filtered_points > 0
        else 0.0
    )

    print(
        f"[INFO] Selected clusters: "
        f"{keep_cluster_ids.tolist()}"
    )
    print(
        f"[INFO] Selected points: "
        f"{selected_points} "
        f"({selected_ratio * 100:.2f}%)"
    )

    # -----------------------------------------------------
    # 7-6. 파일 저장
    # -----------------------------------------------------
    voxel_name = (
        f"{voxel_size * 1000:.1f}"
        .replace(".", "p")
    )

    std_name = (
        f"{std_ratio:.1f}"
        .replace(".", "p")
    )

    output_path = (
        output_dir
        / (
            f"{input_path.stem}"
            f"_{name}"
            f"_voxel_{voxel_name}mm"
            f"_std_{std_name}"
            f"_eps_0p012"
            f".pcd"
        )
    )

    success = o3d.io.write_point_cloud(
        str(output_path),
        result_cloud,
    )

    if success:
        print(f"[INFO] Saved: {output_path}")
        status = "success"
        output_file = output_path.name
    else:
        print(f"[ERROR] Failed to save: {output_path}")
        status = "save_failed"
        output_file = ""

    summary_rows.append({
        "name": name,
        "voxel_size_m": voxel_size,
        "std_ratio": std_ratio,
        "eps_m": EPS,
        "min_points": MIN_POINTS,
        "downsampled_points": downsampled_points,
        "sor_points": filtered_points,
        "sor_removed_points": sor_removed_points,
        "sor_removed_ratio": sor_removed_ratio,
        "cluster_count": len(cluster_ids),
        "noise_points": noise_count,
        "noise_ratio": noise_ratio,
        "selected_cluster_count": len(
            keep_cluster_ids
        ),
        "selected_points": selected_points,
        "selected_ratio": selected_ratio,
        "output_file": output_file,
        "status": status,
    })


# =========================================================
# 8. CSV 저장
# =========================================================
summary_path = (
    output_dir
    / f"{input_path.stem}_parameter_test_summary.csv"
)

if summary_rows:
    with summary_path.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=summary_rows[0].keys(),
        )

        writer.writeheader()
        writer.writerows(summary_rows)

    print()
    print(f"[INFO] Summary saved: {summary_path}")


# =========================================================
# 9. 최종 요약
# =========================================================
print()
print("=" * 90)
print("PARAMETER TEST SUMMARY")
print("=" * 90)

for row in summary_rows:
    print(
        f"{row['name']:12s} | "
        f"voxel={row['voxel_size_m'] * 1000:.1f} mm | "
        f"std={row['std_ratio']:.1f} | "
        f"SOR removed={row['sor_removed_ratio'] * 100:.2f}% | "
        f"noise={row['noise_ratio'] * 100:.2f}% | "
        f"selected={row['selected_points']} | "
        f"status={row['status']}"
    )

print("=" * 90)