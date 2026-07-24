#!/usr/bin/env python3

from pathlib import Path
import csv
import sys

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree


# =========================================================
# 1. 경로 설정
# 현재 가장 좋았던 voxel 2 mm 결과 사용
# =========================================================
input_path = Path(
    "/home/dg/cobot_ws/data/dbscan/good_multitap_voxel_2p0mm_voxel_2p0mm_std_1p5_eps_0p012.pcd"
)

output_dir = Path(
    "/home/dg/cobot_ws/data/smoothing"
)

output_dir.mkdir(
    parents=True,
    exist_ok=True,
)


# =========================================================
# 2. 평활화 실험값
# =========================================================
K_NEIGHBORS = 20

ALPHA_VALUES = [
    0.20
]

# 한 번만 적용하는 것을 권장
ITERATIONS = 3


# =========================================================
# 3. 파일명 변환
# 0.20 -> 0p20
# =========================================================
def number_to_name(value: float, digits: int = 2) -> str:
    return (
        f"{value:.{digits}f}"
        .replace(".", "p")
    )


# =========================================================
# 4. MLS 유사 국소 평면 평활화
# =========================================================
def smooth_by_local_plane(
    cloud: o3d.geometry.PointCloud,
    k_neighbors: int,
    alpha: float,
    iterations: int = 1,
) -> tuple[o3d.geometry.PointCloud, dict]:
    """
    각 포인트를 주변 포인트의 PCA 국소 평면으로 투영한 뒤,
    원래 위치와 투영 위치를 alpha 비율로 혼합한다.

    alpha:
        0.0 -> 원본 유지
        1.0 -> 국소 평면에 완전히 투영
    """

    if not 0.0 <= alpha <= 1.0:
        raise ValueError(
            "alpha must be between 0.0 and 1.0."
        )

    points = np.asarray(
        cloud.points
    ).astype(
        np.float64,
        copy=True,
    )

    if len(points) < k_neighbors:
        raise ValueError(
            f"Point count {len(points)} is smaller than "
            f"k_neighbors={k_neighbors}."
        )

    original_points = points.copy()

    for iteration in range(iterations):
        print(
            f"[INFO] Smoothing iteration "
            f"{iteration + 1}/{iterations}"
        )

        tree = cKDTree(points)
        new_points = points.copy()

        for index, point in enumerate(points):
            _, neighbor_indices = tree.query(
                point,
                k=k_neighbors,
            )

            neighbors = points[
                np.asarray(
                    neighbor_indices,
                    dtype=np.int64,
                )
            ]

            centroid = np.mean(
                neighbors,
                axis=0,
            )

            centered_neighbors = (
                neighbors - centroid
            )

            covariance = (
                centered_neighbors.T
                @ centered_neighbors
            ) / max(
                len(neighbors) - 1,
                1,
            )

            eigenvalues, eigenvectors = (
                np.linalg.eigh(covariance)
            )

            # 가장 작은 고유값 방향이 국소 평면의 법선
            normal = eigenvectors[:, 0]

            normal_length = np.linalg.norm(
                normal
            )

            if normal_length < 1e-12:
                continue

            normal = normal / normal_length

            signed_distance = np.dot(
                point - centroid,
                normal,
            )

            projected_point = (
                point
                - signed_distance * normal
            )

            new_points[index] = (
                (1.0 - alpha) * point
                + alpha * projected_point
            )

        points = new_points

    displacement = np.linalg.norm(
        points - original_points,
        axis=1,
    )

    smoothed_cloud = (
        o3d.geometry.PointCloud()
    )

    smoothed_cloud.points = (
        o3d.utility.Vector3dVector(
            points
        )
    )

    if cloud.has_colors():
        smoothed_cloud.colors = (
            o3d.utility.Vector3dVector(
                np.asarray(
                    cloud.colors
                ).copy()
            )
        )

    if cloud.has_normals():
        smoothed_cloud.normals = (
            o3d.utility.Vector3dVector(
                np.asarray(
                    cloud.normals
                ).copy()
            )
        )

    metrics = {
        "mean_displacement_m": float(
            np.mean(displacement)
        ),
        "median_displacement_m": float(
            np.median(displacement)
        ),
        "p95_displacement_m": float(
            np.percentile(
                displacement,
                95,
            )
        ),
        "max_displacement_m": float(
            np.max(displacement)
        ),
    }

    return smoothed_cloud, metrics


# =========================================================
# 5. 입력 확인
# =========================================================
if not input_path.exists():
    print(
        f"[ERROR] Input file not found: "
        f"{input_path}"
    )
    sys.exit(1)


# =========================================================
# 6. PCD 읽기
# =========================================================
pcd = o3d.io.read_point_cloud(
    str(input_path)
)

point_count = len(
    pcd.points
)

print("=" * 70)
print(f"[INFO] Input file: {input_path}")
print(f"[INFO] Input points: {point_count}")
print("=" * 70)

if point_count == 0:
    print("[ERROR] Point cloud is empty.")
    sys.exit(1)


# =========================================================
# 7. alpha별 평활화 실험
# =========================================================
summary_rows = []

for alpha in ALPHA_VALUES:
    print()
    print("=" * 70)
    print(
        f"[EXPERIMENT] alpha={alpha:.2f}"
    )
    print(
        f"[PARAM] k_neighbors={K_NEIGHBORS}"
    )
    print(
        f"[PARAM] iterations={ITERATIONS}"
    )
    print("=" * 70)

    try:
        smoothed_cloud, metrics = (
            smooth_by_local_plane(
                cloud=pcd,
                k_neighbors=K_NEIGHBORS,
                alpha=alpha,
                iterations=ITERATIONS,
            )
        )

    except Exception as error:
        print(
            f"[ERROR] Smoothing failed: "
            f"{error}"
        )

        summary_rows.append({
            "alpha": alpha,
            "k_neighbors": K_NEIGHBORS,
            "iterations": ITERATIONS,
            "point_count": point_count,
            "mean_displacement_mm": "",
            "median_displacement_mm": "",
            "p95_displacement_mm": "",
            "max_displacement_mm": "",
            "output_file": "",
            "status": "failed",
        })

        continue

    alpha_name = number_to_name(
        alpha,
        digits=2,
    )

    output_path = (
        output_dir
        / (
            f"{input_path.stem}"
            f"_smooth_k{K_NEIGHBORS}"
            f"_alpha_{alpha_name}"
            f"_iter_{ITERATIONS}"
            f".pcd"
        )
    )

    success = o3d.io.write_point_cloud(
        str(output_path),
        smoothed_cloud,
    )

    if not success:
        print(
            f"[ERROR] Failed to save: "
            f"{output_path}"
        )
        status = "save_failed"
        output_file = ""

    else:
        print(
            f"[INFO] Saved: "
            f"{output_path}"
        )
        status = "success"
        output_file = output_path.name

    print(
        "[INFO] Mean displacement: "
        f"{metrics['mean_displacement_m'] * 1000:.3f} mm"
    )
    print(
        "[INFO] Median displacement: "
        f"{metrics['median_displacement_m'] * 1000:.3f} mm"
    )
    print(
        "[INFO] P95 displacement: "
        f"{metrics['p95_displacement_m'] * 1000:.3f} mm"
    )
    print(
        "[INFO] Max displacement: "
        f"{metrics['max_displacement_m'] * 1000:.3f} mm"
    )

    summary_rows.append({
        "alpha": alpha,
        "k_neighbors": K_NEIGHBORS,
        "iterations": ITERATIONS,
        "point_count": point_count,
        "mean_displacement_mm": (
            metrics["mean_displacement_m"]
            * 1000
        ),
        "median_displacement_mm": (
            metrics["median_displacement_m"]
            * 1000
        ),
        "p95_displacement_mm": (
            metrics["p95_displacement_m"]
            * 1000
        ),
        "max_displacement_mm": (
            metrics["max_displacement_m"]
            * 1000
        ),
        "output_file": output_file,
        "status": status,
    })


# =========================================================
# 8. CSV 저장
# =========================================================
summary_path = (
    output_dir
    / (
        f"{input_path.stem}"
        f"_smoothing_summary.csv"
    )
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
    print(
        f"[INFO] Summary saved: "
        f"{summary_path}"
    )


# =========================================================
# 9. 최종 요약
# =========================================================
print()
print("=" * 85)
print("SMOOTHING TEST SUMMARY")
print("=" * 85)

for row in summary_rows:
    if row["status"] == "success":
        print(
            f"alpha={row['alpha']:.2f} | "
            f"k={row['k_neighbors']} | "
            f"mean={row['mean_displacement_mm']:.3f} mm | "
            f"p95={row['p95_displacement_mm']:.3f} mm | "
            f"max={row['max_displacement_mm']:.3f} mm"
        )
    else:
        print(
            f"alpha={row['alpha']:.2f} | "
            f"status={row['status']}"
        )

print("=" * 85)