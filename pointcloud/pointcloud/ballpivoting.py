#!/usr/bin/env python3

from pathlib import Path
import sys

import numpy as np
import open3d as o3d


# =========================================================
# 1. 경로
# =========================================================
input_path = Path(
    "/home/dg/cobot_ws/data/smoothing/"
    "good_multitap_smoothing_alpha0p2_iter2.pcd"
)

output_dir = Path(
    "/home/dg/cobot_ws/data/mesh_smoothing"
)

output_dir.mkdir(
    parents=True,
    exist_ok=True,
)


# =========================================================
# 2. 파라미터
# =========================================================
NORMAL_RADIUS = 0.010
NORMAL_MAX_NN = 40

# 현재 voxel 2 mm 기준
BALL_RADII = [
    0.0025,
    0.0035,
    # 0.005
]

TAUBIN_ITERATIONS = 10

# mesh를 다시 PCD로 변환할 때 포인트 수
SAMPLE_POINT_COUNT = 20000


# =========================================================
# 3. 입력 확인
# =========================================================
if not input_path.exists():
    print(f"[ERROR] File not found: {input_path}")
    sys.exit(1)


# =========================================================
# 4. PCD 읽기
# =========================================================
pcd = o3d.io.read_point_cloud(
    str(input_path)
)

point_count = len(pcd.points)

print(f"[INFO] Input points: {point_count}")

if point_count == 0:
    print("[ERROR] Empty point cloud.")
    sys.exit(1)


# =========================================================
# 5. 노멀 추정
# =========================================================
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=NORMAL_RADIUS,
        max_nn=NORMAL_MAX_NN,
    )
)

# 인접 포인트 간 노멀 방향을 최대한 일관되게 정리
pcd.orient_normals_consistent_tangent_plane(
    k=30
)

pcd.normalize_normals()

print("[INFO] Normal estimation completed.")


# =========================================================
# 6. Ball Pivoting mesh 생성
# =========================================================
radii = o3d.utility.DoubleVector(
    BALL_RADII
)

mesh = (
    o3d.geometry.TriangleMesh
    .create_from_point_cloud_ball_pivoting(
        pcd,
        radii,
    )
)

print(
    f"[INFO] Mesh vertices: "
    f"{len(mesh.vertices)}"
)
print(
    f"[INFO] Mesh triangles: "
    f"{len(mesh.triangles)}"
)

if len(mesh.triangles) == 0:
    print("[ERROR] Mesh reconstruction failed.")
    print("[TIP] Increase BALL_RADII.")
    sys.exit(1)


# =========================================================
# 7. Mesh 정리
# =========================================================
mesh.remove_duplicated_vertices()
mesh.remove_duplicated_triangles()
mesh.remove_degenerate_triangles()
mesh.remove_non_manifold_edges()
mesh.remove_unreferenced_vertices()

mesh.compute_vertex_normals()


# =========================================================
# 8. 원본 mesh 저장
# =========================================================
raw_mesh_path = (
    output_dir
    / f"{input_path.stem}_bpa_raw.ply"
)

success = o3d.io.write_triangle_mesh(
    str(raw_mesh_path),
    mesh,
)

if not success:
    print(
        f"[ERROR] Failed to save raw mesh: "
        f"{raw_mesh_path}"
    )
    sys.exit(1)

print(f"[INFO] Raw mesh saved: {raw_mesh_path}")


# =========================================================
# 9. Taubin smoothing
# =========================================================
smoothed_mesh = mesh.filter_smooth_taubin(
    number_of_iterations=TAUBIN_ITERATIONS,
)

smoothed_mesh.compute_vertex_normals()


# =========================================================
# 10. Smoothed mesh 저장
# =========================================================
smoothed_mesh_path = (
    output_dir
    / (
        f"{input_path.stem}"
        f"_bpa_taubin_iter{TAUBIN_ITERATIONS}.ply"
    )
)

success = o3d.io.write_triangle_mesh(
    str(smoothed_mesh_path),
    smoothed_mesh,
)

if not success:
    print(
        f"[ERROR] Failed to save smoothed mesh: "
        f"{smoothed_mesh_path}"
    )
    sys.exit(1)

print(
    f"[INFO] Smoothed mesh saved: "
    f"{smoothed_mesh_path}"
)


# =========================================================
# 11. Mesh를 다시 PCD로 샘플링
# =========================================================
sampled_pcd = (
    smoothed_mesh.sample_points_poisson_disk(
        number_of_points=SAMPLE_POINT_COUNT,
        init_factor=5,
    )
)

sampled_pcd_path = (
    output_dir
    / (
        f"{input_path.stem}"
        f"_bpa_taubin_iter{TAUBIN_ITERATIONS}"
        f"_sampled_{SAMPLE_POINT_COUNT}.pcd"
    )
)

success = o3d.io.write_point_cloud(
    str(sampled_pcd_path),
    sampled_pcd,
)

if not success:
    print(
        f"[ERROR] Failed to save sampled PCD: "
        f"{sampled_pcd_path}"
    )
    sys.exit(1)

print(
    f"[INFO] Sampled PCD saved: "
    f"{sampled_pcd_path}"
)


# =========================================================
# 12. 결과 요약
# =========================================================
print()
print("=" * 70)
print("MESH SMOOTHING SUMMARY")
print("=" * 70)
print(f"Input points       : {point_count}")
print(f"Mesh vertices      : {len(mesh.vertices)}")
print(f"Mesh triangles     : {len(mesh.triangles)}")
print(f"Taubin iterations  : {TAUBIN_ITERATIONS}")
print(f"Sampled PCD points : {len(sampled_pcd.points)}")
print("=" * 70)