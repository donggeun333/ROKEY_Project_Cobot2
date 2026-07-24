#!/usr/bin/env python3

from pathlib import Path
import sys

import numpy as np
import open3d as o3d


# =========================================================
# 1. 경로 설정
# =========================================================
input_path = Path(
    "/home/dg/cobot_ws/data/smoothing/"
    "good_multitap_smoothing_alpha0p2_iter2.pcd"
)

output_dir = Path(
    "/home/dg/cobot_ws/data/poisson_vertex_test"
)

output_dir.mkdir(
    parents=True,
    exist_ok=True,
)


# =========================================================
# 2. 파라미터
# =========================================================
NORMAL_RADIUS = 0.006
NORMAL_MAX_NN = 30
NORMAL_ORIENT_K = 30

POISSON_DEPTH = 10
POISSON_SCALE = 1.1
LINEAR_FIT = False

# density가 낮은 하위 2% vertex 제거
DENSITY_QUANTILE = 0.05


# =========================================================
# 3. 입력 확인
# =========================================================
if not input_path.exists():
    print(f"[ERROR] File not found: {input_path}")
    sys.exit(1)


# =========================================================
# 4. PCD 읽기
# =========================================================
pcd = o3d.io.read_point_cloud(str(input_path))

point_count = len(pcd.points)

print("=" * 70)
print(f"[INFO] Input file: {input_path}")
print(f"[INFO] Input points: {point_count}")
print("=" * 70)

if point_count == 0:
    print("[ERROR] Point cloud is empty.")
    sys.exit(1)


# =========================================================
# 5. 노멀 추정 및 방향 정렬
# =========================================================
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=NORMAL_RADIUS,
        max_nn=NORMAL_MAX_NN,
    )
)

pcd.orient_normals_consistent_tangent_plane(
    k=NORMAL_ORIENT_K
)

pcd.normalize_normals()

print("[INFO] Normal estimation completed.")


# =========================================================
# 6. Poisson Surface Reconstruction
# =========================================================
mesh_raw, densities = (
    o3d.geometry.TriangleMesh
    .create_from_point_cloud_poisson(
        pcd,
        depth=POISSON_DEPTH,
        width=0,
        scale=POISSON_SCALE,
        linear_fit=LINEAR_FIT,
    )
)

densities = np.asarray(densities)

raw_vertex_count = len(mesh_raw.vertices)
raw_triangle_count = len(mesh_raw.triangles)

print(f"[INFO] Raw vertices: {raw_vertex_count}")
print(f"[INFO] Raw triangles: {raw_triangle_count}")

if raw_vertex_count == 0 or raw_triangle_count == 0:
    print("[ERROR] Poisson reconstruction failed.")
    sys.exit(1)


# =========================================================
# 7. 순수 Poisson Mesh 저장
# =========================================================
mesh_raw.compute_vertex_normals()

scale_name = str(POISSON_SCALE).replace(".", "p")

raw_output_path = (
    output_dir
    / (
        f"{input_path.stem}"
        f"_poisson_raw"
        f"_depth{POISSON_DEPTH}"
        f"_scale{scale_name}"
        f".ply"
    )
)

raw_success = o3d.io.write_triangle_mesh(
    str(raw_output_path),
    mesh_raw,
    write_ascii=False,
)

if not raw_success:
    print(
        f"[ERROR] Failed to save raw mesh: "
        f"{raw_output_path}"
    )
    sys.exit(1)

print(f"[INFO] Raw mesh saved: {raw_output_path}")


# =========================================================
# 8. Density 통계
# =========================================================
density_min = float(np.min(densities))
density_mean = float(np.mean(densities))
density_median = float(np.median(densities))
density_max = float(np.max(densities))

density_threshold = float(
    np.quantile(
        densities,
        DENSITY_QUANTILE,
    )
)

print()
print("[INFO] Density statistics")
print(f"    Minimum   : {density_min:.6f}")
print(f"    Mean      : {density_mean:.6f}")
print(f"    Median    : {density_median:.6f}")
print(f"    Maximum   : {density_max:.6f}")
print(f"    Threshold : {density_threshold:.6f}")


# =========================================================
# 9. 저밀도 Vertex 제거
# mesh_raw을 보존하기 위해 복사본 생성
# =========================================================
mesh_filtered = o3d.geometry.TriangleMesh(mesh_raw)

remove_mask = (
    densities < density_threshold
)

removed_vertex_count = int(
    np.sum(remove_mask)
)

print(
    f"[INFO] Vertices selected for removal: "
    f"{removed_vertex_count} "
    f"({removed_vertex_count / raw_vertex_count * 100:.2f}%)"
)

mesh_filtered.remove_vertices_by_mask(
    remove_mask
)


# =========================================================
# 10. 제거 이후 Mesh 정리
# =========================================================
mesh_filtered.remove_duplicated_vertices()
mesh_filtered.remove_duplicated_triangles()
mesh_filtered.remove_degenerate_triangles()
mesh_filtered.remove_unreferenced_vertices()

mesh_filtered.compute_vertex_normals()

filtered_vertex_count = len(
    mesh_filtered.vertices
)

filtered_triangle_count = len(
    mesh_filtered.triangles
)

print(
    f"[INFO] Filtered vertices: "
    f"{filtered_vertex_count}"
)
print(
    f"[INFO] Filtered triangles: "
    f"{filtered_triangle_count}"
)

if filtered_vertex_count == 0 or filtered_triangle_count == 0:
    print(
        "[ERROR] Filtered mesh is empty. "
        "Decrease DENSITY_QUANTILE."
    )
    sys.exit(1)


# =========================================================
# 11. Vertex 제거 Mesh 저장
# =========================================================
quantile_name = (
    f"{DENSITY_QUANTILE:.2f}"
    .replace(".", "p")
)

filtered_output_path = (
    output_dir
    / (
        f"{input_path.stem}"
        f"_poisson_vertex_filtered"
        f"_depth{POISSON_DEPTH}"
        f"_density_q{quantile_name}"
        f".ply"
    )
)

filtered_success = o3d.io.write_triangle_mesh(
    str(filtered_output_path),
    mesh_filtered,
    write_ascii=False,
)

if not filtered_success:
    print(
        f"[ERROR] Failed to save filtered mesh: "
        f"{filtered_output_path}"
    )
    sys.exit(1)

print(
    f"[INFO] Filtered mesh saved: "
    f"{filtered_output_path}"
)


# =========================================================
# 12. 결과 요약
# =========================================================
print()
print("=" * 70)
print("POISSON VERTEX FILTER TEST SUMMARY")
print("=" * 70)
print(f"Input points          : {point_count}")
print(f"Poisson depth         : {POISSON_DEPTH}")
print(f"Poisson scale         : {POISSON_SCALE}")
print(f"Density quantile      : {DENSITY_QUANTILE}")
print(f"Raw vertices          : {raw_vertex_count}")
print(f"Raw triangles         : {raw_triangle_count}")
print(f"Removed vertices      : {removed_vertex_count}")
print(f"Filtered vertices     : {filtered_vertex_count}")
print(f"Filtered triangles    : {filtered_triangle_count}")
print(f"Raw mesh              : {raw_output_path}")
print(f"Vertex-filtered mesh  : {filtered_output_path}")
print("=" * 70)