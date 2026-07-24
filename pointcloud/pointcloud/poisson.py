#!/usr/bin/env python3

from pathlib import Path
import sys

import open3d as o3d


# =========================================================
# 1. 경로 설정
# =========================================================
input_path = Path(
    "/home/dg/cobot_ws/data/smoothing/"
    "good_multitap_smoothing_alpha0p2_iter2.pcd"
)

output_dir = Path(
    "/home/dg/cobot_ws/data/poisson_raw"
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
NORMAL_ORIENT_K = 30

POISSON_DEPTH = 9
POISSON_SCALE = 1.1
LINEAR_FIT = False


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

print("=" * 70)
print(f"[INFO] Input file: {input_path}")
print(f"[INFO] Input points: {point_count}")
print("=" * 70)

if point_count == 0:
    print("[ERROR] Point cloud is empty.")
    sys.exit(1)


# =========================================================
# 5. 노멀 추정
# Poisson reconstruction에는 방향이 있는 노멀이 필수
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
# 6. 순수 Poisson Surface Reconstruction
# densities는 반환받지만 이번 비교에서는 사용하지 않음
# =========================================================
mesh, densities = (
    o3d.geometry.TriangleMesh
    .create_from_point_cloud_poisson(
        pcd,
        depth=POISSON_DEPTH,
        width=0,
        scale=POISSON_SCALE,
        linear_fit=LINEAR_FIT,
    )
)

vertex_count = len(mesh.vertices)
triangle_count = len(mesh.triangles)

print(f"[INFO] Mesh vertices: {vertex_count}")
print(f"[INFO] Mesh triangles: {triangle_count}")

if vertex_count == 0 or triangle_count == 0:
    print("[ERROR] Poisson reconstruction failed.")
    sys.exit(1)


# =========================================================
# 7. 노멀 계산
# 형상 변경이나 vertex 제거는 하지 않음
# =========================================================
mesh.compute_vertex_normals()


# =========================================================
# 8. 순수 Poisson mesh 저장
# =========================================================
output_path = (
    output_dir
    / (
        f"{input_path.stem}"
        f"_poisson_raw"
        f"_depth{POISSON_DEPTH}"
        f"_scale{str(POISSON_SCALE).replace('.', 'p')}"
        f".ply"
    )
)

success = o3d.io.write_triangle_mesh(
    str(output_path),
    mesh,
    write_ascii=False,
)

if not success:
    print(f"[ERROR] Failed to save mesh: {output_path}")
    sys.exit(1)


# =========================================================
# 9. 결과 요약
# =========================================================
print()
print("=" * 70)
print("RAW POISSON RECONSTRUCTION SUMMARY")
print("=" * 70)
print(f"Input points      : {point_count}")
print(f"Normal radius     : {NORMAL_RADIUS:.3f} m")
print(f"Normal max NN     : {NORMAL_MAX_NN}")
print(f"Normal orient K   : {NORMAL_ORIENT_K}")
print(f"Poisson depth     : {POISSON_DEPTH}")
print(f"Poisson scale     : {POISSON_SCALE}")
print(f"Linear fit        : {LINEAR_FIT}")
print(f"Mesh vertices     : {vertex_count}")
print(f"Mesh triangles    : {triangle_count}")
print(f"Saved mesh        : {output_path}")
print("=" * 70)