#!/usr/bin/env python3

from pathlib import Path
import sys

import open3d as o3d


input_path = Path(
    "/home/dg/cobot_ws/data/smoothing/"
    "good_multitap_smoothing_alpha0p2_iter2.pcd"
)

output_dir = Path(
    "/home/dg/cobot_ws/data/alpha_shape"
)

output_dir.mkdir(
    parents=True,
    exist_ok=True,
)


if not input_path.exists():
    print(f"[ERROR] File not found: {input_path}")
    sys.exit(1)


pcd = o3d.io.read_point_cloud(
    str(input_path)
)

if len(pcd.points) == 0:
    print("[ERROR] Point cloud is empty.")
    sys.exit(1)


alpha_values = [
    0.004,
    0.006,
    0.008,
    0.010,
]


# tetra mesh는 한 번만 생성해서 재사용
tetra_mesh, point_map = (
    o3d.geometry.TetraMesh
    .create_from_point_cloud(pcd)
)


for alpha in alpha_values:
    print("=" * 60)
    print(f"[INFO] Alpha: {alpha:.3f} m")

    mesh = (
        o3d.geometry.TriangleMesh
        .create_from_point_cloud_alpha_shape(
            pcd,
            alpha,
            tetra_mesh,
            point_map,
        )
    )

    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()

    alpha_name = (
        f"{alpha:.3f}"
        .replace(".", "p")
    )

    output_path = (
        output_dir
        / (
            f"{input_path.stem}"
            f"_alpha_{alpha_name}.ply"
        )
    )

    success = o3d.io.write_triangle_mesh(
        str(output_path),
        mesh,
    )

    print(f"[INFO] Vertices: {len(mesh.vertices)}")
    print(f"[INFO] Triangles: {len(mesh.triangles)}")

    if success:
        print(f"[INFO] Saved: {output_path}")
    else:
        print(f"[ERROR] Save failed: {output_path}")