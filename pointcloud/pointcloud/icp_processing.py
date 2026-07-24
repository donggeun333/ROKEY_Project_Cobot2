#!/usr/bin/env python3

from datetime import datetime
from pathlib import Path
import copy
import sys

import numpy as np
import open3d as o3d


# =========================================================
# 1. 경로 및 파라미터 설정
# =========================================================
INPUT_DIR = Path("/home/dg/cobot_ws/data/captures")
OUTPUT_DIR = Path("/home/dg/cobot_ws/data/merged_icp")

timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
OUTPUT_PATH = OUTPUT_DIR / f"merged_icp_{timestamp}.pcd"

# ICP 전처리 파라미터
VOXEL_SIZE = 0.002                  # 2 mm
NORMAL_RADIUS = VOXEL_SIZE * 4.0    # 8 mm

# ICP 대응점 거리
MAX_CORRESPONDENCE_COARSE = 0.03    # 30 mm
MAX_CORRESPONDENCE_FINE = 0.01      # 10 mm

COARSE_ITERATIONS = 60
FINE_ITERATIONS = 100

# 정합 결과 허용 기준
MIN_FITNESS = 0.4
MAX_RMSE = 0.005


# =========================================================
# 2. 포인트클라우드 읽기
# =========================================================
def load_point_cloud(path: Path) -> o3d.geometry.PointCloud:
    if not path.exists():
        raise FileNotFoundError(f"PCD file not found: {path}")

    pcd = o3d.io.read_point_cloud(str(path))

    if len(pcd.points) == 0:
        raise RuntimeError(f"Empty point cloud: {path}")

    return pcd


# =========================================================
# 3. ICP 계산용 전처리
# =========================================================
def preprocess_point_cloud(
    pcd: o3d.geometry.PointCloud,
) -> o3d.geometry.PointCloud:

    processed = pcd.voxel_down_sample(
        voxel_size=VOXEL_SIZE
    )

    if len(processed.points) == 0:
        return processed

    processed, _ = processed.remove_statistical_outlier(
        nb_neighbors=30,
        std_ratio=1.5,
    )

    if len(processed.points) == 0:
        return processed

    processed.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=NORMAL_RADIUS,
            max_nn=50,
        )
    )

    return processed


# =========================================================
# 4. Coarse → Fine ICP
# =========================================================
def run_icp(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    initial_transform: np.ndarray,
) -> o3d.pipelines.registration.RegistrationResult:

    coarse_result = (
        o3d.pipelines.registration.registration_icp(
            source=source,
            target=target,
            max_correspondence_distance=MAX_CORRESPONDENCE_COARSE,
            init=initial_transform,
            estimation_method=(
                o3d.pipelines.registration
                .TransformationEstimationPointToPlane()
            ),
            criteria=(
                o3d.pipelines.registration
                .ICPConvergenceCriteria(
                    relative_fitness=1e-6,
                    relative_rmse=1e-6,
                    max_iteration=COARSE_ITERATIONS,
                )
            ),
        )
    )

    fine_result = (
        o3d.pipelines.registration.registration_icp(
            source=source,
            target=target,
            max_correspondence_distance=MAX_CORRESPONDENCE_FINE,
            init=coarse_result.transformation,
            estimation_method=(
                o3d.pipelines.registration
                .TransformationEstimationPointToPlane()
            ),
            criteria=(
                o3d.pipelines.registration
                .ICPConvergenceCriteria(
                    relative_fitness=1e-7,
                    relative_rmse=1e-7,
                    max_iteration=FINE_ITERATIONS,
                )
            ),
        )
    )

    return fine_result


# =========================================================
# 5. 정합된 포인트클라우드 병합
# =========================================================
def merge_clouds(
    target_full: o3d.geometry.PointCloud,
    source_full: o3d.geometry.PointCloud,
    transformation: np.ndarray,
) -> o3d.geometry.PointCloud:

    aligned_source = copy.deepcopy(source_full)
    aligned_source.transform(transformation)

    merged = target_full + aligned_source

    # 중복 포인트 감소
    merged = merged.voxel_down_sample(
        voxel_size=VOXEL_SIZE
    )

    return merged


# =========================================================
# 6. 메인
# =========================================================
def main() -> None:
    # 폴더 존재 여부 확인
    if not INPUT_DIR.exists():
        print(f"[ERROR] Input directory not found: {INPUT_DIR}")
        sys.exit(1)

    if not OUTPUT_DIR.exists():
        print(f"[ERROR] Output directory not found: {OUTPUT_DIR}")
        sys.exit(1)

    # merged, filtered 등의 결과 파일은 입력에서 제외
    pcd_files = sorted(
        path
        for path in INPUT_DIR.glob("*.pcd")
        if "merged" not in path.name.lower()
        and "filtered" not in path.name.lower()
        and "icp" not in path.name.lower()
    )

    if len(pcd_files) < 2:
        print(
            "[ERROR] At least two capture PCD files are required."
        )
        print(f"[ERROR] Input directory: {INPUT_DIR}")
        sys.exit(1)

    print(f"[INFO] Found {len(pcd_files)} capture files")

    for path in pcd_files:
        print(f"  - {path.name}")

    # 첫 번째 캡처를 기준 cloud로 사용
    merged_full = load_point_cloud(pcd_files[0])

    print()
    print(
        f"[INFO] Base cloud: {pcd_files[0].name}"
    )
    print(
        f"[INFO] Base points: {len(merged_full.points)}"
    )

    # 각 캡처가 이미 base_link 좌표계면 identity로 시작
    initial_transform = np.eye(4)

    successful_alignments = 0
    skipped_alignments = 0

    for index, source_path in enumerate(
        pcd_files[1:],
        start=1,
    ):
        print()
        print(
            f"[INFO] Aligning "
            f"{index}/{len(pcd_files) - 1}: "
            f"{source_path.name}"
        )

        try:
            source_full = load_point_cloud(source_path)
        except (FileNotFoundError, RuntimeError) as error:
            print(f"[WARNING] {error}")
            skipped_alignments += 1
            continue

        source_down = preprocess_point_cloud(source_full)
        target_down = preprocess_point_cloud(merged_full)

        print(
            f"[INFO] Source ICP points: "
            f"{len(source_down.points)}"
        )
        print(
            f"[INFO] Target ICP points: "
            f"{len(target_down.points)}"
        )

        if len(source_down.points) < 3:
            print(
                "[WARNING] Source has fewer than 3 points. "
                "Skipped."
            )
            skipped_alignments += 1
            continue

        if len(target_down.points) < 3:
            print(
                "[ERROR] Target has fewer than 3 points."
            )
            sys.exit(1)

        try:
            result = run_icp(
                source=source_down,
                target=target_down,
                initial_transform=initial_transform,
            )
        except RuntimeError as error:
            print(f"[WARNING] ICP failed: {error}")
            skipped_alignments += 1
            continue

        print(f"[INFO] Fitness: {result.fitness:.6f}")
        print(
            f"[INFO] Inlier RMSE: "
            f"{result.inlier_rmse:.6f}"
        )
        print("[INFO] Transformation:")
        print(result.transformation)

        if result.fitness < MIN_FITNESS:
            print(
                f"[WARNING] Fitness below threshold "
                f"({result.fitness:.6f} < {MIN_FITNESS}). "
                "Skipped."
            )
            skipped_alignments += 1
            continue

        if result.inlier_rmse > MAX_RMSE:
            print(
                f"[WARNING] RMSE above threshold "
                f"({result.inlier_rmse:.6f} > {MAX_RMSE}). "
                "Skipped."
            )
            skipped_alignments += 1
            continue

        merged_full = merge_clouds(
            target_full=merged_full,
            source_full=source_full,
            transformation=result.transformation,
        )

        successful_alignments += 1

        print(
            f"[INFO] Merged points: "
            f"{len(merged_full.points)}"
        )

    # 모든 후속 스캔이 실패한 경우 경고
    if successful_alignments == 0:
        print()
        print(
            "[WARNING] No additional scans passed "
            "the ICP quality criteria."
        )
        print(
            "[WARNING] The output will contain only "
            "the first capture."
        )

    success = o3d.io.write_point_cloud(
        str(OUTPUT_PATH),
        merged_full,
    )

    if not success:
        print(f"[ERROR] Save failed: {OUTPUT_PATH}")
        sys.exit(1)

    print()
    print("========================================")
    print(f"[INFO] Saved: {OUTPUT_PATH}")
    print(
        f"[INFO] Successful alignments: "
        f"{successful_alignments}"
    )
    print(
        f"[INFO] Skipped alignments: "
        f"{skipped_alignments}"
    )
    print(
        f"[INFO] Final points: "
        f"{len(merged_full.points)}"
    )
    print("========================================")


if __name__ == "__main__":
    main()