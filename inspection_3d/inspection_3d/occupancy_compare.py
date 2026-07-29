"""voxel occupancy 기반 inspection_3d 비교 알고리즘.

ROS 의존성을 두지 않고, 순수 파일 입력과 수치 연산만으로 결과를 계산한다.
그래야 comparison 노드뿐 아니라 오프라인 분석 스크립트와 단위 테스트에서도
같은 규칙을 재사용할 수 있다.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import json

import numpy as np
import open3d as o3d


@dataclass(frozen=True)
class ComparisonConfig:
    """비교 허용 오차와 ROI 범위를 담는 불변 설정."""

    voxel_size: float = 0.003
    neighbor_tolerance: int = 1
    min_similarity: float = 0.85
    max_missing_ratio: float = 0.10
    max_added_ratio: float = 0.10
    use_roi: bool = True
    roi_min: np.ndarray | None = None
    roi_max: np.ndarray | None = None


@dataclass(frozen=True)
class ComparisonMetrics:
    """비교 결과를 HMI/로그/파일 저장에 공통으로 쓰는 요약 값."""

    reference_voxel_count: int
    test_voxel_count: int
    common_voxel_count: int
    missing_voxel_count: int
    added_voxel_count: int
    reference_coverage: float
    test_coverage: float
    similarity: float
    missing_ratio: float
    added_ratio: float
    is_normal: bool
    result_text: str


@dataclass(frozen=True)
class ComparisonResult:
    metrics: ComparisonMetrics
    common_pcd: o3d.geometry.PointCloud
    missing_pcd: o3d.geometry.PointCloud
    added_pcd: o3d.geometry.PointCloud
    union_pcd: o3d.geometry.PointCloud


def load_point_cloud(path: Path) -> o3d.geometry.PointCloud:
    if not path.exists():
        raise FileNotFoundError(f"File not found: {path}")

    pcd = o3d.io.read_point_cloud(str(path))
    if len(pcd.points) == 0:
        raise RuntimeError(f"Empty point cloud: {path}")
    return pcd


def crop_point_cloud(
    pcd: o3d.geometry.PointCloud,
    roi_min: np.ndarray,
    roi_max: np.ndarray,
) -> o3d.geometry.PointCloud:
    bbox = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=roi_min,
        max_bound=roi_max,
    )
    return pcd.crop(bbox)


def points_to_voxel_set(
    pcd: o3d.geometry.PointCloud,
    voxel_size: float,
) -> set[tuple[int, int, int]]:
    points = np.asarray(pcd.points, dtype=np.float64)
    voxel_indices = np.floor(points / voxel_size).astype(np.int64)
    return {tuple(index) for index in voxel_indices}


def has_neighbor(
    voxel: tuple[int, int, int],
    target_voxels: set[tuple[int, int, int]],
    tolerance: int,
) -> bool:
    x, y, z = voxel
    for dx in range(-tolerance, tolerance + 1):
        for dy in range(-tolerance, tolerance + 1):
            for dz in range(-tolerance, tolerance + 1):
                if (x + dx, y + dy, z + dz) in target_voxels:
                    return True
    return False


def voxel_set_to_point_cloud(
    voxel_set: set[tuple[int, int, int]],
    voxel_size: float,
) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    if not voxel_set:
        return pcd

    voxel_indices = np.asarray(list(voxel_set), dtype=np.float64)
    voxel_centers = (voxel_indices + 0.5) * voxel_size
    pcd.points = o3d.utility.Vector3dVector(voxel_centers)
    return pcd


def paint_result_clouds(
    common_pcd: o3d.geometry.PointCloud,
    missing_pcd: o3d.geometry.PointCloud,
    added_pcd: o3d.geometry.PointCloud,
    union_pcd: o3d.geometry.PointCloud,
) -> None:
    if len(common_pcd.points) > 0:
        common_pcd.paint_uniform_color([0.0, 1.0, 0.0])
    if len(missing_pcd.points) > 0:
        missing_pcd.paint_uniform_color([0.0, 0.0, 1.0])
    if len(added_pcd.points) > 0:
        added_pcd.paint_uniform_color([1.0, 0.0, 0.0])
    if len(union_pcd.points) > 0:
        union_pcd.paint_uniform_color([0.7, 0.7, 0.7])


def compare_point_clouds(
    reference_pcd: o3d.geometry.PointCloud,
    test_pcd: o3d.geometry.PointCloud,
    config: ComparisonConfig,
) -> ComparisonResult:
    if config.use_roi:
        if config.roi_min is None or config.roi_max is None:
            raise ValueError("ROI bounds are required when use_roi=True.")
        reference_pcd = crop_point_cloud(reference_pcd, config.roi_min, config.roi_max)
        test_pcd = crop_point_cloud(test_pcd, config.roi_min, config.roi_max)

    if len(reference_pcd.points) == 0:
        raise RuntimeError("Reference PCD is empty after ROI.")
    if len(test_pcd.points) == 0:
        raise RuntimeError("Test PCD is empty after ROI.")

    reference_voxels = points_to_voxel_set(reference_pcd, config.voxel_size)
    test_voxels = points_to_voxel_set(test_pcd, config.voxel_size)

    common_voxels = {
        voxel
        for voxel in test_voxels
        if has_neighbor(voxel, reference_voxels, config.neighbor_tolerance)
    }
    added_voxels = {
        voxel
        for voxel in test_voxels
        if not has_neighbor(voxel, reference_voxels, config.neighbor_tolerance)
    }
    missing_voxels = {
        voxel
        for voxel in reference_voxels
        if not has_neighbor(voxel, test_voxels, config.neighbor_tolerance)
    }
    union_voxels = reference_voxels | test_voxels

    missing_ratio = (
        len(missing_voxels) / len(reference_voxels)
        if reference_voxels
        else 0.0
    )
    added_ratio = len(added_voxels) / len(test_voxels) if test_voxels else 0.0
    reference_coverage = 1.0 - missing_ratio
    test_coverage = 1.0 - added_ratio
    similarity = (reference_coverage + test_coverage) / 2.0
    is_normal = (
        similarity >= config.min_similarity
        and missing_ratio <= config.max_missing_ratio
        and added_ratio <= config.max_added_ratio
    )
    result_text = "NORMAL" if is_normal else "ABNORMAL"

    common_pcd = voxel_set_to_point_cloud(common_voxels, config.voxel_size)
    missing_pcd = voxel_set_to_point_cloud(missing_voxels, config.voxel_size)
    added_pcd = voxel_set_to_point_cloud(added_voxels, config.voxel_size)
    union_pcd = voxel_set_to_point_cloud(union_voxels, config.voxel_size)
    paint_result_clouds(common_pcd, missing_pcd, added_pcd, union_pcd)

    metrics = ComparisonMetrics(
        reference_voxel_count=len(reference_voxels),
        test_voxel_count=len(test_voxels),
        common_voxel_count=len(common_voxels),
        missing_voxel_count=len(missing_voxels),
        added_voxel_count=len(added_voxels),
        reference_coverage=reference_coverage,
        test_coverage=test_coverage,
        similarity=similarity,
        missing_ratio=missing_ratio,
        added_ratio=added_ratio,
        is_normal=is_normal,
        result_text=result_text,
    )
    return ComparisonResult(
        metrics=metrics,
        common_pcd=common_pcd,
        missing_pcd=missing_pcd,
        added_pcd=added_pcd,
        union_pcd=union_pcd,
    )


def compare_point_cloud_files(
    reference_path: Path,
    test_path: Path,
    config: ComparisonConfig,
) -> ComparisonResult:
    reference_pcd = load_point_cloud(reference_path)
    test_pcd = load_point_cloud(test_path)
    return compare_point_clouds(reference_pcd, test_pcd, config)


def save_point_cloud(path: Path, pcd: o3d.geometry.PointCloud) -> None:
    if len(pcd.points) == 0:
        return
    if not o3d.io.write_point_cloud(str(path), pcd):
        raise RuntimeError(f"Failed to save: {path}")


def save_comparison_outputs(
    output_dir: Path,
    reference_path: Path,
    test_path: Path,
    config: ComparisonConfig,
    result: ComparisonResult,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    save_point_cloud(output_dir / "common_voxels.pcd", result.common_pcd)
    save_point_cloud(output_dir / "missing_voxels.pcd", result.missing_pcd)
    save_point_cloud(output_dir / "added_voxels.pcd", result.added_pcd)
    save_point_cloud(output_dir / "union_voxels.pcd", result.union_pcd)

    metadata = {
        "reference_path": str(reference_path),
        "test_path": str(test_path),
        "config": {
            "voxel_size": config.voxel_size,
            "neighbor_tolerance": config.neighbor_tolerance,
            "min_similarity": config.min_similarity,
            "max_missing_ratio": config.max_missing_ratio,
            "max_added_ratio": config.max_added_ratio,
            "use_roi": config.use_roi,
            "roi_min": config.roi_min.tolist() if config.roi_min is not None else None,
            "roi_max": config.roi_max.tolist() if config.roi_max is not None else None,
        },
        "metrics": result.metrics.__dict__,
    }
    with (output_dir / "metrics.json").open("w", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2)


def build_summary(
    result: ComparisonResult,
    output_dir: Path,
) -> str:
    metrics = result.metrics
    return (
        f"result={metrics.result_text}, similarity={metrics.similarity:.4f}, "
        f"missing_ratio={metrics.missing_ratio:.4f}, "
        f"added_ratio={metrics.added_ratio:.4f}, output_dir={output_dir}"
    )
