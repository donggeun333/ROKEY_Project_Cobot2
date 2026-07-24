#!/usr/bin/env python3

from pathlib import Path
import sys

import numpy as np

from .occupancy_compare import (
    ComparisonConfig,
    build_summary,
    compare_point_cloud_files,
    save_comparison_outputs,
)


REFERENCE_PATH = Path(
    "/home/dg/cobot_ws/data/pipeline/filtered/good_multitap.pcd"
)
TEST_PATH = Path(
    "/home/dg/cobot_ws/data/pipeline/filtered/bad_multitap.pcd"
)
OUTPUT_DIR = Path(
    "/home/dg/cobot_ws/data/pipeline/comparison"
)


def main() -> int:
    config = ComparisonConfig(
        voxel_size=0.003,
        neighbor_tolerance=1,
        min_similarity=0.85,
        max_missing_ratio=0.10,
        max_added_ratio=0.10,
        use_roi=True,
        roi_min=np.array([0.28, 0.01, -0.03], dtype=np.float64),
        roi_max=np.array([0.46, 0.20, 0.10], dtype=np.float64),
    )

    try:
        result = compare_point_cloud_files(REFERENCE_PATH, TEST_PATH, config)
        save_comparison_outputs(
            output_dir=OUTPUT_DIR,
            reference_path=REFERENCE_PATH,
            test_path=TEST_PATH,
            config=config,
            result=result,
        )
        print(build_summary(result, OUTPUT_DIR))
        return 0
    except Exception as error:
        print(f"[ERROR] {error}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
