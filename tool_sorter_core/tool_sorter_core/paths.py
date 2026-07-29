"""설치형/소스형 실행 모두를 지원하는 모델 경로 해석기."""

from pathlib import Path


PACKAGE_NAME = "tool_sorter_core"


def package_share() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(PACKAGE_NAME))
    except Exception:
        return Path(__file__).resolve().parents[1]


def default_weights_path() -> str:
    return str(package_share() / "models" / "best.pt")


def default_calibration_path() -> str:
    return str(package_share() / "models" / "T_gripper2camera.npy")
