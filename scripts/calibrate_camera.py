"""scripts/calibrate_camera.py — Calibrate camera intrinsics using a chessboard pattern (§4).

Computes focal length (fx, fy), optical center (cx, cy), and distortion coefficients,
saving the result to air-unit/config/camera_intrinsics.yaml.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml


def calibrate(
    image_dir: Path,
    cols: int = 9,
    rows: int = 6,
    square_size_mm: float = 25.0,
    output_path: Path | None = None,
) -> dict[str, float | list[float]]:
    """Calibrate camera using chessboard images in image_dir."""
    try:
        import cv2
        import numpy as np
    except ImportError as e:
        print(f"Error: OpenCV and NumPy are required for calibration: {e}", file=sys.stderr)
        sys.exit(1)

    pattern_size = (cols, rows)
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size_mm

    objpoints: list[np.ndarray] = []
    imgpoints: list[np.ndarray] = []

    image_files = list(image_dir.glob("*.jpg")) + list(image_dir.glob("*.png"))
    if not image_files:
        raise ValueError(f"No image files (.jpg, .png) found in {image_dir}")

    image_size: tuple[int, int] | None = None

    for fname in image_files:
        img = cv2.imread(str(fname))
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])

        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners2)

    if not objpoints:
        raise RuntimeError("No chessboard corners could be detected in any of the provided images.")

    assert image_size is not None
    ret, mtx, dist, _rvecs, _tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None
    )

    result = {
        "fx": float(mtx[0, 0]),
        "fy": float(mtx[1, 1]),
        "cx": float(mtx[0, 2]),
        "cy": float(mtx[1, 2]),
        "image_width": image_size[0],
        "image_height": image_size[1],
        "distortion_coefficients": [float(x) for x in dist.ravel()[:5]],
        "model": "pinhole",
    }

    if output_path:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as f:
            yaml.dump(result, f, sort_keys=False)
        print(f"Calibrated intrinsics saved to: {output_path}")

    return result


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Calibrate camera intrinsics from chessboard images."
    )
    parser.add_argument(
        "--image-dir", type=Path, required=True, help="Path to folder containing calibration photos"
    )
    parser.add_argument(
        "--cols",
        type=int,
        default=9,
        help="Number of inner chessboard corners across width (default: 9)",
    )
    parser.add_argument(
        "--rows",
        type=int,
        default=6,
        help="Number of inner chessboard corners across height (default: 6)",
    )
    parser.add_argument(
        "--square-size", type=float, default=25.0, help="Square side size in mm (default: 25.0)"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("air-unit/config/camera_intrinsics.yaml"),
        help="Target yaml output path",
    )

    args = parser.parse_args()
    try:
        res = calibrate(args.image_dir, args.cols, args.rows, args.square_size, args.output)
        print(
            f"Calibration successful: fx={res['fx']:.2f}, fy={res['fy']:.2f}, cx={res['cx']:.2f}, cy={res['cy']:.2f}"
        )
    except Exception as exc:
        print(f"Calibration failed: {exc}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
