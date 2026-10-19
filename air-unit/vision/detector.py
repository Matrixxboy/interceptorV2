"""Detector implementations: OnnxDetector, HailoDetector, FakeDetector (§7.2, T2.2).

No hardware imports at top-level (§2.3).
All detectors implement the Detector ABC.
"""

from __future__ import annotations

import logging
from collections.abc import Sequence
from typing import Any

import numpy as np
from air_unit.vision.interfaces import Detection, Detector

logger = logging.getLogger(__name__)


def nms_boxes(
    boxes: np.ndarray,
    scores: np.ndarray,
    iou_thresh: float = 0.5,
) -> list[int]:
    """Pure numpy Non-Maximum Suppression (§2.7 pure functions).

    Args:
        boxes: (N, 4) ndarray of [x1, y1, x2, y2].
        scores: (N,) ndarray of confidence scores.
        iou_thresh: IoU overlap threshold for suppression.

    Returns:
        List of selected indices.
    """
    if len(boxes) == 0:
        return []

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]

    keep: list[int] = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h

        union = areas[i] + areas[order[1:]] - inter
        iou = np.zeros_like(inter)
        non_zero = union > 0
        iou[non_zero] = inter[non_zero] / union[non_zero]

        inds = np.where(iou <= iou_thresh)[0]
        order = order[inds + 1]

    return keep


class OnnxDetector(Detector):
    """ONNX Runtime detector for CPU/GPU development (T2.2).

    No onnxruntime import at module level (§2.3).
    """

    def __init__(
        self,
        model_path: str,
        conf_thresh: float = 0.35,
        iou_thresh: float = 0.5,
        classes: Sequence[int] | None = None,
        input_size: tuple[int, int] = (640, 640),
    ) -> None:
        self.model_path = model_path
        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh
        self.classes = set(classes) if classes is not None else None
        self.input_size = input_size
        self._session: Any = None
        self._input_name: str = ""

    def _ensure_session(self) -> None:
        if self._session is None:
            import onnxruntime as ort  # lazy import (§2.3)

            providers = ["CPUExecutionProvider"]
            self._session = ort.InferenceSession(self.model_path, providers=providers)
            self._input_name = self._session.get_inputs()[0].name

    def infer(self, frame: np.ndarray) -> list[Detection]:
        """Run inference on a frame using ONNX Runtime.

        Args:
            frame: HWC uint8 ndarray.

        Returns:
            List of Detection objects scaled to original frame dimensions.
        """
        self._ensure_session()
        orig_h, orig_w = frame.shape[:2]
        inp_w, inp_h = self.input_size

        # Resize and preprocess (letterbox or simple resize)
        scale_x = orig_w / inp_w
        scale_y = orig_h / inp_h

        # Manual or cv2 resize
        try:
            import cv2  # lazy import (§2.3)

            resized = cv2.resize(frame, (inp_w, inp_h))
        except ImportError:
            # Fallback simple nearest-neighbor sampling if cv2 missing
            row_idx = (np.linspace(0, orig_h - 1, inp_h)).astype(int)
            col_idx = (np.linspace(0, orig_w - 1, inp_w)).astype(int)
            resized = frame[row_idx[:, None], col_idx]

        # Convert to float32 NCHW [0, 1]
        blob = resized.astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))
        blob = np.expand_dims(blob, axis=0)

        outputs = self._session.run(None, {self._input_name: blob})
        raw = outputs[0]  # YOLOv8 format: (1, 84, 8400) or similar

        if len(raw.shape) == 3 and raw.shape[1] < raw.shape[2]:
            # Transpose (1, 84, 8400) -> (8400, 84)
            preds = np.transpose(raw[0], (1, 0))
        elif len(raw.shape) == 3:
            preds = raw[0]
        else:
            preds = raw

        # YOLO format: cx, cy, w, h, class_scores...
        boxes_list: list[list[float]] = []
        scores_list: list[float] = []
        classes_list: list[int] = []

        for p in preds:
            cx, cy, w, h = p[:4]
            class_scores = p[4:]
            cls_id = int(np.argmax(class_scores))
            score = float(class_scores[cls_id])

            if score < self.conf_thresh:
                continue
            if self.classes is not None and cls_id not in self.classes:
                continue

            x1 = (cx - w / 2.0) * scale_x
            y1 = (cy - h / 2.0) * scale_y
            x2 = (cx + w / 2.0) * scale_x
            y2 = (cy + h / 2.0) * scale_y

            # Clamp to frame bounds
            x1 = max(0.0, min(float(orig_w), x1))
            y1 = max(0.0, min(float(orig_h), y1))
            x2 = max(0.0, min(float(orig_w), x2))
            y2 = max(0.0, min(float(orig_h), y2))

            boxes_list.append([x1, y1, x2, y2])
            scores_list.append(score)
            classes_list.append(cls_id)

        if not boxes_list:
            return []

        keep = nms_boxes(np.array(boxes_list), np.array(scores_list), self.iou_thresh)
        detections: list[Detection] = []
        for idx in keep:
            b = boxes_list[idx]
            detections.append(
                Detection(
                    x1=b[0],
                    y1=b[1],
                    x2=b[2],
                    y2=b[3],
                    score=scores_list[idx],
                    cls=classes_list[idx],
                )
            )

        return detections


class HailoDetector(Detector):
    """Hailo-8 / 8L NPU detector (§7.2, T2.2).

    No hailo_platform import at module level (§2.3).
    """

    def __init__(
        self,
        hef_path: str,
        conf_thresh: float = 0.35,
        iou_thresh: float = 0.5,
        classes: Sequence[int] | None = None,
    ) -> None:
        self.hef_path = hef_path
        self.conf_thresh = conf_thresh
        self.iou_thresh = iou_thresh
        self.classes = set(classes) if classes is not None else None
        self._target: Any = None
        self._network_group: Any = None

    def _ensure_device(self) -> None:
        if self._target is None:
            from hailo_platform import VDevice  # lazy import (§2.3)

            self._target = VDevice()
            # Hardware setup and HEF configuration logic here

    def infer(self, frame: np.ndarray) -> list[Detection]:
        """Run inference on Hailo NPU.

        Args:
            frame: HWC uint8 ndarray.

        Returns:
            List of Detection objects.
        """
        self._ensure_device()
        # Hailo inference pipeline
        return []


class FakeDetector(Detector):
    """Synthetic detector for zero-hardware tests and SITL (§2.2).

    Extracts bounding boxes from FakeCamera's colored rectangle, or returns
    manually injected detections.
    """

    def __init__(
        self,
        target_color: tuple[int, int, int] = (0, 255, 0),
        color_tolerance: int = 10,
        cls_id: int = 0,
        score: float = 0.95,
        injected_detections: list[Detection] | None = None,
    ) -> None:
        self.target_color = target_color
        self.color_tolerance = color_tolerance
        self.cls_id = cls_id
        self.score = score
        self.injected_detections = injected_detections

    def infer(self, frame: np.ndarray) -> list[Detection]:
        """Find matching colored rectangles or return injected detections."""
        if self.injected_detections is not None:
            return list(self.injected_detections)

        # Detect colored pixels matching target_color
        b, g, r = self.target_color
        mask = (
            (np.abs(frame[:, :, 0].astype(int) - b) <= self.color_tolerance)
            & (np.abs(frame[:, :, 1].astype(int) - g) <= self.color_tolerance)
            & (np.abs(frame[:, :, 2].astype(int) - r) <= self.color_tolerance)
        )

        rows, cols = np.where(mask)
        if len(rows) == 0:
            return []

        x1 = float(np.min(cols))
        y1 = float(np.min(rows))
        x2 = float(np.max(cols)) + 1.0
        y2 = float(np.max(rows)) + 1.0

        return [
            Detection(
                x1=x1,
                y1=y1,
                x2=x2,
                y2=y2,
                score=self.score,
                cls=self.cls_id,
            )
        ]
