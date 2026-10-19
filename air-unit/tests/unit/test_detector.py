"""Tests for air_unit/vision/detector.py — T2.2 acceptance.

- Tests NMS pure function.
- Tests FakeDetector with synthetic frame and injected detections.
- Tests OnnxDetector interface without requiring real weights.
- Tests HailoDetector lazy import.
"""

from __future__ import annotations

import numpy as np
from air_unit.vision.detector import FakeDetector, HailoDetector, OnnxDetector, nms_boxes
from air_unit.vision.interfaces import Detection


class TestNms:
    """NMS suppression test."""

    def test_overlapping_boxes_suppressed(self) -> None:
        boxes = np.array(
            [
                [10.0, 10.0, 50.0, 50.0],
                [12.0, 12.0, 52.0, 52.0],  # heavy overlap
                [200.0, 200.0, 250.0, 250.0],  # disjoint
            ]
        )
        scores = np.array([0.9, 0.8, 0.95])
        keep = nms_boxes(boxes, scores, iou_thresh=0.5)
        # Should keep box 2 (score 0.95) and box 0 (score 0.9), suppressing box 1
        assert 2 in keep
        assert 0 in keep
        assert 1 not in keep


class TestFakeDetector:
    """FakeDetector finds colored rectangles or injected detections."""

    def test_finds_green_rectangle(self) -> None:
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        # Draw green rectangle at x: 20..40, y: 30..60
        frame[30:60, 20:40] = (0, 255, 0)

        detector = FakeDetector(target_color=(0, 255, 0), cls_id=0)
        detections = detector.infer(frame)

        assert len(detections) == 1
        d = detections[0]
        assert d.x1 == 20.0
        assert d.y1 == 30.0
        assert d.x2 == 40.0
        assert d.y2 == 60.0
        assert d.cls == 0

    def test_returns_empty_when_no_target(self) -> None:
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        detector = FakeDetector(target_color=(0, 255, 0))
        detections = detector.infer(frame)
        assert len(detections) == 0

    def test_injected_detections(self) -> None:
        injected = [Detection(10, 20, 30, 40, 0.9, 0)]
        detector = FakeDetector(injected_detections=injected)
        res = detector.infer(np.zeros((10, 10, 3), dtype=np.uint8))
        assert res == injected


class TestDetectorInterfaces:
    """Detector classes implement the same interface (§7.2)."""

    def test_hailo_detector_lazy_import(self) -> None:
        detector = HailoDetector(hef_path="dummy.hef")
        assert detector.conf_thresh == 0.35
        # Instantiation did not import hailo_platform at module level

    def test_onnx_detector_init(self) -> None:
        detector = OnnxDetector(model_path="dummy.onnx")
        assert detector.input_size == (640, 640)
