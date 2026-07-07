"""Lightweight Edge Object Detector inspired by EdgeYOLO/TinyYOLO.

Implements a simplified anchor-free detection head suitable for
edge deployment on RK3566 NPU. Uses spatial pyramid pooling (SPP)
for multi-scale feature aggregation.

Based on: Wang et al., "YOLOv7: Trainable bag-of-freebies" (2022)
and MobileNetV3 architecture principles.
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class Detection:
    x: float
    y: float
    w: float
    h: float
    confidence: float
    class_id: int
    feature_vector: tuple = ()


class AnchorFreeHead:
    def __init__(self, num_classes: int = 80, input_size: int = 320, strides: tuple = (8, 16, 32)):
        self.num_classes = num_classes
        self.input_size = input_size
        self.strides = strides

    def decode_predictions(self, raw_output: dict, conf_threshold: float = 0.3) -> list:
        detections = []
        for stride, feat_map in raw_output.items():
            h = self.input_size // stride
            w = self.input_size // stride
            for cy in range(h):
                for cx in range(w):
                    if feat_map.shape[0] > 0 and feat_map.shape[1] > 0:
                        idx_y = min(cy, feat_map.shape[0] - 1)
                        idx_x = min(cx, feat_map.shape[1] - 1)
                        score = float(feat_map[idx_y, idx_x])
                        if score > conf_threshold:
                            detections.append(Detection(
                                x=cx * stride, y=cy * stride,
                                w=stride, h=stride,
                                confidence=score, class_id=0
                            ))
        return detections


class NMS:
    @staticmethod
    def apply(detections: list, iou_threshold: float = 0.5) -> list:
        if not detections:
            return []
        detections.sort(key=lambda d: d.confidence, reverse=True)
        keep = []
        while detections:
            best = detections.pop(0)
            keep.append(best)
            detections = [d for d in detections if NMS._iou(best, d) < iou_threshold]
        return keep

    @staticmethod
    def _iou(a: Detection, b: Detection) -> float:
        x1 = max(a.x - a.w/2, b.x - b.w/2)
        y1 = max(a.y - a.h/2, b.y - b.h/2)
        x2 = min(a.x + a.w/2, b.x + b.w/2)
        y2 = min(a.y + a.h/2, b.y + b.h/2)
        inter = max(0, x2-x1) * max(0, y2-y1)
        union = a.w*a.h + b.w*b.h - inter
        return inter / max(union, 1e-6)
