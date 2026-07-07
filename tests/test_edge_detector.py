import sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.ai.edge_detector import AnchorFreeHead, NMS, Detection

def test_empty_output():
    head = AnchorFreeHead(input_size=320, strides=(16,))
    dets = head.decode_predictions({16: np.zeros((20, 20))}, conf_threshold=0.5)
    assert len(dets) == 0

def test_detection_above_threshold():
    head = AnchorFreeHead(input_size=320, strides=(16,))
    feat = np.zeros((20, 20))
    feat[5, 5] = 0.9
    dets = head.decode_predictions({16: feat}, conf_threshold=0.3)
    assert len(dets) == 1
    assert dets[0].confidence == 0.9

def test_nms_removes_duplicates():
    dets = [
        Detection(10, 10, 20, 20, 0.9, 0),
        Detection(11, 11, 20, 20, 0.8, 0),
        Detection(100, 100, 20, 20, 0.7, 0),
    ]
    result = NMS.apply(dets, iou_threshold=0.5)
    assert len(result) == 2

def test_nms_empty():
    assert NMS.apply([]) == []

def test_iou_identical():
    a = Detection(10, 10, 20, 20, 1.0, 0)
    assert NMS._iou(a, a) > 0.99
