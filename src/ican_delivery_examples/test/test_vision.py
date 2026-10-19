from pathlib import Path

import cv2
import numpy as np
import pytest
import json
import rclpy
from cv_bridge import CvBridge
from std_msgs.msg import String

from ican_delivery_examples.window_detector import ReferenceMatcher, WindowDetector


DIRECTORY = Path(__file__).resolve().parents[3] / "binary_install/share/ican_delivery_sim/materials/textures/window_candidates"


@pytest.fixture(scope="module")
def matcher():
    return ReferenceMatcher(DIRECTORY)


@pytest.mark.parametrize("name", ["window_bird_01", "window_bird_02", "window_target_1", "window_target_2"])
@pytest.mark.parametrize("flipped", [False, True])
def test_recognizes_camera_like_warp(matcher, name, flipped):
    source = cv2.imread(str(DIRECTORY / f"{name}.jpg"), 0)
    if flipped:
        source = cv2.flip(source, 1)
    h, w = source.shape
    matrix = cv2.getPerspectiveTransform(np.float32([[0, 0], [w, 0], [w, h], [0, h]]),
        np.float32([[190, 210], [580, 225], [560, 480], [200, 460]]))
    view = cv2.warpPerspective(source, matrix, (800, 800), borderValue=130)
    result = matcher.detect(view)
    assert result is not None
    assert result["label"] == name


def test_no_detection_on_blank_or_random_scene(matcher):
    assert matcher.detect(np.full((800, 800), 130, np.uint8)) is None
    noise = np.random.default_rng(42).integers(0, 256, (800, 800), dtype=np.uint8)
    assert matcher.detect(noise) is None


def test_aruco_against_dark_gazebo_wall():
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters_create()
    params.minMarkerDistanceRate = .01
    markers = DIRECTORY.parent / "aruco_markers"
    for identifier in (0, 3, 9, 21, 23):
        marker = cv2.imread(str(markers / f"aruco_4x4_50_id_{identifier:02d}.png"), 0)
        scene = np.full((800, 800), 75, np.uint8)
        scene[180:580, 200:600] = cv2.resize(marker, (400, 400))
        _, ids, _ = cv2.aruco.detectMarkers(scene, dictionary, parameters=params)
        assert ids is not None and identifier in ids.flatten()


def test_confirmed_evidence_is_saved_from_the_same_frame(tmp_path):
    rclpy.init()
    node = WindowDetector()
    try:
        evidence = tmp_path / "window_1.jpg"
        request = dict(window="window_1", token="test_request", evidence_file=str(evidence))
        node.on_request(String(data=json.dumps(request)))
        marker = cv2.imread(str(DIRECTORY.parent / "aruco_markers/aruco_4x4_50_id_03.png"))
        scene = np.full((800, 800, 3), 75, np.uint8)
        scene[180:580, 200:600] = cv2.resize(marker, (400, 400))
        for i in range(3):
            frame = CvBridge().cv2_to_imgmsg(scene, "bgr8")
            frame.header.stamp.sec = i+1
            node.last_frame = 0.
            node.on_image(frame)
            node.on_request(String(data=json.dumps(request)))
            assert evidence.exists() == (i == 2)
        assert node.confirmed["marker_id"] == 3
        assert node.confirmed["confirmed_frames"] == 3
        assert node.confirmed["image_stamp"] == [3, 0]
        assert node.confirmed["evidence_file"] == str(evidence)
        node.on_request(String(data=json.dumps(dict(window="window_2", token="next"))))
        assert node.confirmed is None and node.frames == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
