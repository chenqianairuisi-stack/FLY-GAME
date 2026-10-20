"""Camera-only detection: ArUco + SIFT/RANSAC for bundled practice targets.

Never reads material assignments or world files to decide what is visible.
"""
import json
from pathlib import Path
import time

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ReferenceMatcher:
    def __init__(self, directory, min_inliers=8):
        cv2.setNumThreads(1)
        self.sift = cv2.SIFT_create(nfeatures=1800)
        self.matcher = cv2.FlannBasedMatcher(dict(algorithm=1, trees=4), dict(checks=64))
        self.min_inliers = min_inliers
        self.references = []
        for path in sorted(Path(directory).glob("*")):
            if path.suffix.lower() not in (".png", ".jpg", ".jpeg"):
                continue
            gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if gray is None:
                continue
            for width in (300, 600):
                scaled = cv2.resize(gray, (width, int(gray.shape[0]*width/gray.shape[1])))
                for flipped in (False, True):
                    img = cv2.flip(scaled, 1) if flipped else scaled
                    points, descriptors = self.sift.detectAndCompute(img, None)
                    if descriptors is not None:
                        self.references.append((path.stem, img.shape, points, descriptors))

    def detect_regions(self, gray, candidate=None):
        # Find poster rectangles in the camera image before extracting SIFT.
        # This avoids spending most of the frame budget on the floor/walls.
        contours, _ = cv2.findContours(cv2.Canny(gray, 15, 40),
                                      cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        boxes = set()
        for contour in contours:
            x,y,w,h = cv2.boundingRect(contour)
            quad = cv2.approxPolyDP(contour, .03*cv2.arcLength(contour, True), True)
            if (w >= 40 and h >= 30 and w*h < gray.size*.4 and
                    len(quad) == 4 and cv2.isContourConvex(quad)):
                boxes.add((x,y,w,h))
        for x,y,w,h in sorted(boxes, key=lambda box: box[2]*box[3]):
            x0,y0 = max(0,x-12),max(0,y-12)
            crop = gray[y0:min(gray.shape[0],y+h+12), x0:min(gray.shape[1],x+w+12)]
            result = self.detect(crop, candidate)
            if result:
                result["corners"] = (np.array(result["corners"])+[x0,y0]).tolist()
                return result
        # Low-contrast or partially merged poster borders can defeat the
        # contour proposal. Keep the full-frame geometric matcher as a
        # fallback so the faster ROI path does not lose valid candidates.
        return self.detect(gray, candidate)

    def detect(self, gray, candidate=None):
        points, descriptors = self.sift.detectAndCompute(gray, None)
        if descriptors is None or len(points) < self.min_inliers:
            return None
        self.matcher.clear()
        self.matcher.add([descriptors])
        self.matcher.train()
        best = None
        for label, shape, ref_points, ref_descriptors in self.references:
            if candidate is not None and label != candidate:
                continue
            pairs = self.matcher.knnMatch(ref_descriptors, k=2)
            matches = [p[0] for p in pairs if len(p) == 2 and p[0].distance < .72*p[1].distance]
            if len(matches) < self.min_inliers:
                continue
            src = np.float32([ref_points[m.queryIdx].pt for m in matches])
            dst = np.float32([points[m.trainIdx].pt for m in matches])
            homography, mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.)
            if homography is None or mask is None:
                continue
            count = int(mask.sum())
            if count < self.min_inliers or count / len(matches) < .5:
                continue
            h, w = shape
            corners = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
            quad = cv2.perspectiveTransform(corners, homography)
            area = abs(cv2.contourArea(quad))
            if not np.isfinite(quad).all() or not cv2.isContourConvex(quad):
                continue
            if not 500 < area < gray.size*.98:
                continue
            if best is None or count > best["inliers"]:
                best = dict(label=label, inliers=count, corners=quad.reshape(-1, 2).tolist())
        return best


class WindowDetector(Node):
    def __init__(self):
        super().__init__("delivery_detector")
        self.declare_parameter("required_frames", 3)
        self.declare_parameter("min_inliers", 8)
        directory = Path(get_package_share_directory("ican_delivery_sim")) / "materials/textures/window_candidates"
        self.reference = ReferenceMatcher(directory, int(self.get_parameter("min_inliers").value))
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.params = cv2.aruco.DetectorParameters_create()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        # Gazebo's dark wall surrounds a narrow white marker margin. The
        # default .05 merges the marker and outer margin as duplicate squares.
        self.params.minMarkerDistanceRate = .01
        self.request = None
        self.last_frame = 0.
        self.last_label = None
        self.frames = 0
        self.confirmed = None
        self.pub = self.create_publisher(String, "/delivery/detections", 10)
        self.debug = self.create_publisher(Image, "/delivery/debug/image", 1)
        self.status = self.create_publisher(String, "/delivery/window_detector/status", 10)
        self.create_subscription(String, "/delivery/detection_request", self.on_request, 10)
        self.create_subscription(Image, "/uav/camera/image", self.on_image,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info("相机识别就绪：ArUco 4x4_50 / 候选图片特征匹配")

    def on_request(self, msg):
        if msg.data == "cancel":
            self.request = None
        else:
            # Each request has a unique token, echoed in the result.
            request = json.loads(msg.data)
            if request != self.request:
                self.request = request
                self.frames = 0
                self.last_label = None
                self.confirmed = None

    def on_image(self, msg):
        if self.request is None or time.monotonic()-self.last_frame < .03:
            return
        self.last_frame = time.monotonic()
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        result = None
        confirmed = None
        if self.request["window"] == "window_1":
            for flipped in (False, True):
                view = cv2.flip(gray, 1) if flipped else gray
                corners, ids, _ = cv2.aruco.detectMarkers(view, self.dictionary, parameters=self.params)
                if ids is not None:
                    index = max(range(len(corners)), key=lambda i: abs(cv2.contourArea(corners[i])))
                    quad = corners[index].reshape(-1, 2).copy()
                    if flipped:
                        quad[:, 0] = gray.shape[1]-1-quad[:, 0]
                    result = dict(label=f"aruco_{int(ids[index][0])}", marker_id=int(ids[index][0]),
                                  corners=quad.tolist())
                    break
        else:
            result = self.reference.detect_regions(gray, self.last_label)

        if result:
            label = result["label"]
            self.frames = self.frames+1 if label == self.last_label else 1
            self.last_label = label
            cv2.polylines(image, [np.int32(result["corners"])], True, (0, 255, 0), 2)
            cv2.putText(image, f"{label} {self.frames}", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, .8, (0, 255, 0), 2)
            if self.frames >= self.get_parameter("required_frames").value:
                if self.confirmed is None:
                    result.update(self.request)
                    result["confirmed_frames"] = self.frames
                    result["image_stamp"] = [msg.header.stamp.sec, msg.header.stamp.nanosec]
                    # Save the exact confirmed camera frame in the detector.
                    # Moving full images between nodes can drop individual
                    # frames under GUI load; detection must not depend on it.
                    evidence = self.request.get("evidence_file")
                    if evidence and not cv2.imwrite(evidence, image):
                        self.get_logger().error(f"无法保存识别图像：{evidence}")
                        return
                    self.confirmed = result
                confirmed = self.confirmed
        else:
            # Confirmation requires consecutive matching camera frames.
            self.frames = 0
            self.last_label = None
        self.status.publish(String(data=json.dumps(dict(request=self.request, label=self.last_label, frames=self.frames))))
        debug = self.bridge.cv2_to_imgmsg(image, "bgr8")
        debug.header = msg.header
        self.debug.publish(debug)
        if confirmed:
            self.pub.publish(String(data=json.dumps(confirmed)))


def main(args=None):
    rclpy.init(args=args)
    node = WindowDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
