#!/usr/bin/env python3
"""
LiDAR Multi-Class Object Detector.

Classifies detected clusters into:
  - BOX       : small object  (0.1 – 0.35m width)
  - TABLE     : medium object (0.36 – 1.3m width)
  - SHELF     : large fixture (1.3 – 4.0m width) — includes conveyor belt frame
  - CONVEYOR  : medium-large (0.5 – 1.5m width) at ~2.5, -3.5 world coords
  - WALL      : very large    (> 4.0m width)

Topics:
  Subscribed:
    /scan                   — sensor_msgs/LaserScan
  Published:
    /detected_objects       — visualization_msgs/MarkerArray  (RViz)
    /detected_box_poses     — geometry_msgs/PoseArray         (all objects)
    /lidar_classification   — std_msgs/String                 (JSON summary)
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA, String


# Width thresholds (metres)
BOX_MIN,      BOX_MAX      = 0.10, 0.35
TABLE_MIN,    TABLE_MAX    = 0.36, 1.30
CONVEYOR_MIN, CONVEYOR_MAX = 0.50, 1.60   # conveyor belt cross-section
SHELF_MIN,    SHELF_MAX    = 1.30, 4.50
# anything wider → WALL

# Colour map: class → (r, g, b)
CLASS_COLORS = {
    'BOX':      (1.0, 0.5, 0.0),   # orange
    'TABLE':    (1.0, 0.0, 0.0),   # red
    'CONVEYOR': (0.0, 0.5, 1.0),   # blue
    'SHELF':    (0.0, 1.0, 0.5),   # green
    'WALL':     (0.5, 0.5, 0.5),   # grey
}

CLUSTER_BREAK_DIST = 0.22
MAX_DETECT_RANGE   = 9.0


def _classify(width: float) -> str:
    if width < BOX_MAX:
        return 'BOX'
    if width < TABLE_MAX:
        return 'TABLE'
    if width < SHELF_MIN:
        # Overlapping range: prefer CONVEYOR if size hints match
        return 'CONVEYOR' if width <= CONVEYOR_MAX else 'SHELF'
    if width < SHELF_MAX:
        return 'SHELF'
    return 'WALL'


class LidarClassifier(Node):

    def __init__(self):
        super().__init__('lidar_box_detector')

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._cb, qos)

        self.marker_pub = self.create_publisher(MarkerArray, '/detected_objects', 10)
        self.pose_pub   = self.create_publisher(PoseArray,   '/detected_box_poses', 10)
        self.class_pub  = self.create_publisher(String,      '/lidar_classification', 10)

        self.get_logger().info(
            'LiDAR Multi-Class Detector ready  '
            '[BOX | TABLE | SHELF | CONVEYOR | WALL]'
        )

    # ── Scan callback ───────────────────────────────────────────────────────

    def _cb(self, msg: LaserScan):
        clusters  = self._cluster(msg)
        detections = self._classify_clusters(clusters)
        self._publish(detections, msg.header)

    # ── Clustering ──────────────────────────────────────────────────────────

    def _cluster(self, msg: LaserScan):
        clusters, current = [], []
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > MAX_DETECT_RANGE:
                if current:
                    clusters.append(current)
                    current = []
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x, y  = r * math.cos(angle), r * math.sin(angle)
            if current:
                px, py, _ = current[-1]
                if math.hypot(x - px, y - py) > CLUSTER_BREAK_DIST:
                    clusters.append(current)
                    current = []
            current.append((x, y, r))
        if current:
            clusters.append(current)
        return clusters

    # ── Classification ──────────────────────────────────────────────────────

    def _classify_clusters(self, clusters):
        results = []
        for cluster in clusters:
            if len(cluster) < 3:
                continue
            xs = [p[0] for p in cluster]
            ys = [p[1] for p in cluster]
            width = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
            cx    = sum(xs) / len(xs)
            cy    = sum(ys) / len(ys)
            dist  = math.hypot(cx, cy)
            label = _classify(width)
            results.append({'label': label, 'cx': cx, 'cy': cy,
                            'width': width, 'dist': dist})
        return results

    # ── Publishing ──────────────────────────────────────────────────────────

    def _publish(self, detections, header: Header):
        ma = MarkerArray()
        pa = PoseArray()
        pa.header = header

        # Clear all old markers
        del_m = Marker()
        del_m.header = header
        del_m.action = Marker.DELETEALL
        ma.markers.append(del_m)

        counts = {}
        for i, d in enumerate(detections):
            label = d['label']
            counts[label] = counts.get(label, 0) + 1
            r, g, b = CLASS_COLORS[label]

            # Sphere marker
            m = Marker()
            m.header   = header
            m.ns       = 'classified'
            m.id       = i
            m.type     = Marker.CYLINDER
            m.action   = Marker.ADD
            m.pose.position.x = d['cx']
            m.pose.position.y = d['cy']
            m.pose.position.z = 0.5
            sz = max(0.15, min(d['width'], 0.8))
            m.scale.x  = sz
            m.scale.y  = sz
            m.scale.z  = 0.4
            m.color    = ColorRGBA(r=r, g=g, b=b, a=0.55)
            m.lifetime.sec = 1
            ma.markers.append(m)

            # Text label
            t = Marker()
            t.header   = header
            t.ns       = 'labels'
            t.id       = i + 500
            t.type     = Marker.TEXT_VIEW_FACING
            t.action   = Marker.ADD
            t.pose.position.x = d['cx']
            t.pose.position.y = d['cy']
            t.pose.position.z = 1.2
            t.text     = f"{label}\n{d['dist']:.1f}m  w={d['width']:.2f}m"
            t.scale.z  = 0.18
            t.color    = ColorRGBA(r=r, g=g, b=b, a=1.0)
            t.lifetime.sec = 1
            ma.markers.append(t)

            # PoseArray entry (tables & boxes only for navigator)
            if label in ('TABLE', 'BOX'):
                p = Pose()
                p.position.x = d['cx']
                p.position.y = d['cy']
                p.orientation.w = 1.0
                pa.poses.append(p)

        self.marker_pub.publish(ma)
        self.pose_pub.publish(pa)

        if detections:
            summary = {k: v for k, v in counts.items()}
            msg = String()
            msg.data = json.dumps(summary)
            self.class_pub.publish(msg)
            self.get_logger().info(
                'Detected: ' + '  '.join(f'{k}×{v}' for k, v in counts.items()),
                throttle_duration_sec=2.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = LidarClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
