#!/usr/bin/env python3
"""
LiDAR Table Detector Node.

Subscribes to /scan (LaserScan), clusters consecutive close-range points,
and filters for TABLE-shaped objects (0.4–1.2m width) — ignoring small
shelf boxes and large walls/shelves.

The table is the pickup target. Once found, its position is published
so the navigator can approach it.

Topics published:
  /detected_objects     — visualization_msgs/MarkerArray  (RViz markers)
  /detected_box_poses   — geometry_msgs/PoseArray         (table position for navigator)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA


class LidarTableDetector(Node):

    # TABLE filtering thresholds (metres)
    # Table is 0.8m x 0.6m — LiDAR cross-section is ~0.4-1.2m depending on angle
    # Shelf boxes are ~0.2m, shelves are >1.5m, walls are >3m
    MIN_TABLE_WIDTH = 0.35
    MAX_TABLE_WIDTH = 1.3
    MAX_DETECT_RANGE = 8.0       # scan range
    CLUSTER_BREAK_DIST = 0.20    # gap to break cluster

    # Isolation filter: table should NOT be adjacent to a very large structure
    NEARBY_LARGE_STRUCTURE_DIST = 0.6   # if a big cluster is within this, skip

    def __init__(self):
        super().__init__('lidar_box_detector')

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, qos)

        self.marker_pub = self.create_publisher(
            MarkerArray, '/detected_objects', 10)
        self.pose_pub = self.create_publisher(
            PoseArray, '/detected_box_poses', 10)

        self.get_logger().info('LiDAR Table Detector ready — listening on /scan')
        self.get_logger().info(
            f'  Table filter: width [{self.MIN_TABLE_WIDTH:.2f}, '
            f'{self.MAX_TABLE_WIDTH:.2f}]m')

    # ── Scan callback ─────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        clusters = self._cluster_scan(msg)
        tables = self._filter_tables(clusters)
        self._publish(tables, msg.header)

    # ── Clustering ────────────────────────────────────────────────────────

    def _cluster_scan(self, msg: LaserScan):
        """Group consecutive valid points into clusters."""
        clusters = []
        current = []

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min or r > self.MAX_DETECT_RANGE:
                if current:
                    clusters.append(current)
                    current = []
                continue

            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            if current:
                px, py, _ = current[-1]
                if math.hypot(x - px, y - py) > self.CLUSTER_BREAK_DIST:
                    clusters.append(current)
                    current = []

            current.append((x, y, r))

        if current:
            clusters.append(current)

        return clusters

    # ── Table filter ──────────────────────────────────────────────────────

    def _filter_tables(self, clusters):
        """Return list of (cx, cy, width) for clusters matching TABLE dimensions.

        A table appears as a medium-width cluster (0.4–1.2m) that is NOT
        part of a wall or shelf (which are >1.5m wide).
        """
        # First pass: compute all cluster info
        cluster_info = []
        for cluster in clusters:
            if len(cluster) < 3:
                continue
            xs = [p[0] for p in cluster]
            ys = [p[1] for p in cluster]
            width = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
            cx = sum(xs) / len(xs)
            cy = sum(ys) / len(ys)
            cluster_info.append((cx, cy, width))

        # Identify large structures (shelves, walls)
        large_structures = [
            (cx, cy) for cx, cy, w in cluster_info if w > 1.5
        ]

        # Second pass: keep only table-sized clusters that are NOT near large structures
        tables = []
        for cx, cy, width in cluster_info:
            if not (self.MIN_TABLE_WIDTH <= width <= self.MAX_TABLE_WIDTH):
                continue

            # Check isolation: skip if near a large shelf/wall
            near_large = False
            for lx, ly in large_structures:
                if math.hypot(cx - lx, cy - ly) < self.NEARBY_LARGE_STRUCTURE_DIST:
                    near_large = True
                    break

            if not near_large:
                tables.append((cx, cy, width))

        return tables

    # ── Publishing ────────────────────────────────────────────────────────

    def _publish(self, tables, header: Header):
        # --- MarkerArray ---
        ma = MarkerArray()
        delete_marker = Marker()
        delete_marker.header = header
        delete_marker.action = Marker.DELETEALL
        ma.markers.append(delete_marker)

        for i, (cx, cy, w) in enumerate(tables):
            m = Marker()
            m.header = header
            m.ns = 'detected_tables'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.4
            m.scale.x = w
            m.scale.y = w
            m.scale.z = 0.8
            m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.6)
            m.lifetime.sec = 1
            ma.markers.append(m)

            # Text label
            t = Marker()
            t.header = header
            t.ns = 'table_labels'
            t.id = i + 100
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = cx
            t.pose.position.y = cy
            t.pose.position.z = 1.0
            dist = math.hypot(cx, cy)
            t.text = f'TABLE ({dist:.1f}m, w={w:.2f}m)'
            t.scale.z = 0.2
            t.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            t.lifetime.sec = 1
            ma.markers.append(t)

        self.marker_pub.publish(ma)

        # --- PoseArray (only table positions) ---
        pa = PoseArray()
        pa.header = header
        for cx, cy, _ in tables:
            p = Pose()
            p.position.x = cx
            p.position.y = cy
            p.position.z = 0.0
            p.orientation.w = 1.0
            pa.poses.append(p)

        self.pose_pub.publish(pa)

        if tables:
            self.get_logger().info(
                f'Detected {len(tables)} table(s) — nearest at '
                f'{min(math.hypot(t[0], t[1]) for t in tables):.2f}m',
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = LidarTableDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
