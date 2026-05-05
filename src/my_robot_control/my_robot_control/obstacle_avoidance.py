import numpy as np

class ObstacleAvoidance:
    def __init__(self):
        self.safe_distance = 0.6
        self.stop_distance = 0.3

    def is_obstacle_close(self, scan_msg):
        if not scan_msg.ranges: return False
        ranges = np.array(scan_msg.ranges)
        ranges[np.isinf(ranges)] = 30.0
        
        num = len(ranges)
        start, end = num // 3, 2 * num // 3
        front_sector = ranges[start:end]
        
        return np.min(front_sector) < self.safe_distance

    def get_action(self, scan_msg):
        ranges = np.array(scan_msg.ranges)
        ranges[np.isinf(ranges)] = 30.0
        mid = len(ranges) // 2
        
        # Split front field into Left and Right
        right_sector = ranges[mid-90 : mid]
        left_sector = ranges[mid : mid+90]
        
        v = 0.1
        w = -0.8 if np.mean(left_sector) < np.mean(right_sector) else 0.8
        return v, w
