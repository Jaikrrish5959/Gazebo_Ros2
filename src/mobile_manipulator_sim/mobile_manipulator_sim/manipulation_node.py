from action_msgs.msg import GoalStatusArray, GoalStatus
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64

class ManipulationNode(Node):
    def __init__(self):
        super().__init__('manipulation_node')
        
        # Subscribe to Nav2 action status
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.nav_status_callback, 10)
        
        # Publishers for joints
        self.pubs = {}
        joints = [
            'shoulder_joint', 'elbow_joint', 'wrist_joint', 
            'finger_left_joint', 'finger_right_joint'
        ]
        
        for j in joints:
            topic = f'/model/mobile_manipulator/joint/{j}/cmd_pos'
            self.pubs[j] = self.create_publisher(Float64, topic, 10)
            
        self.state = "SEARCH" # SEARCH, PRE_GRASP, GRASP, LIFT
        self.timer = self.create_timer(1.0, self.control_loop)
        
        # Poses
        self.poses = {
            'HOME': {'shoulder': 0.0, 'elbow': 0.0, 'wrist': 0.0, 'gripper': 0.04}, # Gripper Open
            'PRE_GRASP': {'shoulder': 0.0, 'elbow': 0.5, 'wrist': 1.0, 'gripper': 0.04}, 
            'GRASP_LOWER': {'shoulder': 0.0, 'elbow': 1.0, 'wrist': 0.5, 'gripper': 0.04},
            'GRASP_CLOSE': {'shoulder': 0.0, 'elbow': 1.0, 'wrist': 0.5, 'gripper': -0.01}, # Close tight
            'LIFT': {'shoulder': -0.5, 'elbow': 0.0, 'wrist': 0.0, 'gripper': -0.01}
        }
        
        self.target_pose = 'HOME'
        self.nav_arrived = False
        
        self.get_logger().info('Manipulation Node Started')

    def nav_status_callback(self, msg):
        # Check if any goal has succeeded
        arrived = False
        if msg.status_list:
            # Check the latest status
            latest_status = msg.status_list[-1]
            if latest_status.status == GoalStatus.STATUS_SUCCEEDED:
                arrived = True
        
        if arrived:
            self.nav_arrived = True
        else:
            # Only reset if we were already searching, to prevent reset during manipulation
            if self.state == "SEARCH":
                self.nav_arrived = False
                self.target_pose = 'HOME'

    def control_loop(self):
        if self.state == "SEARCH":
            if self.nav_arrived:
                self.get_logger().info('Arrived at target. Starting Manipulation.')
                self.state = "PRE_GRASP"
                self.target_pose = 'PRE_GRASP'
            else:
                self.target_pose = 'HOME'
                
        elif self.state == "PRE_GRASP":
            # Wait a bit then lower
            self.state = "GRASP_LOWER"
            self.target_pose = 'GRASP_LOWER'
            
        elif self.state == "GRASP_LOWER":
            self.state = "GRASP_CLOSE"
            self.target_pose = 'GRASP_CLOSE'
            
        elif self.state == "GRASP_CLOSE":
            self.state = "LIFT"
            self.target_pose = 'LIFT'
            
        elif self.state == "LIFT":
            pass # Stay holding
            
        self.publish_pose(self.target_pose)

    def publish_pose(self, pose_name):
        p = self.poses[pose_name]
        
        def pub(name, val):
            msg = Float64()
            msg.data = float(val)
            self.pubs[name].publish(msg)
            
        pub('shoulder_joint', p['shoulder'])
        pub('elbow_joint', p['elbow'])
        pub('wrist_joint', p['wrist'])
        pub('finger_left_joint', p['gripper'])
        pub('finger_right_joint', p['gripper'])

def main(args=None):
    rclpy.init(args=args)
    node = ManipulationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
