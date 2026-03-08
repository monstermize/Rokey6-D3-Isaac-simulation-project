import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time


class TestROS2Bridge(Node):
    def __init__(self):

        super().__init__("test_ros2bridge")

        self.publisher_ = self.create_publisher(JointState, "/joint_command", 10)

        self.joint_state = JointState()
        self.joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        
        num_joints = len(self.joint_state.name)

        self.default_joints = np.array([0.0, 0.0, -0.0, -2.3, -0.0, 1.6], dtype=np.float64)

        self.joint_state.position = self.default_joints.tolist()

        self.oscillation_mask = np.array([3.0, 3.0, 1.0, 3.0, 1.0, 3.0], dtype=np.float64)
        # self.oscillation_mask = np.array([0.0, 0.0, 1.0, 0.0, 1.0, 0.0], dtype=np.float64)

        amplitude_deg = 90.0
        self.amplitude_rad = np.deg2rad(amplitude_deg)
        
        self.time_start = time.time()

        timer_period = 0.05  
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()

        elapsed_time = time.time() - self.time_start
        oscillation_value = np.sin(elapsed_time) * self.amplitude_rad

        joint_position = self.default_joints + (self.oscillation_mask * oscillation_value)

        self.joint_state.position = joint_position.tolist()

        print(self.joint_state)
        self.publisher_.publish(self.joint_state)

def main(args=None):
    rclpy.init(args=args)

    ros2_publisher = TestROS2Bridge()

    rclpy.spin(ros2_publisher)

    ros2_publisher.destroy_node()
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()