
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class MatlabBridge(Node):
    def __init__(self):
        super().__init__('matlab_bridge')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/matlab/joint_goals',
            self.listener_callback,
            10)
        self._action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        self.joint_names = [
            '5DOF_V2-v1_Base-v1_Revolute-1',
            '5DOF_V2-v1_Base_tube-v1_Revolute-13',
            '5DOF_V2-v1_Ledd1-v1_Revolute-4',
            '5DOF_V2-v1_Ledd2-v1_Revolute-15',
            '5DOF_V2-v1_Ledd-4-v1_Revolute-9'
        ]
        
        self.get_logger().info('MATLAB bridge started. Listening on /matlab/joint_goals.')

    def listener_callback(self, msg):
        if len(msg.data) != len(self.joint_names):
            self.get_logger().warn(f"Received {len(msg.data)} joint goals, but expected {len(self.joint_names)}. Ignoring.")
            return

        self.get_logger().info(f'Received joint goals: {msg.data}')
        self.send_goal(msg.data)

    def send_goal(self, positions):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 5  # Default time to reach goal

        goal_msg.trajectory.points.append(point)

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

def main(args=None):
    rclpy.init(args=args)
    matlab_bridge = MatlabBridge()
    rclpy.spin(matlab_bridge)
    matlab_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
