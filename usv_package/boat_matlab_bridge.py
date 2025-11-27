#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class BoatMatlabBridge(Node):
    """
    A bridge to facilitate communication between a MATLAB boat controller
    and the ROS 2 simulation environment.
    """
    def __init__(self):
        super().__init__('boat_matlab_bridge')

        # QoS Profiles
        # For Gazebo p3d odometry and for thruster_mixer (Foxy defaults = reliable)
        reliable_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # For communication with MATLAB script, which uses best_effort
        best_effort_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Odometry bridge: Gazebo -> MATLAB
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  # Correct topic for p3d plugin output
            self.odom_callback,
            qos_profile=reliable_profile # Match Gazebo's default reliable publisher
        )
        self.matlab_odom_pub = self.create_publisher(
            Odometry, 
            '/matlab/odom', 
            qos_profile=best_effort_profile # Match MATLAB's best_effort subscriber
        )

        # Command bridge: MATLAB -> thruster_mixer
        self.cmd_sub = self.create_subscription(
            WrenchStamped,
            '/matlab/cmd_wrench',
            self.cmd_callback,
            qos_profile=best_effort_profile # Match MATLAB's best_effort publisher
        )
        self.thruster_cmd_pub = self.create_publisher(
            WrenchStamped, 
            '/cmd_wrench', 
            qos_profile=reliable_profile # Match thruster_mixer's reliable subscriber
        )

        self.get_logger().info('Boat MATLAB bridge has started with corrected QoS for Foxy.')

    def odom_callback(self, msg):
        self.matlab_odom_pub.publish(msg)

    def cmd_callback(self, msg):
        self.thruster_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    bridge_node = BoatMatlabBridge()
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
