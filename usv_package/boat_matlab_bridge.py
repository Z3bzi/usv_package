#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class BoatMatlabBridge(Node):
    """
    A bridge to connect an external MATLAB controller with the USV.

    This node forwards odometry data from Gazebo to MATLAB and
    forwards wrench commands from MATLAB back to the USV's thruster mixer.
    It handles the necessary QoS changes for compatibility between the different nodes.
    """
    def __init__(self):
        super().__init__('boat_matlab_bridge')

        # Gazebo and our controller nodes use a reliable QoS profile.
        reliable_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # The MATLAB connection is configured to use best_effort.
        best_effort_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Bridge odometry from Gazebo to MATLAB
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile=reliable_profile
        )
        self.matlab_odom_pub = self.create_publisher(
            Odometry, 
            '/matlab/odom',
            qos_profile=best_effort_profile
        )

        # Bridge commands from MATLAB to the thruster mixer
        self.cmd_sub = self.create_subscription(
            WrenchStamped,
            '/matlab/cmd_wrench',
            self.cmd_callback,
            qos_profile=best_effort_profile
        )
        self.thruster_cmd_pub = self.create_publisher(
            WrenchStamped, 
            '/cmd_wrench',
            qos_profile=reliable_profile
        )

        self.get_logger().info('Boat MATLAB bridge started.')

    def odom_callback(self, msg: Odometry):
        """Forwards odometry from Gazebo to MATLAB."""
        self.matlab_odom_pub.publish(msg)

    def cmd_callback(self, msg: WrenchStamped):
        """Forwards wrench commands from MATLAB to the thruster mixer."""
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
