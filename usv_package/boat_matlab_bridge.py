#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class BoatMatlabBridge(Node):
    """
    A bridge to facilitate communication with an external MATLAB controller.

    This node serves two main purposes:
    1. Odometry Forwarding: It subscribes to the odometry data published by
       Gazebo and republishes it on a dedicated topic for MATLAB, ensuring
       correct QoS compatibility.
    2. Command Forwarding: It subscribes to wrench commands published by
       MATLAB and republishes them on the topic expected by the USV's
       thruster mixer, again ensuring correct QoS.
    """
    def __init__(self):
        super().__init__('boat_matlab_bridge')

        # --- QoS Profile Definitions ---
        # For topics where the source or destination is a critical simulation
        # component (like Gazebo plugins or controllers) that defaults to reliable.
        reliable_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # For topics where the source or destination is the MATLAB script,
        # which is configured to use best_effort.
        best_effort_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Odometry Bridge (Gazebo -> MATLAB) ---
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',  # Source: Gazebo's p3d plugin
            self.odom_callback,
            qos_profile=reliable_profile  # Match Gazebo's default reliable publisher
        )
        self.matlab_odom_pub = self.create_publisher(
            Odometry, 
            '/matlab/odom',  # Destination: MATLAB subscriber
            qos_profile=best_effort_profile  # Match MATLAB's best_effort subscriber
        )

        # --- Command Bridge (MATLAB -> Thruster Mixer) ---
        self.cmd_sub = self.create_subscription(
            WrenchStamped,
            '/matlab/cmd_wrench',  # Source: MATLAB publisher
            self.cmd_callback,
            qos_profile=best_effort_profile  # Match MATLAB's best_effort publisher
        )
        self.thruster_cmd_pub = self.create_publisher(
            WrenchStamped, 
            '/cmd_wrench',  # Destination: thruster_mixer node
            qos_profile=reliable_profile  # Match thruster_mixer's reliable subscriber
        )

        self.get_logger().info('Boat MATLAB bridge has started with corrected QoS for Foxy.')

    def odom_callback(self, msg: Odometry):
        """Receives odometry from Gazebo and republishes it for MATLAB."""
        self.matlab_odom_pub.publish(msg)

    def cmd_callback(self, msg: WrenchStamped):
        """Receives commands from MATLAB and republishes them for the thruster mixer."""
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
