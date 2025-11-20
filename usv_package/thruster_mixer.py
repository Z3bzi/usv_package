#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, WrenchStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos = QoSProfile(depth=10)
qos.reliability = ReliabilityPolicy.RELIABLE
qos.history = HistoryPolicy.KEEP_LAST

def clip(x, lo, hi):
    return max(lo, min(hi, x))


class ThrusterMixer(Node):
    def __init__(self):
        super().__init__('thruster_mixer')

        self.declare_parameter('max_thrust', 20.0)      # [N] maks fremdriftskraft per thruster
        self.declare_parameter('turn_mix', 0.6)         # hvor “aggressiv” turning er (1.0 = klassisk miks)
        self.declare_parameter('cmd_topic', '/cmd_wrench')
        self.declare_parameter('left_topic', '/usv/left_thrust')
        self.declare_parameter('right_topic', '/usv/right_thrust')

        self.max_thrust = float(self.get_parameter('max_thrust').value)
        self.turn_mix = float(self.get_parameter('turn_mix').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.left_topic = str(self.get_parameter('left_topic').value)
        self.right_topic = str(self.get_parameter('right_topic').value)

        self.pub_left = self.create_publisher(Wrench, self.left_topic, 10)
        self.pub_right = self.create_publisher(Wrench, self.right_topic, 10)

        self.sub = self.create_subscription(WrenchStamped, self.cmd_topic, self.on_cmd_wrench, qos)

        self.get_logger().info(
            f'ThrusterMixer oppe. max_thrust={self.max_thrust}N, turn_mix={self.turn_mix}, '
            f'cmd_topic={self.cmd_topic}, publiserer til {self.left_topic} / {self.right_topic}'
        )

    def on_cmd_wrench(self, msg: WrenchStamped):
        throttle = clip(msg.wrench.force.x, -5.0, 5.0)
        turn = clip(msg.wrench.torque.z, -2.0, 2.0)

        left_cmd = clip(throttle - self.turn_mix * turn, -5.0, 5.0)
        right_cmd = clip(throttle + self.turn_mix * turn, -1.0, 1.0)

        left_N = left_cmd * self.max_thrust
        right_N = right_cmd * self.max_thrust

        wl = Wrench()
        wl.force.x = -left_N
        wr = Wrench()
        wr.force.x = -right_N

        self.pub_left.publish(wl)
        self.pub_right.publish(wr)


def main():
    rclpy.init()
    node = ThrusterMixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
