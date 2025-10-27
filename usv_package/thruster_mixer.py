#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Wrench

def clip(x, lo, hi):
    return max(lo, min(hi, x))

class ThrusterMixer(Node):
    def __init__(self):
        super().__init__('thruster_mixer')

        # Parametre
        self.declare_parameter('max_thrust', 20.0)     # [N] maks fremdriftskraft per thruster
        self.declare_parameter('turn_mix', 1.0)        # hvor “aggressiv” turning er (1.0 = klassisk miks)
        self.declare_parameter('left_topic', '/usv/left_thrust')
        self.declare_parameter('right_topic', '/usv/right_thrust')
        self.declare_parameter('use_wrench', False)    # sett True hvis plugin vil ha Wrench

        self.max_thrust = float(self.get_parameter('max_thrust').value)
        self.turn_mix   = float(self.get_parameter('turn_mix').value)
        self.left_topic = str(self.get_parameter('left_topic').value)
        self.right_topic= str(self.get_parameter('right_topic').value)
        self.use_wrench = bool(self.get_parameter('use_wrench').value)

        # Publiserere
        if self.use_wrench:
            self.pub_left  = self.create_publisher(Wrench,  self.left_topic,  10)
            self.pub_right = self.create_publisher(Wrench,  self.right_topic, 10)
        else:
            self.pub_left  = self.create_publisher(Vector3, self.left_topic,  10)
            self.pub_right = self.create_publisher(Vector3, self.right_topic, 10)

        # Subscriber på cmd_vel (normalisert [-1,1])
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)

        self.get_logger().info(
            f'ThrusterMixer oppe. max_thrust={self.max_thrust}N, turn_mix={self.turn_mix}, '
            f'publiserer til {self.left_topic} / {self.right_topic}, use_wrench={self.use_wrench}'
        )

    def on_cmd_vel(self, msg: Twist):
        # Normaliserte inputs [-1,1]
        throttle = clip(msg.linear.x,  -1.0, 1.0)
        turn     = clip(msg.angular.z, -1.0, 1.0)

        # Klassisk differensialmiksing
        left_cmd  = clip(throttle - self.turn_mix * turn, -1.0, 1.0)
        right_cmd = clip(throttle + self.turn_mix * turn, -1.0, 1.0)

        # Skaler til Newton
        left_N  = left_cmd  * self.max_thrust
        right_N = right_cmd * self.max_thrust

        if self.use_wrench:
            wl = Wrench(); wl.force.x = left_N
            wr = Wrench(); wr.force.x = right_N
            self.pub_left.publish(wl)
            self.pub_right.publish(wr)
        else:
            vl = Vector3(); vl.x = left_N
            vr = Vector3(); vr.x = right_N
            self.pub_left.publish(vl)
            self.pub_right.publish(vr)

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
