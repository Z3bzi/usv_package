import rclpy
from geometry_msgs.msg import Twist
from tkinter import Tk, Scale, HORIZONTAL


class TwistGui:
    """Simple Tk GUI that publishes geometry_msgs/Twist on /cmd_vel."""

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('twist_gui')
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        self.root = Tk()
        self.root.title('CmdVel GUI')

        self.linear = Scale(
            self.root,
            from_=-1.0,
            to=1.0,
            resolution=0.01,
            orient=HORIZONTAL,
            label='Linear X',
            command=self.update,
        )
        self.angular = Scale(
            self.root,
            from_=-1.0,
            to=1.0,
            resolution=0.01,
            orient=HORIZONTAL,
            label='Angular Z',
            command=self.update,
        )
        self.linear.pack(fill='x')
        self.angular.pack(fill='x')

    def update(self, _):
        twist = Twist()
        twist.linear.x = self.linear.get()
        twist.angular.z = self.angular.get()
        self.pub.publish(twist)

    def run(self):
        try:
            self.root.mainloop()
        finally:
            rclpy.shutdown()


def main():
    gui = TwistGui()
    gui.run()
