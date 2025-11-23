import rclpy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tkinter import Tk, Scale, HORIZONTAL


class TwistGui:
    """Simple Tk GUI that publishes geometry_msgs/Twist on /cmd_vel."""

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('twist_gui')
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.node.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10,
        )

        self.root = Tk()
        self.root.title('CmdVel GUI')

        self.linear = Scale(
            self.root,
            from_=-5.0,
            to=5.0,
            resolution=0.01,
            orient=HORIZONTAL,
            label='Linear X',
            command=self.update_base,
        )
        self.angular = Scale(
            self.root,
            from_=-2.0,
            to=2.0,
            resolution=0.01,
            orient=HORIZONTAL,
            label='Angular Z',
            command=self.update_base,
        )
        self.linear.pack(fill='x')
        self.angular.pack(fill='x')

        # Arm joint sliders (values in radians)
        self.arm_joints = [
            ('5DOF_V2-v1_Base-v1_Revolute-1', (-3.14, 3.14)),
            ('5DOF_V2-v1_Base_tube-v1_Revolute-13', (-2.0, 1.6)),
            ('5DOF_V2-v1_Ledd1-v1_Revolute-4', (-3.14, 3.14)),
            ('5DOF_V2-v1_Ledd2-v1_Revolute-15', (-3.14, 3.14)),
            ('5DOF_V2-v1_Ledd-4-v1_Revolute-9', (-3.14, 3.14)),
        ]
        self.arm_scales = []
        for joint_name, (lo, hi) in self.arm_joints:
            slider = Scale(
                self.root,
                from_=lo,
                to=hi,
                resolution=0.01,
                orient=HORIZONTAL,
                label=joint_name,
                command=self.update_arm,
            )
            slider.pack(fill='x')
            self.arm_scales.append(slider)

    def update_base(self, _):
        twist = Twist()
        twist.linear.x = self.linear.get()
        twist.angular.z = self.angular.get()
        self.pub.publish(twist)

    def update_arm(self, _):
        traj = JointTrajectory()
        traj.joint_names = [name for name, _ in self.arm_joints]

        point = JointTrajectoryPoint()
        point.positions = [slider.get() for slider in self.arm_scales]
        # Short execution time keeps controller happy while sliding
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(0.3 * 1e9)

        traj.points.append(point)
        self.arm_pub.publish(traj)

    def run(self):
        try:
            self.root.mainloop()
        finally:
            rclpy.shutdown()


def main():
    gui = TwistGui()
    gui.run()
