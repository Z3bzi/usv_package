import rclpy
from geometry_msgs.msg import WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tkinter import Tk, Scale, HORIZONTAL, Button, Text, END


class TwistGui:
    """
    Simple Tk GUI that publishes geometry_msgs/WrenchStamped on /cmd_wrench
    and allows saving a sequence of arm joint positions for MATLAB.
    """

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('twist_gui')
        self.pub = self.node.create_publisher(WrenchStamped, '/cmd_wrench', 10)
        self.arm_pub = self.node.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10,
        )

        self.root = Tk()
        self.root.title('CmdWrench & Arm GUI')
        self.saved_waypoints = []

        self.linear = Scale(
            self.root,
            from_=-5.0,
            to=5.0,
            resolution=0.01,
            orient=HORIZONTAL,
            label='Force X',
            command=self.update_base,
        )
        self.angular = Scale(
            self.root,
            from_=-2.0,
            to=2.0,
            resolution=0.01,
            orient=HORIZONTAL,
            label='Torque Z',
            command=self.update_base,
        )
        self.linear.pack(fill='x')
        self.angular.pack(fill='x')

        # Arm joint sliders (values in radians)
        self.arm_joints = [
            ('base_rotate', (-3.14, 3.14)),
            ('shoulder', (-2.0, 1.6)),
            ('elbow', (-3.14, 3.14)),
            ('wrist', (-3.14, 3.14)),
            ('tool', (-3.14, 3.14)),
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
                length=400,  # Make sliders wider
                command=self.update_arm,
            )
            slider.pack(fill='x')
            self.arm_scales.append(slider)

        # --- New widgets for saving position sequence ---
        self.save_button = Button(
            self.root,
            text="Save Arm Position",
            command=self.save_arm_position
        )
        self.save_button.pack(pady=(10, 0))
        
        self.clear_button = Button(
            self.root,
            text="Clear Waypoints",
            command=self.clear_waypoints
        )
        self.clear_button.pack()

        self.position_text = Text(
            self.root,
            height=10, # Adjust height for multiple lines
            width=80   # Make text box wider
        )
        self.position_text.pack(pady=5)
        self.position_text.insert(END, "Click 'Save Arm Position' to record waypoints here...")
        self.position_text.config(state='disabled')


    def save_arm_position(self):
        """Saves the current slider positions to the text box as a MATLAB matrix."""
        current_positions = [slider.get() for slider in self.arm_scales]
        self.saved_waypoints.append(current_positions)
        
        # Build the MATLAB matrix string
        output_str = "waypoints = [\n"
        for i, point in enumerate(self.saved_waypoints):
            pos_str = ", ".join([f"{p:.3f}" for p in point])
            output_str += f"    {pos_str};  % Point {i+1}\n"
        output_str += "];"
        
        # Update the text widget
        self.position_text.config(state='normal')
        self.position_text.delete('1.0', END)
        self.position_text.insert(END, output_str)
        self.position_text.config(state='disabled')

    def clear_waypoints(self):
        """Clears the saved waypoints and the text box."""
        self.saved_waypoints = []
        self.position_text.config(state='normal')
        self.position_text.delete('1.0', END)
        self.position_text.insert(END, "Waypoints cleared. Click 'Save Arm Position' to record new waypoints...")
        self.position_text.config(state='disabled')

    def update_base(self, _):
        msg = WrenchStamped()
        msg.wrench.force.x = self.linear.get()
        msg.wrench.torque.z = self.angular.get()
        self.pub.publish(msg)

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
