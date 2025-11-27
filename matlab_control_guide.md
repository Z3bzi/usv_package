
# Controlling the USV Arm from MATLAB

This guide explains how to control the robot arm on the USV using MATLAB.

A `matlab_bridge` ROS2 node has been created to simplify the control process. You only need to publish an array of joint positions to the `/matlab/joint_goals` topic.

## 1. Prerequisites

- MATLAB with the ROS Toolbox installed.
- A working ROS2 installation with the `usv_package` built and sourced.
- Your ROS2 and MATLAB environments must be configured to communicate. This usually means ensuring the `ROS_DOMAIN_ID` environment variable is set to the same value in both your ROS2 terminal and in your MATLAB session.

## 2. MATLAB Control Script

The following MATLAB script will send a goal to the arm to move it to a specified set of joint angles.

```matlab
% Connect to the ROS2 network
ros2 node;
clear all;
clc;

% Make sure your ROS_DOMAIN_ID is set correctly, e.g., setenv('ROS_DOMAIN_ID', '30')

% Create a publisher for the /matlab/joint_goals topic
publisher = ros2publisher('/matlab/joint_goals', 'std_msgs/Float64MultiArray');

% Wait for a subscriber to connect
% This ensures the matlab_bridge node is ready to receive messages
disp('Waiting for matlab_bridge subscriber...');
while (publisher.NumSubscribers == 0)
    pause(1); % Wait for 1 second before checking again
end
disp('matlab_bridge subscriber connected.');

% Create a message
msg = ros2message(publisher);

% Set the desired joint positions (in radians)
% IMPORTANT: Change these values to your desired target positions
% The order of joints is:
% 1. '5DOF_V2-v1_Base-v1_Revolute-1'
% 2. '5DOF_V2-v1_Base_tube-v1_Revolute-13'
% 3. '5DOF_V2-v1_Ledd1-v1_Revolute-4'
% 4. '5DOF_V2-v1_Ledd2-v1_Revolute-15'
% 5. '5DOF_V2-v1_Ledd-4-v1_Revolute-9'
msg.data = [0.5, 0.2, 0.5, -0.5, 0.3]; % Example values

% Send the message
disp('Sending joint goals to the arm...');
send(publisher, msg);

```

## 3. How to Use

1.  **Build and source the workspace:** The new `matlab_bridge` node has been added, so you need to rebuild your ROS2 workspace.
    ```bash
    cd /home/rocotics/ros2_ws
    colcon build --packages-select usv_package
    source install/setup.bash
    ```
2.  **Launch the simulation:** In your ROS2 terminal, launch the USV simulation:
    ```bash
    ros2 launch usv_package full_Launch.launch.py
    ```
    This will now also start the `matlab_bridge` node.
3.  **Open MATLAB:** Start MATLAB and make sure the ROS Toolbox is installed.
4.  **Set ROS_DOMAIN_ID:** In the MATLAB command window, set the `ROS_DOMAIN_ID` to match your ROS2 setup. For example:
    ```matlab
    setenv('ROS_DOMAIN_ID', '30')
    ```
5.  **Run the script:** Copy the MATLAB script above into a new `.m` file (e.g., `control_arm_simple.m`) or run it directly in the MATLAB command window.
6.  **Modify the script:** Change the `msg.data` array to the desired joint angles for your application.

This will send the command to the robot arm, and you should see it move in the Gazebo simulation.

