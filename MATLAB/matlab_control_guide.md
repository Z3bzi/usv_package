# MATLAB Control Guide for the USV Robot Arm

This guide explains the current, working methods for controlling the 5-DOF robot arm in the simulation.

## Method 1: Trajectory Control with MATLAB Action Client

This is the primary and most robust method for sending complex, multi-point trajectories to the arm. It uses a MATLAB script that acts as a ROS 2 "action client" to send a goal to the arm's controller.

### How it Works
The script sends a `FollowJointTrajectory` goal to the `/arm_controller/follow_joint_trajectory` action server. This goal contains one or more trajectory points, each with a position for all 5 joints and a time-from-start for the arm to reach that point.

### The Script (`control_arm_trajectory.m`)
This script will move the arm through a pre-defined 2-point trajectory.

```matlab
% MATLAB script for sending a trajectory to the USV Robot Arm
clc;
clear;
close all;
fprintf('Starting Robot Arm Trajectory Controller...\n');

%% ---------- 1. ROS 2 SETUP ----------
% Set ROS environment variables
setenv("ROS_DOMAIN_ID", "30");
setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp");

% Create a ROS 2 node
clear node; % Clear any previous nodes
node = ros2node("matlab_trajectory_client");

% Create an Action Client for the arm controller
action_name = '/arm_controller/follow_joint_trajectory';
action_client = ros2actionclient(node, action_name, 'control_msgs/FollowJointTrajectory');
fprintf("Action client created for '%s'.\n", action_name);


%% ---------- 2. WAIT FOR ACTION SERVER ----------
fprintf("Waiting for the action server to be available...\n");
if ~waitForServer(action_client, 'Timeout', 15)
    error("Action server '%s' not available after 15 seconds. Is the simulation running?", action_name);
end
fprintf("Action server is available.\n");


%% ---------- 3. BUILD THE TRAJECTORY GOAL ----------
% Create a goal message
goal_msg = ros2message(action_client);

% --- Define the joint names (must match controller config) ---
goal_msg.trajectory.joint_names = { ...
    'base_rotate', ...
    'shoulder', ...
    'elbow', ...
    'wrist', ...
    'tool' ...
};

% --- Create trajectory points ---
% First point
traj_point_1 = ros2message('trajectory_msgs/JointTrajectoryPoint');
traj_point_1.positions = [0.2, 0.1, 0.2, -0.2, 0.1];
traj_point_1.velocities = []; % Explicitly set to empty
traj_point_1.accelerations = []; % Explicitly set to empty
traj_point_1.effort = []; % Explicitly set to empty
traj_point_1.time_from_start.sec = int32(2); % Reach this point in 2 seconds

% Second point (final goal)
traj_point_2 = ros2message('trajectory_msgs/JointTrajectoryPoint');
traj_point_2.positions = [0.5, 0.2, 0.5, -0.5, 0.3];
traj_point_2.velocities = []; % Explicitly set to empty
traj_point_2.accelerations = []; % Explicitly set to empty
traj_point_2.effort = []; % Explicitly set to empty
traj_point_2.time_from_start.sec = int32(5); % Reach this point in 5 seconds

% --- Add the points to the trajectory ---
goal_msg.trajectory.points = [traj_point_1; traj_point_2];


%% ---------- 4. SEND THE GOAL ----------
fprintf("Sending trajectory goal to the arm controller...\n");
sendGoal(action_client, goal_msg);
fprintf("Trajectory goal sent. Check Gazebo.\n");

```

## Method 2: Interactive Control & Waypoint Generation

For interactively finding desired arm positions, you can use the `twist_gui` that launches with the simulation.

### How it Works
The GUI has sliders for each arm joint. Moving a slider sends a simple, single-point trajectory to the arm controller, allowing you to "jog" the arm into position.

### Generating Waypoints for MATLAB
The GUI provides a simple workflow for creating trajectories:

1.  **Launch the Simulation:** The `twist_gui` window will appear automatically.
2.  **Position the Arm:** Use the 5 arm sliders in the GUI to move the arm to a desired waypoint.
3.  **Save the Position:** Click the **"Save Arm Position"** button. The joint values will be formatted into a MATLAB matrix row and will appear in the text box.
4.  **Repeat:** Move the arm to the next desired waypoint and click "Save Arm Position" again. A new row will be added to the matrix.
5.  **Clear (Optional):** If you want to start a new sequence, click the **"Clear Waypoints"** button.
6.  **Copy to MATLAB:** Once you have defined all your waypoints, you can copy the `waypoints = [...];` matrix from the GUI's text box and paste it directly into a MATLAB script to build a complex trajectory.

```