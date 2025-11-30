% MATLAB script for sending a trajectory to the USV Robot Arm
% This script was refactored to use a loop for waypoint generation.
clc;
clear;
close all;

fprintf('Starting Robot Arm Trajectory Controller...\n');

%% Configuration
% Adjust these settings as needed.

% Time (in seconds) between each waypoint
time_step = 2.0; 

% Joint Names (must match your controller config)
joint_names = { ...
    'base_rotate', ...
    'shoulder', ...
    'elbow', ...
    'wrist', ...
    'tool' ...
};

% Waypoints Matrix
% Each row is a time step, each column is a joint angle.
% Columns: [Base, Tube, Ledd1, Ledd2, Ledd4]
waypoints_wide = [
    -1.610,  1.130, -1.840,  1.040, -0.830;   % Point 1
     0.050,  1.130, -1.840,  1.040, -0.830;   % Point 2
     1.350,  1.130, -1.840,  1.040, -0.830;   % Point 3
     1.590,  1.130, -1.840,  1.040,  0.230;   % Point 4
     3.240,  0.480, -2.230,  1.290,  0.010;   % Point 5
     3.140,  0.580, -2.230,  1.290,  0.010;   % Point 6
     3.140,  0.580, -2.230,  1.240, -3.140;   % Point 7
];


waypoints_specific = [
    0.000, 1.110, -1.330, -0.090, 0.000;  % Point 1
    0.000, 1.110, -1.180,  0.620, 0.000;  % Point 2
    0.000, 0.700, -1.960,  1.200, 0.090;  % Point 3
    3.140, 0.480, -2.260,  1.200, 0.090;  % Point 4
    3.140, 0.480, -2.260,  1.200, 3.140   % Point 5
];

waypoints = waypoints_wide;

%% ROS 2 Setup
% Configure ROS environment variables
setenv("ROS_DOMAIN_ID", "30");
setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp");

% Create a ROS 2 node for this script
clear node; 
node = ros2node("matlab_trajectory_client");

% Set up the Action Client for the arm controller
action_name = '/arm_controller/follow_joint_trajectory';
action_client = ros2actionclient(node, action_name, 'control_msgs/FollowJointTrajectory');
fprintf("Action client ready for '%s'.\n", action_name);

%% Wait for Action Server
fprintf("Waiting for the arm controller action server to become available...\n");
if ~waitForServer(action_client, 'Timeout', 15)
    error("Action server '%s' not available. Is the simulation or robot arm controller running?", action_name);
end
fprintf("Action server is now active.\n");

%% Build the Trajectory Goal
goal_msg = ros2message(action_client);
goal_msg.trajectory.joint_names = joint_names;

% Prepare an array of trajectory points
num_points = size(waypoints, 1);
trajectory_points = repmat(ros2message('trajectory_msgs/JointTrajectoryPoint'), num_points, 1);

fprintf("Constructing a trajectory with %d points...\n", num_points);

% Populate trajectory points from waypoints
for i = 1:num_points
    current_pos = waypoints(i, :); % Get current joint positions from the matrix
    
    trajectory_points(i).positions = current_pos;
    
    % Controller handles velocities/accelerations, so we leave these empty
    trajectory_points(i).velocities = [];
    trajectory_points(i).accelerations = [];
    trajectory_points(i).effort = [];
    
    % Calculate the time from the start for this point
    % Example: if i=1 and time_step=2s, time_from_start will be 2s.
    current_time = time_step * i;
    
    % Convert time to seconds and nanoseconds
    trajectory_points(i).time_from_start.sec = int32(floor(current_time));
    trajectory_points(i).time_from_start.nanosec = uint32((current_time - floor(current_time)) * 1e9);
end

% Assign the generated points to the goal message
goal_msg.trajectory.points = trajectory_points;

%% Send the Trajectory Goal
fprintf("Sending the trajectory goal to the arm controller...\n");
sendGoal(action_client, goal_msg);
fprintf("Trajectory goal successfully sent!\n");

%% Shutdown
% Give a brief moment for messages to send before clearing
pause(1); 
clear node action_client;
fprintf("Script finished executing.\n");