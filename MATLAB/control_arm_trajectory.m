% MATLAB script for sending a trajectory to the USV Robot Arm
% Refactored to use a loop for waypoint generation.
clc;
clear;
close all;

fprintf('Starting Robot Arm Trajectory Controller...\n');

%% ---------- 1. CONFIGURATION (EDIT HERE) ----------

% Define the time (in seconds) between each waypoint
time_step = 2.0; 

% Define the Joint Names (Must match your controller config)
joint_names = { ...
    'base_rotate', ...
    'shoulder', ...
    'elbow', ...
    'wrist', ...
    'tool' ...
};

% Define Waypoints Matrix 
% Each ROW is a time step. Each COLUMN is a joint angle.
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

%% ---------- 2. ROS 2 SETUP ----------
% Set ROS environment variables
setenv("ROS_DOMAIN_ID", "30");
setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp");

% Create a ROS 2 node
clear node; 
node = ros2node("matlab_trajectory_client");

% Create an Action Client
action_name = '/arm_controller/follow_joint_trajectory';
action_client = ros2actionclient(node, action_name, 'control_msgs/FollowJointTrajectory');
fprintf("Action client created for '%s'.\n", action_name);

%% ---------- 3. WAIT FOR ACTION SERVER ----------
fprintf("Waiting for the action server...\n");
if ~waitForServer(action_client, 'Timeout', 15)
    error("Action server '%s' not available. Is the simulation running?", action_name);
end
fprintf("Action server is available.\n");

%% ---------- 4. BUILD THE TRAJECTORY GOAL ----------
goal_msg = ros2message(action_client);
goal_msg.trajectory.joint_names = joint_names;

% Initialize an array of messages for the points
num_points = size(waypoints, 1);
trajectory_points = repmat(ros2message('trajectory_msgs/JointTrajectoryPoint'), num_points, 1);

fprintf("Building trajectory with %d points...\n", num_points);

% Loop through the waypoints matrix to create trajectory points
for i = 1:num_points
    % Extract position row from matrix
    current_pos = waypoints(i, :);
    
    % Assign positions to the message
    trajectory_points(i).positions = current_pos;
    
    % Leave velocities/accelerations empty (controller will handle interpolation)
    trajectory_points(i).velocities = [];
    trajectory_points(i).accelerations = [];
    trajectory_points(i).effort = [];
    
    % Calculate time_from_start
    % If i=1, time is 2s. If i=2, time is 4s, etc.
    current_time = time_step * i;
    
    % Handle seconds and nanoseconds for precision
    trajectory_points(i).time_from_start.sec = int32(floor(current_time));
    trajectory_points(i).time_from_start.nanosec = uint32((current_time - floor(current_time)) * 1e9);
end

% Assign the array of points to the goal message
goal_msg.trajectory.points = trajectory_points;

%% ---------- 5. SEND THE GOAL ----------
fprintf("Sending trajectory goal...\n");
sendGoal(action_client, goal_msg);
fprintf("Trajectory goal sent successfully.\n");

%% ---------- 6. SHUTDOWN ----------
% Pause briefly to ensure message leaves the buffer before clearing node
pause(1); 
clear node action_client;
fprintf("Script finished.\n");