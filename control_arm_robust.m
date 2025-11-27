% Robust MATLAB script for controlling the USV Robot Arm
% 
% This script uses the structure and connection-checking inspired by the
% boat control script to provide a more reliable way to send joint
% commands to the robot arm via the 'matlab_bridge'.

clc;
clear;
close all;
fprintf('Starting Robot Arm Controller for Gazebo...\n');

%% ---------- 1. ROS 2 SETUP ----------
% Set ROS environment variables. This should match your ROS 2 setup.
setenv("ROS_DOMAIN_ID", "30");
setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp");

% Check if the ROS 2 network is reachable by listing topics
try
    topic_list = ros2("topic", "list");
    fprintf('Successfully connected to ROS 2 network.\n');
catch ME
    error("Failed to list ROS 2 topics. Is the ROS 2 environment sourced and running? Error: %s", ME.message);
end

% Create a ROS 2 node
node = ros2node("matlab_arm_controller");

% Create a publisher for the '/matlab/joint_goals' topic
% The 'matlab_bridge' node subscribes to this topic.
pub_joint_goals = ros2publisher(node, ...
    "/matlab/joint_goals", ...
    "std_msgs/Float64MultiArray", ...
    "Reliability", "reliable", ...
    "History", "keeplast", ...
    "Depth", 1);

fprintf("Publisher created for '/matlab/joint_goals'.\n");


%% ---------- 2. WAIT FOR CONNECTION ----------
% It's crucial to wait for the matlab_bridge to subscribe before sending
% a message. Otherwise, the message might be lost.

fprintf("Waiting for the 'matlab_bridge' node to connect...\n");
wait_time = 0;
max_wait_time = 15; % seconds

while (pub_joint_goals.NumSubscribers == 0)
    if wait_time > max_wait_time
        error("No subscribers connected to '/matlab/joint_goals' after %d seconds. Is the 'matlab_bridge' node running?", max_wait_time);
    end
    fprintf("Still waiting for a subscriber...\n");
    pause(1); % Wait for 1 second
    wait_time = wait_time + 1;
end

fprintf("Connection established with 'matlab_bridge'!\n");


%% ---------- 3. DEFINE AND SEND GOAL ----------
% Create a message with the desired joint positions.
msg_goal = ros2message(pub_joint_goals);

% --- TUNING PARAMETERS ---
% Define the target joint positions in radians.
% Modify these values to move the arm to a different position.
% The order of joints is:
% 1. '5DOF_V2-v1_Base-v1_Revolute-1'      (Base rotation)
% 2. '5DOF_V2-v1_Base_tube-v1_Revolute-13' (Shoulder)
% 3. '5DOF_V2-v1_Ledd1-v1_Revolute-4'      (Elbow)
% 4. '5DOF_V2-v1_Ledd2-v1_Revolute-15'     (Wrist pitch)
% 5. '5DOF_V2-v1_Ledd-4-v1_Revolute-9'     (Wrist roll)

joint_positions = [0.5, 0.2, 0.5, -0.5, 0.3]; % Example goal

% --- Assign data to the message ---
msg_goal.data = joint_positions;

% --- Send the command ---
send(pub_joint_goals, msg_goal);

fprintf("\nGoal sent to the robot arm with positions:\n");
disp(joint_positions);
fprintf("\nCheck Gazebo to see if the arm has moved.\n");

%% ---------- 4. SHUTDOWN ----------
% Clean up the ROS 2 node
clear node pub_joint_goals;
fprintf("\nScript finished.\n");
