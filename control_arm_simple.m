% Simple MATLAB script for controlling the USV Robot Arm
% 
% This script sends a single joint command to the robot arm.
% It has been simplified to work with older versions of the MATLAB ROS Toolbox
% that do not support checking for subscribers.

% --- 1. ROS 2 Setup ---
% Set ROS environment variables. This should match your ROS 2 setup.
setenv("ROS_DOMAIN_ID", "30");
setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp");

% Create a ROS 2 node.
% It's good practice to clear old nodes if you run the script multiple times.
clear node;
node = ros2node("matlab_arm_controller_simple");

% Create a publisher for the '/matlab/joint_goals' topic.
topic_name = "/matlab/joint_goals";
pub_joint_goals = ros2publisher(node, topic_name, "std_msgs/Float64MultiArray", ...
    "Reliability", "reliable", "History", "keeplast", "Depth", 1);


% --- 2. Wait for Connection (Simplified) ---
% We pause for a few seconds to give the ROS network time to establish
% the connection between this publisher and the 'matlab_bridge' subscriber.
% This is used because your MATLAB version does not support subscriber checks.
fprintf("Pausing for 3 seconds to allow network connection...\n");
pause(3);


% --- 3. Define and Send Goal ---
% Create a message with the desired joint positions.
msg_goal = ros2message(pub_joint_goals);

% Define the target joint positions in radians.
% The order of joints is:
% 1. '5DOF_V2-v1_Base-v1_Revolute-1'      (Base rotation)
% 2. '5DOF_V2-v1_Base_tube-v1_Revolute-13' (Shoulder)
% 3. '5DOF_V2-v1_Ledd1-v1_Revolute-4'      (Elbow)
% 4. '5DOF_V2-v1_Ledd2-v1_Revolute-15'     (Wrist pitch)
% 5. '5DOF_V2-v1_Ledd-4-v1_Revolute-9'     (Wrist roll)
joint_positions = [0.5, 0.2, 0.5, -0.5, 0.3]; % Example goal

% Assign data to the message and send.
msg_goal.data = joint_positions;
send(pub_joint_goals, msg_goal);

fprintf("Goal sent to the robot arm. Check Gazebo.\n");


% --- 4. Shutdown ---
% Clean up the ROS 2 node and publisher.
clear node pub_joint_goals;
fprintf("Script finished.\n");
