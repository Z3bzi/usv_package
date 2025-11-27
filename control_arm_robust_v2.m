% Simple and Robust MATLAB script for controlling the USV Robot Arm
%
% This script sends joint commands to the robot arm via the 'matlab_bridge'.
% It includes essential ROS2 setup and a robust check to ensure the
% 'matlab_bridge' node is connected before sending commands.

% Set ROS environment variables. This should match your ROS 2 setup.
setenv("ROS_DOMAIN_ID", "30");
setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp");

% Create a ROS 2 node
node = ros2node("matlab_arm_controller_simple");

% Create a publisher for the '/matlab/joint_goals' topic
topic_name = "/matlab/joint_goals";
pub_joint_goals = ros2publisher(node, topic_name, "std_msgs/Float64MultiArray", ...
    "Reliability", "reliable", "History", "keeplast", "Depth", 1);

% Wait for the 'matlab_bridge' node to connect
wait_time = 0;
max_wait_time = 15; % seconds
while (getSubscriberCount(node, topic_name) == 0)
    if wait_time > max_wait_time
        error("No subscribers connected to '%s' after %d seconds. Is the 'matlab_bridge' node running and accessible?", topic_name, max_wait_time);
    end
    pause(1); % Wait for 1 second
    wait_time = wait_time + 1;
end

% Create a message with the desired joint positions.
msg_goal = ros2message(pub_joint_goals);

% Define the target joint positions in radians.
% Modify these values to move the arm to a different position.
% The order of joints is:
% 1. '5DOF_V2-v1_Base-v1_Revolute-1'      (Base rotation)
% 2. '5DOF_V2-v1_Base_tube-v1_Revolute-13' (Shoulder)
% 3. '5DOF_V2-v1_Ledd1-v1_Revolute-4'      (Elbow)
% 4. '5DOF_V2-v1_Ledd2-v1_Revolute-15'     (Wrist pitch)
% 5. '5DOF_V2-v1_Ledd-4-v1_Revolute-9'     (Wrist roll)
joint_positions = [0.5, 0.2, 0.5, -0.5, 0.3]; % Example goal

% Assign data to the message and send
msg_goal.data = joint_positions;
pause(0.5); % Small pause before sending
send(pub_joint_goals, msg_goal);

% Clean up the ROS 2 node
clear node pub_joint_goals;


%% ---------- HELPER FUNCTION ----------
function count = getSubscriberCount(node, topic_name)
    % This function checks the number of subscribers on a given topic
    % by querying the ROS2 network.
    
    % Get detailed info for the specific topic
    try
        info_str = ros2("topic", "info", topic_name);
        % Find the line with "Subscription count:"
        lines = splitlines(info_str);
        sub_line_idx = contains(lines, "Subscription count:");
        
        if any(sub_line_idx)
            sub_line = lines(sub_line_idx);
            % Extract the number
            parts = split(sub_line, ':');
            count = str2double(strtrim(parts{2}));
        else
            count = 0;
        end
    catch
        % If the command fails for any reason, assume 0 subscribers
        count = 0;
    end
end