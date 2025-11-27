import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

# Name of the package
package_name = 'usv_package'

def generate_launch_description():
    # Define paths
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'usv_ocean.world')
    xacro_file_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'ele306baat_V4.urdf.xacro')

    # --- 1. Environment and Launch Arguments ---

    # Disable Gazebo's model database to prevent blocking
    disable_model_db = SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value='')

    # Add the buoyancy plugin library to the Gazebo plugin path
    try:
        buoy_share = get_package_share_directory('buoyancy_plugin')
        buoy_prefix = os.path.dirname(os.path.dirname(buoy_share))
        buoy_lib = os.path.join(buoy_prefix, 'lib')
        existing_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
        plugin_path = buoy_lib if not existing_path else f"{buoy_lib}:{existing_path}"
        gazebo_plugin_env = SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=plugin_path)
    except Exception:
        gazebo_plugin_env = None

    # Declare a launch argument to use simulation time
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')


    # --- 2. Core Simulation Nodes ---

    # Start the Gazebo server with the specified world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
        ),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )

    # Process the xacro file to generate the URDF
    robot_description_raw = xacro.process_file(xacro_file_path).toxml()

    # Start the robot_state_publisher, which publishes TF frames for the robot
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time
        }]
    )

    # Spawn the robot entity in Gazebo from the 'robot_description' topic
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'ele306baat',
            '-x', '0.0', '-y', '0.0', '-z', '0.2', '-Y', '0.0'
        ],
        output='screen'
    )


    # --- 3. ROS2 Control Spawners ---

    # Start the joint_state_broadcaster to publish joint states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Start the arm_controller for the robotic arm
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # --- 4. Custom Utility Nodes ---

    # Start the thruster_mixer to convert wrench commands to thruster forces
    mixer = Node(
        package='usv_package',
        executable='thruster_mixer',
        name='thruster_mixer',
        parameters=[{
            'max_thrust': 20.0,
            'turn_mix': 1.0,
            'left_topic': '/usv/left_thrust',
            'right_topic': '/usv/right_thrust',
            'use_wrench': True
        }],
        output='screen'
    )

    # Start the interactive GUI for jogging the robot and arm
    twist_gui = Node(
        package='usv_package',
        executable='twist_gui',
        name='twist_gui',
        output='screen'
    )

    # Start the bridge for communication with the MATLAB boat controller
    boat_matlab_bridge = Node(
        package='usv_package',
        executable='boat_matlab_bridge',
        name='boat_matlab_bridge',
        output='screen'
    )


    # --- 5. Assemble the Launch Description ---
    
    items = [
        sim_time_arg,
        disable_model_db,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        mixer,
        twist_gui,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        boat_matlab_bridge
    ]
    if gazebo_plugin_env:
        items.insert(1, gazebo_plugin_env) # Set environment variable early

    return LaunchDescription(items)

