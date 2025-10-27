import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ensure Gazebo can find locally built buoyancy plugin
    try:
        buoy_share = get_package_share_directory('buoyancy_plugin')
        buoy_prefix = os.path.dirname(os.path.dirname(buoy_share))
        buoy_lib = os.path.join(buoy_prefix, 'lib')
        gazebo_plugin_env = SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH',
            value=f"{buoy_lib}:" + os.environ.get('GAZEBO_PLUGIN_PATH', '')
        )
    except Exception:
        gazebo_plugin_env = None
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([FindPackageShare('usv_package'), 'worlds', 'usv_ocean.world'])
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.2')
    Y_arg = DeclareLaunchArgument('Y', default_value='0.0')

    world_path = LaunchConfiguration('world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py']),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )

    xacro_file = PathJoinSubstitution([FindPackageShare('usv_package'), 'urdf', 'robot_description.urdf.xacro'])
    robot_description = {'robot_description': Command(['xacro', xacro_file, '--inorder'])}

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    spawner = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic','robot_description','-entity','usv',
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'),
                   '-Y', LaunchConfiguration('Y')],
        output='screen'
    )

    # 2 s delay så /spawn_entity er oppe
    delayed_spawner = TimerAction(period=2.0, actions=[spawner])

    actions = [use_sim_time, world_arg, x_arg, y_arg, z_arg, Y_arg]
    if gazebo_plugin_env:
        actions.append(gazebo_plugin_env)
    actions += [gazebo, rsp, delayed_spawner]

    return LaunchDescription(actions)
