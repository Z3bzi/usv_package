from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
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
    robot_description = {'robot_description': Command(['xacro', ' ', xacro_file, ' ', '--inorder'])}

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

    return LaunchDescription([use_sim_time, world_arg, x_arg, y_arg, z_arg, Y_arg,
                              gazebo, rsp, delayed_spawner])
