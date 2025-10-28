import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

package_name = 'usv_package'
world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'usv_ocean.world')

def generate_launch_description():
    # 1) Slå av ekstern modelldatabase (unngå blocking)
    disable_model_db = SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value='')

    # 2) Legg til plugin-bibliotek i GAZEBO_PLUGIN_PATH
    try:
        buoy_share = get_package_share_directory('buoyancy_plugin')
        buoy_prefix = os.path.dirname(os.path.dirname(buoy_share))
        buoy_lib = os.path.join(buoy_prefix, 'lib')
        existing = os.environ.get('GAZEBO_PLUGIN_PATH', '')
        plugin_path = buoy_lib if not existing else f"{buoy_lib}:{existing}"
        gazebo_plugin_env = SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=plugin_path)
    except Exception:
        gazebo_plugin_env = None

    # 3) Launch-argument for sim-tid
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 4) Start Gazebo med world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
        ),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )

    # 5) Prosesser xacro → URDF
    xacro_file_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'ele306baat.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file_path).toxml()

    # 6) Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time
        }]
    )

    # 7) Spawn-innstillinger
    robot_pos = ['0.0', '0.0', '0.2']
    robot_yaw = '0.0'

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'ele306baat',
            '-x', robot_pos[0], '-y', robot_pos[1], '-z', robot_pos[2], '-Y', robot_yaw
        ],
        output='screen'
    )

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

    twist_gui = Node(
        package='usv_package',
        executable='twist_gui',
        name='twist_gui',
        output='screen'
    )

    items = [
        sim_time_arg,
        disable_model_db,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        mixer,
        twist_gui
    ]
    if gazebo_plugin_env:
        items.insert(1, gazebo_plugin_env)  # sett miljøvariabel tidlig

    return LaunchDescription(items)
