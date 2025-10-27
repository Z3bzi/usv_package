import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Resolve world path from the package share directory
world_path = os.path.join(get_package_share_directory('usv_package'), 'worlds', 'usv_ocean.world')

def generate_launch_description():
    # Ensure Gazebo can find locally built buoyancy plugin
    try:
        buoy_share = get_package_share_directory('buoyancy_plugin')
        buoy_prefix = os.path.dirname(os.path.dirname(buoy_share))
        buoy_lib = os.path.join(buoy_prefix, 'lib')
        existing = os.environ.get('GAZEBO_PLUGIN_PATH', '')
        value = buoy_lib if not existing else f"{buoy_lib}:{existing}"
        gazebo_plugin_env = SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=value)
        print("AAAAAAAAA:+", buoy_lib)
    except Exception:
        gazebo_plugin_env = None
    # Declaring use_sim_time as a launch argument that can then be used in all launch files
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    #Starting Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )

    actions = [sim_time_arg]
    if gazebo_plugin_env:
        actions.append(gazebo_plugin_env)
    actions.append(gazebo)

    return LaunchDescription(actions)
