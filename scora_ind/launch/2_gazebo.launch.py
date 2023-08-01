import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    package_dir = get_package_share_directory('scora_ind')
    urdf = os.path.join(package_dir,'scora_gazebo.urdf')
    world_file_name = 'celdamanufactura.world'
    world_path = os.path.join(package_dir, 'worlds', world_file_name)

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        
    return LaunchDescription([
         Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),

#  Gazebo related stuff required to launch the robot in simulation
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen'),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "scora_ind", "-x", "-0.24", "-y", "0.45", "-z", "0.87", "-R", "0", "-P", "0", "-Y", "1.5708"])
    ])