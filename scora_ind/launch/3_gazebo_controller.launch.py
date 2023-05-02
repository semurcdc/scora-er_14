import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    package_dir = get_package_share_directory('scora_ind')
    urdf = os.path.join(package_dir,'scora_Joint_trajectory_controller.urdf')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        
    return LaunchDescription([
         Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]
        ),

#  Gazebo related stuff required to launch the robot in simulation
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=["-topic", "robot_description", "-entity", "scora_ind"]),
#   Running the controllers in launch file
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'start','joint_state_broadcaster'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_trajectory_controller'],
            output='screen'
        ),
        Node(
            package='rqt_scora',
            executable='rqt_scora',
            name='rqt_scora',
            output='screen',
           ),
  ])