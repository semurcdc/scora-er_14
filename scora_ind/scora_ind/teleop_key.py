#!/usr/bin/env python3
import os
import select
import sys
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

step_size_J1J2 = 0.03926991  # 2.25°
step_size_J3 = 0.0174533  # 4.2mm 0.05°
step_size_J4 = 0.125664  # 7.2°
step_size_J5J6 = 0.0174533  # 4.2mm 0.05°

msg = """
Control Your Scora-er 14!
---------------------------
Moving around:

  J1 J2 J3 J4 J5 J6
   q  w  e  r  t  y
   a  s  d  f  g  h

        x

q,w,e,r,t,y/a,s,d,f,g,h : increase/decrease the position 1º degree 

space key, x : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(
            JointTrajectory, publish_topic, 10)
        timer_period = 1
        self.joints = ['link_1_to_base_link', 'link_2_to_link_1', 'link_3_to_link_2',
                       'link_4_to_link_3', 'gripper_left_to_link_4', 'gripper_right_to_link_4']
        self.goal_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    a1 = 0.0
    a2 = 0.0
    a3 = 0.0
    a4 = 0.0
    a5 = 0.0
    a6 = 0.0

    scora_trajectory = Trajectory_publisher()
    try:
        print(msg)
        while (1):
            key = get_key(settings)
            if key == 'q':
                a1 = a1 - step_size_J1J2
            if key == 'a':
                a1 = a1 + step_size_J1J2

            if key == 'w':
                a2 = a2 - step_size_J1J2
            if key == 's':
                a2 = a2 + step_size_J1J2

            if key == 'e':
                a3 = a3 - step_size_J3
            if key == 'd':
                a3 = a3 + step_size_J3

            if key == 'r':
                a4 = a4 - step_size_J4
            if key == 'f':
                a4 = a4 + step_size_J4

            if key == 't':
                a5 = a5 - step_size_J5J6
            if key == 'g':
                a5 = a5 + step_size_J5J6

            if key == 'y':
                a6 = a6 - step_size_J5J6
            if key == 'h':
                a6 = a6 + step_size_J5J6

            if (key == '\x03'):
                rclpy.shutdown()
                break

            scora_trajectory.goal_positions = [a1, a2, a3, a4, a5, a6]
            trajectory_msg = JointTrajectory()
            trajectory_msg.joint_names = scora_trajectory.joints
            point = JointTrajectoryPoint()
            point.positions = scora_trajectory.goal_positions
            point.time_from_start = Duration(sec=1)
            trajectory_msg.points.append(point)
            scora_trajectory.trajectory_publihser.publish(trajectory_msg)

    except Exception as e:
        print(e)


if __name__ == '__main__':
    main()
