import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(
            JointTrajectory, publish_topic, 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['link_1_to_base_link', 'link_2_to_link_1',
                       'link_3_to_link_2', 'link_4_to_link_3',
                       'gripper_left_to_link_4', 'gripper_right_to_link_4']
        self.goal_positions = [-0.5, 0.5, 0.5, 0.5, 0.3, 0.1]

    def timer_callback(self):
        scora_trajectory_msg = JointTrajectory()
        scora_trajectory_msg.joint_names = self.joints
        # creating a point
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=2)
        # adding newly created point into trajectory message
        scora_trajectory_msg.points.append(point)
        self.trajectory_publihser.publish(scora_trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()
    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
