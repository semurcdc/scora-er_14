
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
class TrajectorySubscriber(Node):
    def __init__(self):
        super().__init__('trajectory_subscriber_node')
        self.subscription = self.create_subscription(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', self.callback, 10)
        self.subscription  # para evitar que el objeto sea eliminado por el recolector de basura
    def callback(self, msg):
        # Acceder a los valores de las posiciones
        positions = msg.points[0].positions
        velocities = msg.points[0].velocities
        accelerations = msg.points[0].accelerations
        # Imprimir los valores de las posiciones
        print(f'Positions: {positions}')
        print(f'Velocities: {velocities}')
        print(f'Accelerations: {accelerations}')
def main(args=None):
    rclpy.init(args=args)
    trajectory_subscriber = TrajectorySubscriber()
    rclpy.spin(trajectory_subscriber)
    trajectory_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
