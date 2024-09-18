import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtlePoseShowNode(Node):
    def __init__(self):
        super().__init__('turtle_pose_show_node')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.radius = 2.0

    def pose_callback(self, msg):
        x = msg.x
        y = msg.y
        theta = msg.theta
        self.get_logger().info(f'Turtle position: x={x}, y={y}, theta={theta}, following circle with radius={self.radius}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseShowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
