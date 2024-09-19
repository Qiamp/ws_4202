import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtlePoseShowNode(Node):
    def __init__(self):
        super().__init__('turtle_pose_show_node')
        self.subscription_1 = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.subscription_2 = self.create_subscription(
            Twist,
            '/turtle1/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def pose_callback(self, msg):
        x = msg.x
        y = msg.y
        theta = msg.theta
        self.get_logger().info(f'Turtle position: x={x}, y={y}, theta={theta}')

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        self.get_logger().info(f'Radius:{linear_x/angular_z}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseShowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
