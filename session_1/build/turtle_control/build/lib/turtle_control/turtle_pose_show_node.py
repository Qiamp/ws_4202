import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtlePoseShowNode(Node):
    def __init__(self):
        super().__init__('turtle_pose_show_node')
        self.pose = None  # Store the latest pose
        self.twist = None  # Store the latest twist

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
        self.pose = msg  # Update the latest pose
        self.publish_info()  # Call publish info to log combined message

    def cmd_vel_callback(self, msg):
        self.twist = msg  # Update the latest twist
        self.publish_info()  # Call publish info to log combined message

    def publish_info(self):
        if self.pose and self.twist:  # Ensure both pose and twist have been received
            x = self.pose.x
            y = self.pose.y
            theta = self.pose.theta
            linear_x = self.twist.linear.x
            angular_z = self.twist.angular.z
            radius = linear_x/angular_z

            # Print both pose and radius information
            self.get_logger().info(f'Turtle position: x={x}, y={y}, theta={theta}, Radius:{radius}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseShowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
