import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleControlNode(Node):
    def __init__(self):
        super().__init__('turtle_control_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_velocity)  # 2 Hz
        self.radius = 2.0
        self.angular_speed = 0.5  # 设置角速度为0.5 rad/s
        self.linear_speed = self.radius * self.angular_speed

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.linear_speed  # 沿x轴的线速度
        msg.angular.z = self.angular_speed  # 绕z轴的角速度
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing velocity: linear={self.linear_speed}, angular={self.angular_speed}, radius={self.radius}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
