import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive

class TwistToAckermannNode(Node):
    def __init__(self):
        super().__init__('twist_to_ackermann')
        
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('drive_topic', 'drive')
        
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        drive_topic = self.get_parameter('drive_topic').value
        
        self.subscription = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.twist_callback,
            10)
        self.publisher = self.create_publisher(AckermannDrive, drive_topic, 10)
        
        self.get_logger().info('Twist to Ackermann node initialized')

    def twist_callback(self, msg):
        ackermann_msg = AckermannDrive()
        ackermann_msg.speed = msg.linear.x
        ackermann_msg.steering_angle = msg.angular.z  # TEB已经发布角度，而不是角速度
        
        # 可以根据需要添加一些限制逻辑
        ackermann_msg.speed = max(min(ackermann_msg.speed, 2.0), -2.0)  # 限制速度在 -2 到 2 m/s
        ackermann_msg.steering_angle = max(min(ackermann_msg.steering_angle, 0.7), -0.7)  # 限制转向角在 -0.7 到 0.7 弧度
        
        self.publisher.publish(ackermann_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToAckermannNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()