import rclpy as rp
from rclpy.node import Node

from geometry_msgs.msg import Twist
# Twist: 선형속도와 각속도 정로블 담고 있는 표준 메시지 타입

class TurtlesimPublisher(Node):

    def __init__(self):
        super().__init__('turtlesim_publisher')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 2.0
        self.publisher.publish(msg)


def main(args=None):
    rp.init(args=args)

    turtlesim_publisher = TurtlesimPublisher()
    rp.spin(turtlesim_publisher)

    turtlesim_publisher.destroy_node()
    rp.shutdown()
    

if __name__ == '__main__':
    main()