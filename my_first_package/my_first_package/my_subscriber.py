import rclpy as rp # ROS 2의 Python 클라이언트 라이브러리로, ROS 2 통신을 위한 핵심 기능을 제공
from rclpy.node import Node # ROS 2의 기본 단위인 노드를 생성하는 클래스입니다. 모든 ROS 2 노드 클래스는 이 Node 클래스를 상속받아야 합니다.

from turtlesim.msg import Pose # 패키지에 정의된 메시지 타입입니다. 거북이의 위치와 방향 정보를 담고 있음

# Node 클래스를 상속받아 ROS 2 노드의 기능을 갖게됨
class TurtlesimSubscriber(Node):
    
    # 생성자
    def __init__(self):
        super().__init__('turtlesim_subscriber') # 부모의 생성자로 초기화
        self.subscription = self.create_subscription(
            Pose, # 구독할 메시지 형식
            'turtle1/pose', # 구독할 토픽 이름
            self.callback, # 메시지 수신시 호출될 함수
            10 # 메시지 처리가 늦어질 경우 버퍼에 저장할 메시지 개수
        )
        self.subscription # prevent unused variable warning

    def callback(self, msg):
        print("X: ", msg.x, ", Y: ", msg.y)


def main(args=None):
    rp.init(args=args) # ros 통신은 항상 초기화 하고 써야함

    # TurtlesimSubscriber 클래스의 인스턴스를 생성하여 노드를 만듭니다.
    turtlesim_subscriber = TurtlesimSubscriber()

    # 노드가 계속 실행되도록 함,
    rp.spin(turtlesim_subscriber)

    # 스핀 종료시 노드 소멸시킴
    turtlesim_subscriber.destroy_node()

    # ROS2 통신 종료
    rp.shutdown()


if __name__ == '__main__':
    main()