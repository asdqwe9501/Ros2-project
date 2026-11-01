import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from custom_interface.msg import Target
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # 출발/정지/종료처럼 time.sleep 으로 시간이 걸리는 콜백은 별도 그룹으로 분리해
        # 실행 중에도 twist_info 등 나머지 콜백이 처리되도록 함
        self.maneuver_group = MutuallyExclusiveCallbackGroup()

        # 지정된 차량 정보를 받아올 subscription
        self.car_info_subscription_ = self.create_subscription(
            Target,
            'start_car',
            self.car_info_listener_callback,
            10,
            callback_group=self.maneuver_group)

        # 카메라, 라이다 정보를 처리하는 노드에 차량 정보를 전달하는 publisher
        self.car_info_publisher = self.create_publisher(
            String,
            'car_info',
            10
        )

        # line_follower 노드로부터 차량에 전달할 속도 정보를 받아오는 subscription
        self.twist_subscription = self.create_subscription(
            Twist,
            'twist_info',
            self.twist_listener_callback,
            10
        )

        # line_follower 노드로부터 차량 상태(직진, 회전) 정보를 받아오는 subscription
        self.drive_issue_subscription = self.create_subscription(
            String,
            'drive_issue',
            self.drive_issue_listener_callback,
            10
        )

        # 정지선 검출 정보를 받아오는 subscription
        self.stop_issue_subscription = self.create_subscription(
            String,
            'stop_issue',
            self.stop_issue_listener_callback,
            10,
            callback_group=self.maneuver_group
        )

        # 종료 지점 검출 정보를 받아오는 subscription
        self.end_issue_subscription = self.create_subscription(
            String,
            'end_issue',
            self.end_issue_listener_callback,
            10,
            callback_group=self.maneuver_group
        )

        # 초기화
        self.twist_publisher_ = None

        # 회전 여부
        self.turn = False
        # 정지 여부
        self.stop = False
        # 정지선 종류 구분
        self.count = 0

    def car_info_listener_callback(self, msg: Target):
        # 지정 차량
        car = msg.car
        self.get_logger().info('I heard: "%s"' % msg.car)

        time.sleep(5)

        # 차량에 속도 정보를 전달할 publisher
        self.twist_publisher_ = self.create_publisher(Twist, '/demo/' + car + '_cmd_demo', 10)

        # 차량 출발
        twist = Twist()
        for i in range(200):
            twist.linear.x = 3.0  # 초기 속도 낮게 설정
            self.twist_publisher_.publish(twist)
            time.sleep(0.01)

        # 카메라, 라이다 정보를 처리하는 노드들에 차량 정보 전달
        car_info_msg = String()
        car_info_msg.data = car
        self.car_info_publisher.publish(car_info_msg)

    def twist_listener_callback(self, twist: Twist):
        # 차량이 아직 지정되지 않았으면 무시
        if self.twist_publisher_ is None:
            return
        # 차선을 따라 주행하기 위한 속도 정보 전달
        if not self.stop:
            self.twist_publisher_.publish(twist)

    def drive_issue_listener_callback(self, msg: String):
        if msg.data == '직진':
            self.turn = False
        elif msg.data == '회전':
            self.turn = True

    def stop_issue_listener_callback(self, msg: String):
        # 차량이 아직 지정되지 않았으면 무시
        if self.twist_publisher_ is None:
            return

        # 언덕 정지선
        if msg.data == '정지' and not self.turn and not self.stop and self.count == 1:
            self.stop = True
            self.count += 1

            twist = Twist()

            # 차량이 밀리지 않도록
            for i in range(3500):
                twist.linear.x = 1.3
                self.twist_publisher_.publish(twist)
                time.sleep(0.001)

            # 차량 출발 (출발이 끝날 때까지 line_follower 속도 명령이 끼어들지 않도록 유지)
            for i in range(2000):
                twist.linear.x = 6.0
                self.twist_publisher_.publish(twist)
                time.sleep(0.001)

            self.stop = False

        # 평지 정지선
        if msg.data == '정지' and not self.turn and not self.stop and self.count == 0:
            self.stop = True
            self.count += 1
            twist = Twist()

            # 3초간 정지
            for i in range(400):
                twist.linear.x = 0.0
                self.twist_publisher_.publish(twist)
                time.sleep(0.01)

            # 차량 출발
            for i in range(200):
                twist.linear.x = 6.0
                self.twist_publisher_.publish(twist)
                time.sleep(0.01)

            self.stop = False

    def end_issue_listener_callback(self, msg: String):
        # 차량이 아직 지정되지 않았으면 무시
        if self.twist_publisher_ is None:
            return

        if msg.data == '종료':
            self.stop = True
            self.get_logger().info('종료 지점 도달: 차량 정지')

            twist = Twist()
            for i in range(100):
                twist.linear.x = 0.0
                self.twist_publisher_.publish(twist)
                time.sleep(0.01)

            # 콜백 안에서 노드를 직접 파괴하는 대신 spin 루프를 빠져나가 종료
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    executor = MultiThreadedExecutor()
    executor.add_node(controller)

    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        pass

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()