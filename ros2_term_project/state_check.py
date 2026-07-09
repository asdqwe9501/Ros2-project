import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time


class StateCheck(Node):

    def __init__(self):
        super().__init__('state_check')

        # 지정된 차량 정보를 받아올 subscription
        self.car_info_subscription_ = self.create_subscription(
            String,
            'car_info',
            self.car_info_listener_callback,
            10
        )

        # 차선 침범 횟수를 받아오는 subscription
        self.invasion_info_subscription_ = self.create_subscription(
            String,
            'invasion_info',
            self.invasion_info_listener_callback,
            10
        )

        # 속도 로그 출력 주기 제한용 타임스탬프
        self.last_vel_log_time = 0.0

    def car_info_listener_callback(self, msg: String):
        car = msg.data
        # 차량이 지정되면 속도 정보를 받는 subscription 생성
        self.vel_info_subcription = self.create_subscription(
            Twist,
            '/demo/' + car + '_cmd_demo',
            self.vel_info_listener_callback,
            10
        )

    def vel_info_listener_callback(self, twist: Twist):
        # 콜백 안에서 sleep 으로 실행기를 멈추지 않고, 1초에 한 번만 로그 출력
        now = time.monotonic()
        if now - self.last_vel_log_time < 1.0:
            return
        self.last_vel_log_time = now
        self.get_logger().info('선 속도: %f' % twist.linear.x)

    def invasion_info_listener_callback(self, msg: String):
        self.get_logger().info('차선 침범: %s' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    state_check = StateCheck()

    rclpy.spin(state_check)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state_check.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()