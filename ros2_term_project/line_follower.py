import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .line_tracker import LineTracker
import cv_bridge


class LineFollower(Node):
    def __init__(self, line_tracker: LineTracker):
        super().__init__('line_follower')

        # 선 검출 기능
        self.line_tracker = line_tracker

        # 지정된 차량 정보를 받아올 subscription
        self.car_info_subscription_ = self.create_subscription(
            String,
            'car_info',
            self.car_info_listener_callback,
            10
        )

        # 차량에 전달할 속도 정보를 전달하는 publisher
        self.twist_info_publisher_ = self.create_publisher(
            Twist,
            'twist_info',
            10
        )

        # 주행 상태 정보를 전달할 publisher
        self.drive_issue_publisher_ = self.create_publisher(
            String,
            'drive_issue',
            10
        )

        # 차선 침범 횟수를 전달할 publisher
        self.invasion_info_publisher_ = self.create_publisher(
            String,
            'invasion_info',
            10
        )

        # 이미지 구독을 위한 초기 설정
        self.image_subscription_ = None
        self.bridge = cv_bridge.CvBridge()

        # 차선 침범 정보 퍼블리싱 주기 설정
        self.timer_period = 1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def car_info_listener_callback(self, msg: String):
        car = msg.data
        # 차량이 지정되면 카메라 이미지를 구독
        self.image_subscription_ = self.create_subscription(
            Image, '/demo/' + car + '_front_camera/image_raw',
            self.lane_image_callback, 10
        )

    def timer_callback(self):
        # 차선 침범 횟수 퍼블리싱
        msg = String()
        msg.data = str(self.line_tracker.invasion_count)
        self.invasion_info_publisher_.publish(msg)

    def lane_image_callback(self, image: Image):
        # ROS 이미지를 OpenCV 이미지로 변환
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        # 차선을 감지하고 중앙값을 계산
        self.line_tracker.process(img)

        # Twist 메시지 설정
        twist = Twist()
        msg = String()

        # delta 값을 기준으로 회전 속도 계산
        twist.angular.z = (-1) * self.line_tracker.delta / 200  # 더 부드럽게 회전하도록 비율 조정

        # 회전 속도 제한
        twist.angular.z = max(min(twist.angular.z, 0.7), -0.7)

        # 회전 여부를 판단하여 주행 상태 메시지 생성
        if abs(twist.angular.z) > 0.08:
            msg.data = '회전'
        else:
            msg.data = '직진'

        # 직진과 회전에 따른 속도 조정
        if abs(twist.angular.z) > 0.3:  # 회전 중일 때
            twist.linear.x = 2.5  # 회전 시 속도 감소
        else:
            twist.linear.x = 5.0  # 직진 시 속도 유지

        # 메시지 퍼블리싱
        self.drive_issue_publisher_.publish(msg)
        self.twist_info_publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    # LineTracker 객체 생성
    tracker = LineTracker()

    # LineFollower 노드 실행
    follower = LineFollower(tracker)

    rclpy.spin(follower)

    # 노드 종료
    follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()