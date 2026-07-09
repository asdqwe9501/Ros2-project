import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from .end_line_tracker import EndLineTracker
from .stop_line_tracker import StopLineTracker
import cv_bridge
import cv2
import numpy as np


class LineDetector(Node):
    def __init__(self, end_line_tracker: EndLineTracker, stop_line_tracker: StopLineTracker):
        super().__init__('line_detector')

        # 트래커 인스턴스
        self.end_line_tracker = end_line_tracker
        self.stop_line_tracker = stop_line_tracker

        # 차량 정보를 받아올 subscription
        self.car_info_subscription_ = self.create_subscription(
            String,
            'car_info',
            self.car_info_listener_callback,
            10
        )

        # 종료선 감지 결과 퍼블리셔
        self.end_issue_publisher_ = self.create_publisher(
            String,
            'end_issue',
            10
        )

        # 정지선 감지 결과 퍼블리셔
        self.stop_issue_publisher_ = self.create_publisher(
            String,
            'stop_issue',
            10
        )

        # 장애물 감지 결과 퍼블리셔
        self.obstacle_issue_publisher_ = self.create_publisher(
            String,
            'obstacle_issue',
            10
        )

        # 카메라 이미지 구독용 설정
        self.image_subscription_ = None
        self.bridge = cv_bridge.CvBridge()

        # 장애물 감지 상태 플래그
        self.obstacle_detected = False

    def car_info_listener_callback(self, msg: String):
        car = msg.data
        # 차량에 따라 카메라 정보를 받는 subscription 생성
        self.image_subscription_ = self.create_subscription(
            Image,
            '/demo/' + car + '_camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, image: Image):
        # ROS 이미지를 OpenCV 이미지로 변환
        img = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        # 종료선 감지
        self.end_line_tracker.process(img)
        if self.end_line_tracker.delta is not None and self.end_line_tracker.delta < 30:
            end_msg = String()
            end_msg.data = '종료'
            self.end_issue_publisher_.publish(end_msg)

        # 정지선 감지
        self.stop_line_tracker.process(img)
        if self.stop_line_tracker.delta is not None and self.stop_line_tracker.delta < 3.0:
            stop_msg = String()
            stop_msg.data = '정지'
            self.stop_issue_publisher_.publish(stop_msg)
        else:
            stop_msg = String()
            stop_msg.data = ''
            self.stop_issue_publisher_.publish(stop_msg)

        # 장애물 감지
        self.obstacle_detected = self.detect_obstacle(img)
        obstacle_msg = String()
        if self.obstacle_detected:
            obstacle_msg.data = '장애물 감지'
        else:
            obstacle_msg.data = ''
        self.obstacle_issue_publisher_.publish(obstacle_msg)

    def detect_obstacle(self, img):
        """
        도로와 차선을 제외한 영역에서 장애물을 감지
        """
        # HSV 색상 공간으로 변환
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 빨간색 범위 설정 (예시: 빨간색 범위)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # 두 개의 빨간색 마스크 결합
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)

        # 빨간색만 감지된 영역
        obstacle_mask = red_mask

        # 마스크 영역이 일정 크기 이상이면 장애물로 판단
        obstacle_area = cv2.countNonZero(obstacle_mask)
        return obstacle_area > 5000  # 장애물 감지 기준값 (픽셀 개수)


def main(args=None):
    rclpy.init(args=args)

    # 트래커 객체 생성
    end_line_tracker = EndLineTracker()
    stop_line_tracker = StopLineTracker()

    # LineDetector 노드 실행
    node = LineDetector(end_line_tracker, stop_line_tracker)

    rclpy.spin(node)

    # 노드 종료
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()