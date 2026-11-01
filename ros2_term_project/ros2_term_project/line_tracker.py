import cv2
import numpy as np
import time


class LineTracker:
    def __init__(self):
        self._delta = 0.0  # 차량 중심과 차선 중심의 거리
        self._invasion = 0  # 차선 침범 횟수 초기화
        self._invading = False  # 현재 차선 침범 상태 여부
        self.prev_center = None  # 이전 중앙값
        self.prev_right_cx = None  # 이전 오른쪽 차선 중앙값
        self.prev_left_cx = None  # 이전 왼쪽 차선 중앙값

    def process(self, img: np.ndarray) -> None:
        h, w, _ = img.shape

        # 흰색 차선 마스크 생성
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 25, 255])
        msk = cv2.inRange(hsv, lower_white, upper_white)

        # 동적 ROI 설정 (상단 1/3, 하단 5/6)
        roi_top = int(h * 1 / 3)
        roi_bottom = int(h * 5 / 6)
        roi_mask = np.zeros_like(msk)
        roi_mask[roi_top:roi_bottom, :] = 1
        msk = cv2.bitwise_and(msk, msk, mask=roi_mask)

        # Hough Transform으로 차선 감지
        lines = cv2.HoughLinesP(msk, 1, np.pi / 180, threshold=90, minLineLength=60, maxLineGap=40)

        left_line_xs = []
        right_line_xs = []

        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    if x1 < w / 2 and x2 < w / 2:  # 왼쪽 차선
                        left_line_xs.extend([x1, x2])
                    elif x1 > w / 2 and x2 > w / 2:  # 오른쪽 차선
                        right_line_xs.extend([x1, x2])

        # 차선 중앙 계산
        left_cx = np.mean(left_line_xs) if left_line_xs else self.prev_left_cx
        right_cx = np.mean(right_line_xs) if right_line_xs else self.prev_right_cx
        lane_center = None

        if left_cx is not None and right_cx is not None:
            # 두 차선 모두 감지된 경우
            lane_center = (left_cx + right_cx) / 2
            self.prev_left_cx = left_cx
            self.prev_right_cx = right_cx
        elif right_cx is not None:
            # 오른쪽 차선만 감지된 경우
            lane_center = right_cx - (right_cx - w / 2) / 2
            self.prev_right_cx = right_cx
        elif left_cx is not None:
            # 왼쪽 차선만 감지된 경우
            lane_center = left_cx + (w / 2 - left_cx) / 2
            self.prev_left_cx = left_cx
        else:
            # 차선이 전혀 감지되지 않을 경우
            lane_center = self.prev_center if self.prev_center is not None else w / 2

        # 이전 중앙값 업데이트
        self.prev_center = lane_center

        # delta 계산
        self._delta = lane_center - w / 2

        # 차선 침범 판단 (해상도 비율 기반)
        threshold = 0.1 * w  # 이미지 너비의 10% 기준
        if abs(self._delta) > threshold:
            # 침범 상태로 새로 진입할 때만 1회 카운트 (프레임마다 중복 집계 방지)
            if not self._invading:
                self._invasion += 1
                self._invading = True
        else:
            self._invading = False

        # 시각화
        self.visualize(img, left_cx, right_cx, lane_center, h, roi_top)

        # 디버깅 화면 출력
        # cv2.imshow("Camera View", img)
        # cv2.imshow("Mask View", msk)
        # cv2.waitKey(1)

    def visualize(self, img, left_cx, right_cx, lane_center, h, roi_top):
        """시각화 함수: 차선 및 차선 중앙 시각화"""
        if left_cx is not None:
            cv2.line(img, (int(left_cx), h), (int(left_cx), roi_top), (255, 0, 0), 5)  # 파란선
        if right_cx is not None:
            cv2.line(img, (int(right_cx), h), (int(right_cx), roi_top), (0, 255, 0), 5)  # 초록선
        if lane_center is not None:
            cv2.circle(img, (int(lane_center), int(h * 3 / 4)), 10, (0, 0, 255), -1)  # 빨간 원

    @property
    def delta(self):
        return self._delta

    @property
    def invasion_count(self):
        return self._invasion


# 테스트 코드
def main():
    tracker = LineTracker()

    # 이미지 로딩 한 번만 진행
    img = cv2.imread('../worlds/sample.jpg')
    for i in range(100):
        tracker.process(img)
        time.sleep(0.1)

if __name__ == '__main__':
    main()