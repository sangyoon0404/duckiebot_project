import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedBoxFollower(Node):
    def __init__(self):
        super().__init__('red_box_follower')

        # [설정] 속도 및 감도 조절
        self.MAX_LINEAR_SPEED = 2.0
        self.MAX_ANGULAR_SPEED = 4.0
        self.TURN_GAIN = 0.02
        self.STOP_DISTANCE_AREA = 30000

        # 퍼블리셔 & 서브스크라이버 설정
        self.sub_image = self.create_subscription(
            Image,
            '/duckie/camera/image_raw',
            self.image_callback,
            10
        )
        self.pub_cmd_vel = self.create_publisher(Twist, '/duckie/cmd_vel', 10)
        
        # RQT용 토픽도 혹시 모르니 살려둡니다
        self.pub_debug_img = self.create_publisher(Image, '/duckie/camera/image_detected', 10)

        self.bridge = CvBridge()
        self.get_logger().info("Red Box Follower Started! (Pop-up Window Enabled)")

    def image_callback(self, msg):
        try:
            # 1. 이미지 변환 (ROS -> OpenCV)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            height, width, _ = cv_image.shape

            # 2. 이미지 처리 (BGR -> HSV)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # 3. 빨간색 마스크 생성
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 100, 100])
            upper_red2 = np.array([180, 255, 255])

            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.addWeighted(mask1, 1.0, mask2, 1.0, 0.0)

            # 4. 윤곽선 찾기
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            twist = Twist()
            
            # 빨간색 물체 발견 시
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)

                if area > 100:
                    # [시각화] 초록색 박스 그리기
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    
                    # [시각화] 중심점 및 텍스트 표시
                    cx = x + w // 2
                    cy = y + h // 2
                    cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.putText(cv_image, "Target", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    # [제어] 주행 로직
                    error_x = (width / 2) - cx
                    angular_z = self.TURN_GAIN * error_x
                    twist.angular.z = max(min(angular_z, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)

                    if area < self.STOP_DISTANCE_AREA:
                        twist.linear.x = self.MAX_LINEAR_SPEED * (1.0 - abs(error_x) / (width / 2))
                        twist.linear.x = max(twist.linear.x, 0.2)
                    else:
                        twist.linear.x = 0.0
                        self.get_logger().info("Target Reached! Stopping.")

            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            # 5. 명령 전송
            self.pub_cmd_vel.publish(twist)
            self.pub_debug_img.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

            # [핵심] OpenCV 창 띄우기 (여기서 화면을 보여줍니다)
            cv2.imshow("Red Box View", cv_image)
            cv2.waitKey(1)  # 1ms 대기 (화면 갱신을 위해 필수)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RedBoxFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 창 닫기
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()