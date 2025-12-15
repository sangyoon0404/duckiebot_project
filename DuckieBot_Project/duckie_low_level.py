import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class LowLevelController(Node):
    def __init__(self):
        super().__init__('low_level_controller')
        
        # 1. 고수준 명령(Twist)을 받는 리스너 (Teleop 키보드 등에서 받음)
        # 토픽: /duckie/cmd_vel
        self.sub_cmd = self.create_subscription(
            Twist,
            '/duckie/cmd_vel',
            self.cmd_callback,
            10
        )
        
        # 2. 저수준 명령(Wheel Velocity)을 보내는 퍼블리셔 (Isaac Sim으로 보냄)
        # 토픽: /duckie/wheel_left, /duckie/wheel_right
        self.pub_left = self.create_publisher(Float64, '/duckie/wheel_left', 10)
        self.pub_right = self.create_publisher(Float64, '/duckie/wheel_right', 10)

        # 3. 로봇 파라미터 (Duckiebot DB21 기준)
        self.L = 0.102   # 바퀴 간격 (Wheel Distance, m)
        self.R = 0.0318  # 바퀴 반지름 (Wheel Radius, m)
        
        self.get_logger().info("Low-level Controller Started! (Twist -> Wheel Velocity)")

    def cmd_callback(self, msg):
        # 입력받은 고수준 명령 (m/s, rad/s)
        v = msg.linear.x      # 직진 속도
        w = msg.angular.z     # 회전 속도

        # [핵심] 역운동학(Inverse Kinematics) 계산
        # 공식: v_wheel = v +/- (w * L / 2)
        v_left_linear = v - (w * self.L / 2.0)
        v_right_linear = v + (w * self.L / 2.0)

        # 선속도(m/s)를 모터 각속도(rad/s)로 변환
        # 공식: omega = v / R
        cmd_left_rad = v_left_linear / self.R
        cmd_right_rad = v_right_linear / self.R

        # 메시지 포장
        msg_left = Float64()
        msg_right = Float64()
        msg_left.data = cmd_left_rad
        msg_right.data = cmd_right_rad

        # Isaac Sim으로 전송
        self.pub_left.publish(msg_left)
        self.pub_right.publish(msg_right)

        # 디버깅용 로그 (필요시 주석 해제)
        # self.get_logger().info(f"Input(v={v:.2f}, w={w:.2f}) -> Motor(L={cmd_left_rad:.1f}, R={cmd_right_rad:.1f})")

def main(args=None):
    rclpy.init(args=args)
    node = LowLevelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 멈춤 명령 전송
        stop_msg = Float64()
        stop_msg.data = 0.0
        node.pub_left.publish(stop_msg)
        node.pub_right.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()