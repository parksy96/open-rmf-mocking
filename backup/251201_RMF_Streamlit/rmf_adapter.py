import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import FleetState, RobotState, Location, RobotMode
import socket
import threading
import json
import time

class SimpleAdapter(Node):
    def __init__(self):
        super().__init__('simple_linear_adapter')
        
        # Publisher 설정
        self.publisher = self.create_publisher(FleetState, 'fleet_states', 10)
        
        # 상태 변수 초기화 (중요)
        self.current_x = 0.0 
        self.fleet_name = "mock_fleet"
        self.robot_name = "stridebot_1"
        self.map_name = "L1"  # 맵이름 수동지정

        # TCP 서버 설정
        self.host = '127.0.0.1'
        self.port = 12345
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        
        # 스레드 시작
        self.thread = threading.Thread(target=self.start_server)
        self.thread.daemon = True
        self.thread.start()

        # 10Hz 주기로 Publish
        self.timer = self.create_timer(0.1, self.publish_state)
        self.get_logger().info(f"Adapter Ready. Map: {self.map_name}")

    def start_server(self):
        while True:
            try:
                conn, addr = self.server_socket.accept()
                with conn:
                    self.get_logger().info(f"Client Connected: {addr}")
                    while True:
                        data = conn.recv(1024)
                        if not data: break
                        try:
                            # 데이터 파싱
                            msg = json.loads(data.decode())
                            # 값 갱신
                            self.current_x = float(msg.get("x", 0.0))
                        except ValueError:
                            pass
            except Exception as e:
                pass

    def publish_state(self):
        fleet_msg = FleetState()
        fleet_msg.name = self.fleet_name

        robot_msg = RobotState()
        robot_msg.name = self.robot_name
        robot_msg.battery_percent = 100.0 # 배터리 정상값 지정
        
        loc = Location()
        loc.x = self.current_x
        loc.y = 0.0
        loc.yaw = 0.0 # 회전 없음 고정
        loc.t = self.get_clock().now().to_msg()
        loc.level_name = self.map_name # 층수 이름 L1 - L2
        
        robot_msg.location = loc
        robot_msg.mode.mode = RobotMode.MODE_MOVING
        
        fleet_msg.robots = [robot_msg]
        self.publisher.publish(fleet_msg)
        
        # 디버깅용 로그 (터미널에서 x값이 잘 바뀌는지 확인용)
        # self.get_logger().info(f"Publishing X: {self.current_x} at {self.map_name}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()