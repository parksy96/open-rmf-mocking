import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import FleetState, RobotState, Location, RobotMode
from std_msgs.msg import String  # 명령 수신용 메시지 타입
import socket
import threading
import json
import time

class SimpleAdapter(Node):
    def __init__(self):
        super().__init__('simple_linear_adapter')
        
        # 1. Publisher (상태 보고: 로봇 -> RMF)
        self.publisher = self.create_publisher(FleetState, 'fleet_states', 10)
        
        # 2. Subscriber (명령 수신: RMF/Streamlit -> 로봇)
        self.command_sub = self.create_subscription(
            String, 
            '/robot_command', 
            self.handle_command, 
            10
        )
        
        # 로봇 상태 변수
        self.current_x = 0.0
        self.fleet_name = "mock_fleet"
        self.robot_name = "stridebot_1"
        self.map_name = "L1"
        self.battery = 100.0

        # TCP 서버 설정
        self.host = '127.0.0.1'
        self.port = 12345
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        
        self.client_conn = None  # 연결된 클라이언트(로봇) 저장용

        # 서버 스레드 시작
        self.thread = threading.Thread(target=self.start_server)
        self.thread.daemon = True
        self.thread.start()

        # 10Hz 상태 발행 타이머
        self.timer = self.create_timer(0.1, self.publish_state)
        self.get_logger().info(f"✅ Adapter Ready on {self.port}. Waiting for Robot...")

    def start_server(self):
        while True:
            try:
                conn, addr = self.server_socket.accept()
                self.client_conn = conn  # 연결 객체 저장 (명령 보낼 때 사용)
                self.get_logger().info(f"🔗 Robot Connected: {addr}")
                
                with conn:
                    while True:
                        data = conn.recv(1024)
                        if not data: break
                        try:
                            msg = json.loads(data.decode())
                            self.current_x = float(msg.get("x", 0.0))
                            self.battery = float(msg.get("battery", 100.0))
                        except ValueError:
                            pass
                
                self.client_conn = None
                self.get_logger().warn("❌ Robot Disconnected")
            except Exception as e:
                self.get_logger().error(f"Server Error: {e}")

    def handle_command(self, msg):
        """ROS 토픽으로 들어온 명령을 로봇(TCP)에게 전달"""
        cmd = msg.data.lower()
        self.get_logger().info(f"📥 Command Received from ROS: {cmd}")
        
        if self.client_conn:
            try:
                # 로봇에게 그대로 전송 (예: "start", "stop")
                self.client_conn.sendall(cmd.encode())
            except Exception as e:
                self.get_logger().error(f"Failed to send command to robot: {e}")
        else:
            self.get_logger().warn("⚠️ No robot connected to send command!")

    def publish_state(self):
        fleet_msg = FleetState()
        fleet_msg.name = self.fleet_name

        robot_msg = RobotState()
        robot_msg.name = self.robot_name
        robot_msg.battery_percent = self.battery
        
        loc = Location()
        loc.x = self.current_x
        loc.y = 0.0
        loc.yaw = 0.0
        loc.t = self.get_clock().now().to_msg()
        loc.level_name = self.map_name
        
        robot_msg.location = loc
        robot_msg.mode.mode = RobotMode.MODE_MOVING
        
        fleet_msg.robots = [robot_msg]
        self.publisher.publish(fleet_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()