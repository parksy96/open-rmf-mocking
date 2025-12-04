import socket
import time
import json
import threading

HOST = '127.0.0.1'
PORT = 12345

# 로봇 상태 관리 (Thread-safe를 위해 전역 사용)
robot_state = {
    "x": 0.0,
    "active": False,  # 기본값: 정지 상태
    "direction": 1,
    "battery": 100.0
}

def listen_for_commands(sock):
    """서버(Adapter)로부터 명령을 수신하는 별도 스레드"""
    print("👂 Command Listener Started")
    while True:
        try:
            data = sock.recv(1024)
            if not data:
                print("Disconnected from server")
                break
            
            # 명령 해석
            cmd = data.decode().strip()
            print(f"📨 Command Received: [{cmd}]")
            
            if cmd == "start":
                robot_state["active"] = True
                print("▶ Robot Started Moving")
            elif cmd == "stop":
                robot_state["active"] = False
                print("⏸ Robot Stopped")
            elif cmd == "reset":
                robot_state["x"] = 0.0
                robot_state["active"] = False
                print("↺ Robot Position Reset")
                
        except Exception as e:
            print(f"Listener Error: {e}")
            break

def run_mock_robot():
    print("--- Mock Robot Client (Control Enabled) ---")
    
    # 1. 서버 접속 시도
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))
            print(f"✅ Connected to Adapter at {HOST}:{PORT}")
            
            # 2. 수신 스레드 실행 (백그라운드에서 명령 대기)
            listener = threading.Thread(target=listen_for_commands, args=(s,), daemon=True)
            listener.start()
            break
        except ConnectionRefusedError:
            print("⏳ Adapter not ready... retrying in 2s")
            time.sleep(2)

    # 3. 메인 루프 (상태 전송 및 이동)
    speed = 0.2
    try:
        while True:
            # active 상태일 때만 좌표 이동
            if robot_state["active"]:
                robot_state["x"] += speed * robot_state["direction"]
                
                # 벽에 닿으면 방향 전환
                if robot_state["x"] > 10.0:
                    robot_state["x"] = 10.0
                    robot_state["direction"] = -1
                elif robot_state["x"] < 0.0:
                    robot_state["x"] = 0.0
                    robot_state["direction"] = 1
                
                # 배터리 소모 시뮬레이션
                robot_state["battery"] -= 0.01

            # 상태 전송
            payload = {
                "x": round(robot_state["x"], 2),
                "battery": round(robot_state["battery"], 1)
            }
            s.sendall(json.dumps(payload).encode())
            
            time.sleep(0.1) # 10Hz
    except KeyboardInterrupt:
        print("\n🛑 Simulation Terminated")
        s.close()

if __name__ == "__main__":
    run_mock_robot()