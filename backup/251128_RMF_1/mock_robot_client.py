import socket
import time
import json

HOST = '127.0.0.1'
PORT = 12345

def run_mock_robot():
    print("--- Mock Robot Client Started ---")
    
    # 1. 서버(Adapter)에 접속 시도
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))
            print(f"[Connected] Connected to Adapter at {HOST}:{PORT}")
            break
        except ConnectionRefusedError:
            print("[Waiting] Adapter not ready... retrying in 2 sec")
            time.sleep(2)

    # 2. 선형 움직임 시뮬레이션 (0 -> 10 -> 0 반복)
    x = 0.0
    direction = 1
    speed = 0.2  # 속도 조절

    try:
        while True:
            # 좌표 계산 (왔다 갔다)
            x += speed * direction
            if x > 10.0 or x < 0.0:
                direction *= -1 
            
            # 패킷 전송 (JSON 포맷)
            payload = {"x": round(x, 2)}
            data = json.dumps(payload)
            s.sendall(data.encode())
            
            # print(f"[Sent] {data}") #  주석 처리
            time.sleep(0.1) # 10Hz
    except KeyboardInterrupt:
        print("\n[Stopped] Simulation Stopped")
        s.close()

if __name__ == "__main__":
    run_mock_robot()