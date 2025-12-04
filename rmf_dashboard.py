import streamlit as st
import streamlit.components.v1 as components
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

# --- 1. Python ROS(rclpy) 설정  ---
# 시각화(데이터 수신)는 JS가 하고, 명령(데이터 송신)은 Python으로 진행
class CommandPublisher(Node):
    def __init__(self):
        super().__init__('streamlit_commander')
        self.publisher = self.create_publisher(String, '/robot_command', 10)

    def send_command(self, cmd_str):
        msg = String()
        msg.data = cmd_str
        self.publisher.publish(msg)

@st.cache_resource
def get_commander():
    if not rclpy.ok():
        rclpy.init()
    node = CommandPublisher()
    # 별도 스레드x
    return node

# --- 2. 메인 앱 ---
def main():
    st.set_page_config(page_title="RMF Control Tower", layout="wide")
    st.markdown("##  Open-RMF Real-time Control Tower")
    
    # Commander 노드 가져오기
    commander = get_commander()

    # =================================================================
    # 시각화 (HTML + JS)
    # =================================================================
    html_code = """
    <!DOCTYPE html>
    <html>
    <head>
        <script type="text/javascript" src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"></script>
        <style>
            body { margin: 0; padding: 10px; font-family: sans-serif; background-color: white; }
            
            /* 트랙 디자인 */
            .track-wrapper {
                position: relative;
                width: 95%;
                height: 60px;
                background-color: #f1f3f5;
                border-radius: 30px;
                margin: 20px auto;
                border: 2px solid #dee2e6;
            }
            
            /* 로봇 아이콘 */
            .robot-icon {
                position: absolute;
                top: 50%;
                left: 0%; 
                width: 45px;
                height: 45px;
                background: linear-gradient(135deg, #00C9FF, #92FE9D);
                border: 3px solid white;
                border-radius: 50%;
                transform: translate(-50%, -50%);
                box-shadow: 0 4px 10px rgba(0,0,0,0.2);
                transition: left 0.1s linear; /* 움직임 애니메이션 */
                
                display: flex; align-items: center; justify-content: center;
                font-size: 20px; font-weight: bold; color: white;
            }

            /* 정보 텍스트 */
            .info-panel {
                display: flex; justify-content: space-around;
                margin-top: 20px; color: #495057; font-weight: bold;
            }
            .badge { background: #eee; padding: 5px 15px; border-radius: 10px; }
        </style>
    </head>
    <body>
        <div class="track-wrapper">
            <div id="robot" class="robot-icon">🤖</div>
        </div>
        
        <div class="info-panel">
            <span class="badge">🤖 <span id="bot-name">-</span></span>
            <span class="badge">📍 <span id="pos-x">0.00</span> m</span>
            <span class="badge">🔋 <span id="battery">0</span> %</span>
            <span class="badge">📡 <span id="status" style="color:red">●</span></span>
        </div>

        <script>
            // 브라우저가 직접 ROSBridge(9090)에 접속
            var ros = new ROSLIB.Ros({ url : 'ws://localhost:9090' });

            ros.on('connection', function() {
                document.getElementById("status").style.color = "green";
            });
            ros.on('close', function() {
                document.getElementById("status").style.color = "red";
            });

            var listener = new ROSLIB.Topic({
                ros : ros,
                name : '/fleet_states',
                messageType : 'rmf_fleet_msgs/msg/FleetState'
            });

            listener.subscribe(function(msg) {
                if (msg.robots && msg.robots.length > 0) {
                    var robot = msg.robots[0];
                    var x = robot.location.x;
                    
                    // 텍스트 업데이트
                    document.getElementById("bot-name").innerText = robot.name;
                    document.getElementById("pos-x").innerText = x.toFixed(2);
                    document.getElementById("battery").innerText = robot.battery_percent;

                    // 위치 이동 (CSS Transition 활용)
                    var percent = (x / 10.0) * 100;
                    percent = Math.max(0, Math.min(100, percent));
                    document.getElementById("robot").style.left = percent + "%";
                }
            });
        </script>
    </body>
    </html>
    """

    # Streamlit HTML/JS (높이 200px)
    components.html(html_code, height=180)

    # =================================================================
    # 3. 명령 버튼 (Python)
    # =================================================================
    st.divider()
    st.subheader("🎮 Command Interface")
    
    col1, col2, col3 = st.columns(3)
    
    if col1.button("▶ START", use_container_width=True, type="primary"):
        commander.send_command("start")
        st.toast("🟢 로봇 이동 시작")

    if col2.button("⏸ STOP", use_container_width=True, type="secondary"):
        commander.send_command("stop")
        st.toast("🔴 로봇 정지")

    if col3.button("↺ RESET", use_container_width=True):
        commander.send_command("reset")
        st.toast("⚪ 위치 초기화")

if __name__ == "__main__":
    main()