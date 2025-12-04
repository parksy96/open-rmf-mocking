# 🤖 Open-RMF Simple Monitor & Control

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue) ![Open-RMF](https://img.shields.io/badge/Open--RMF-22.09-orange) ![Python](https://img.shields.io/badge/Python-3.10-yellow) ![Streamlit](https://img.shields.io/badge/Streamlit-1.28-red)

**Open-RMF의 데이터 흐름과 관제 시스템 테스트 프로젝트**

가상 로봇(Mock Robot)을 생성하고, Custom Adapter를 통해 Open-RMF와 통신하며, Streamlit 기반의 웹 대시보드에서 실시간 모니터링 및 제어(Start/Stop)를 수행


## 🏗️ Architecture

 **Mocking Client** - **RMF Adapter** - **Web UI** Structure

```mermaid
graph LR
    subgraph "External World (Mock)"
        Client[Mock Robot Client] -- "TCP/IP (JSON)" --> Adapter
        Adapter -- "TCP/IP (Command)" --> Client
    end

    subgraph "ROS 2 / Open-RMF"
        Adapter[Custom RMF Adapter] -- "Publish /fleet_states" --> ROS((ROS 2 Core))
        ROS -- "Subscribe /robot_command" --> Adapter
    end

    subgraph "Web Interface"
        ROS -- "Rosbridge (WebSocket)" --> Browser["Streamlit Dashboard<br>(HTML/JS Visualization)"]
        Browser -- "Streamlit Button" --> ROS
    end
````

## ✨ Key Features

  * **Mock Robot Client**: 실제 로봇 없이도 TCP 소켓 통신으로 위치 데이터(x좌표, 배터리)를 생성하고 명령을 수신
  * **Custom Python Adapter**: 외부 로봇 프로토콜(TCP/JSON)을 Open-RMF 표준 메시지(FleetState)로 변환
  * **Bi-directional Control**: 웹 대시보드에서 Start/Stop 버튼을 누르면 로봇이 즉각 반응
  * **Dashboard**
      * **Control**: Python Streamlit의 위젯 사용
      * **Visualization**: HTML5/CSS/JS + roslibjs를 사용하여 시각화 구현


## 🛠️ Prerequisites (환경 설정)

  * **OS**: Ubuntu 22.04 LTS
  * **Middleware**: ROS 2 Humble Hawksbill

### Libraries

```bash
# Open-RMF Essential
sudo apt install ros-humble-rmf-dev ros-humble-rmf-fleet-msgs

# WebSocket Bridge
sudo apt install ros-humble-rosbridge-server

# Python Dependencies
pip install streamlit
```


## 🚀 How to Run

### 1\. Build Package

```bash
cd ~/Open-RMF/rmf_ws
colcon build --packages-select simple_linear_adapter
source install/setup.bash
```

### 2\. Run Adapter (Terminal 1)

```bash
source ~/Open-RMF/rmf_ws/install/setup.bash
ros2 run simple_linear_adapter adapter
```

### 3\. Run Mock Robot (Terminal 2)

```bash
python3 mock_robot_client.py
```

### 4\. Run Rosbridge (Terminal 3)

```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 5\. Run Dashboard (Terminal 4)

```bash
source /opt/ros/humble/setup.bash
streamlit run rmf_dashboard.py
```

**접속 주소**: [http://localhost:8501](https://www.google.com/search?q=http://localhost:8501)


## 📂 File Structure

```text
.
├── mock_robot_client.py    # 가상 로봇 클라이언트 (TCP Send/Recv)
├── rmf_dashboard.py        # Streamlit + HTML/JS 대시보드 소스
└── rmf_ws/                 # ROS 2 Workspace
    └── src/
        └── simple_linear_adapter/  # Custom Adapter Package
            ├── simple_linear_adapter/
            │   └── rmf_adapter.py  # 어댑터 로직 (Node)
            ├── package.xml
            └── setup.py
```

## 🗓️ Roadmap

  - [x] Phase 1: Mocking Robot & Basic Adapter (Monitoring)
  - [x] Phase 2: Web Visualization with Streamlit & Roslibjs
  - [x] Phase 3: Bi-directional Control (Command)
  - [ ] Phase 4: Integration with RMF Core (rmf\_fleet\_adapter) & Traffic Editor Map


```
```
