# RMF Robot Client (Fleet Adapter & FSM)

본 저장소는 **Open-RMF 기반 다중 로봇 관제 시스템**에서 실제 로봇(Client) 측 코드를 담당하는 패키지 모음입니다.  

- **역할**: RMF Server가 내려주는 작업/경로를 수신하여 FSM을 통해 Nav2 주행 스택으로 실행  
- **구성**: 로봇 어댑터(Fleet Adapter), 상태 기계(FSM), 주행 스택(Navigation2 Stack), 외부 브리지(MQTT/Socket.IO), Docker 환경  
- **활용**: 실외 배달로봇 다중 운영 및 관제 시스템에 적용  

---

## 📌 1. 필요하게 된 상황
실외 환경에서 다수의 배달로봇을 운행하려면, 각 로봇이 단독으로 주행만 할 수 있어서는 부족합니다.  
- 중앙 관제(RMF Server)와 로봇 간 **작업/상태 교환**이 필요  
- 보행자, 차량, 날씨 등으로 인한 장애물 등장과 위험 지역 등 **실외 주행 제약 조건** 존재

---

## 🔧 2. 시스템 구성
로봇 단(Client)에서는 다음과 같은 구성 요소로 동작합니다:

- **로봇 어댑터 (`fleet_adapter`)**  
  - RMF Server가 내려주는 작업/경로(PathRequest)을 Nav2 명령으로 변환  
  - 로봇 상태(RobotState)를 주기적으로 RMF Server에 보고  

- **상태 기계 (`fsm_waypoint_node`)**  
  - Nav2 태스크의 상위 제어(실행·취소·재시작)  
  - Nav2가 제공하는 피드백을 바탕으로 안정적인 주행 사이클 유지  
  - 문제 상황 발생 시 정지, 상황 해소 시 재주행 관리
  - 주행/악세서리 텔레메트리 데이터 발행  

- **주행 스택 (`navigation2_stack`)**  
  - 로봇 주행의 핵심 엔진 (경로 계획·추종·복구)  
  - 실외 환경에 맞게 커스텀  
    - Planner: **StraightLine** (직선 경로 최적화)
    - Controller: **RotationShim + Regulated Pure Pursuit**  
    - Behavior Tree: `navigate_to_pose_w_replanning_and_recovery.xml` (Stop & Go)

- **외부 브리지 (`rmf_demos_bridges`)**  
  - MQTT/Socket.IO를 통한 클라우드 및 외부 모니터링 연계  
  - 텔레메트리 데이터를 외부 시스템에 전달  

- **원격 제어 (WebSocket 기반)**  
  - WSS(WebSocket Secure)를 통해 외부에서 제어 명령 수신  

- **Docker 환경**  
  - 전체 시스템을 컨테이너로 패키징하여 손쉽게 실행·배포 가능

---

## 🔀 3. 시스템 아키텍처 & 데이터 흐름
```mermaid
flowchart LR
  subgraph Control
    RMF_Server["rmf_server"]
  end

  subgraph RobotSide
    Fleet_Adapter["fleet_adapter"]
    FSM["fsm_waypoint_node"]
    Nav2["navigation2_stack"]
    Robot["배달로봇"]
  end

  subgraph External
    Bridges["MQTT / Socket.IO Bridge"]
    Cognito["WebSocket Control"]
  end

  RMF_Server -- PathRequest --> Fleet_Adapter
  Fleet_Adapter --> FSM
  FSM -->|Action Client| Nav2
  Nav2 -->|TF / Odom| Robot
  Nav2 -->|Result / Feedback| FSM
  Fleet_Adapter -->|RobotState| RMF_Server
  FSM --> Bridges
  FSM --> Cognito