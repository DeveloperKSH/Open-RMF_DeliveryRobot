# RMF Robot Client (Fleet Adapter & FSM)

본 저장소는 **Open-RMF 기반 다중 로봇 관제 시스템**에서  
실제 로봇단(Client) 측 코드를 담당하는 패키지 모음입니다.  

- **역할**: RMF 서버가 내려주는 미션/경로(PathRequest)를 Nav2를 통해 실행  
- **구성**: 로봇 어댑터, 상태머신(FSM), 외부 브리지(MQTT/Socket.IO), Docker 환경  
- **활용**: 실외 물류로봇 다중 운영 및 관제 시스템에 적용  

---

## 1. 📌 필요하게 된 상황
실외 환경에서 다수의 물류 로봇을 운행하려면, 각 로봇이 단독으로 주행만 할 수 있어서는 부족합니다.  
- 중앙 관제(RMF)와 로봇 간 **미션/상태 교환**이 필요  
- 실외 환경 특성상 장애물 등장, 직선 위주 동선 등 **주행 제약 조건** 존재  

---

## 2. 🔧 해결 방법
이를 위해 로봇단에 다음 기능을 구현했습니다:
- **Fleet Adapter (`fleet_robot`)**:  
  RMF가 내려주는 PathRequest를 Nav2 Action으로 변환
- **FSM (`fsm_waypoint`)**:  
  Nav2 태스크 실행 관리, Stop&Go 로직, 재계획·복구 처리
- **외부 브리지 (`rmf_demos_bridges`)**:  
  MQTT/Socket.IO를 통한 클라우드/외부 모니터링 연계
- **원격 제어 (`cognito`)**:  
  WebSocket 기반 실시간 제어 및 상태 반영
- **Nav2 커스텀**:  
  - Planner: **StraightLine** (실외 직선 경로 최적화)  
  - Controller: **RotationShim + Regulated Pure Pursuit**  
  - BehaviorTree: `navigate_to_pose_w_replanning_and_recovery.xml` (Stop&Go)  

---

## 3. 🏗️ 코드 구성 및 역할
```mermaid
flowchart LR
  subgraph Control[관제 / 서버]
    RMF_Server[RMF Core / Server]
  end

  subgraph RobotSide[로봇 단]
    Fleet_Adapter[fleet_robot (Fleet Adapter)]
    FSM[fsm_waypoint (FSM)]
    Nav2[Navigation2 Stack]
    Robot[물류 로봇]
  end

  subgraph External[외부 연계]
    Bridges[MQTT / Socket.IO Bridge]
    Cognito[WebSocket Control]
  end

  RMF_Server -- PathRequest --> Fleet_Adapter
  Fleet_Adapter --> FSM
  FSM -->|Action Client| Nav2
  Nav2 -->|TF/Odom| Robot
  Nav2 -->|Result/Feedback| FSM
  Fleet_Adapter -->|RobotState| RMF_Server
  FSM --> Bridges
  FSM --> Cognito