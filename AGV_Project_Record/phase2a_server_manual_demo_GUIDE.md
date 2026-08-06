# Phase 2A 서버 변경 가이드: 수동 `demo12` 물리 데모

> 이 문서는 이전의 `phase2a_server_manual_demo.patch`를 대체한다. 이전 파일은
> unified-diff hunk 길이가 잘못되어 `git apply --check`에서 `corrupt patch at
> line 58`로 실패했고, `Map.cpp`의 한글 주석 문맥도 인코딩에 따라 달라질 수
> 있었다. 따라서 그 패치를 적용하지 말고 아래 변경을 서버 저장소에서 직접
> 반영한 뒤 빌드한다.

대상 저장소:

```text
/home/hwang-juhyun/FinalProject/SmartFactory_AGV_DigitalTwin_Server
```

이 단계의 동작 계약은 다음과 같다.

```text
서버 시작 → AGV 1 한 대를 물리 맵 노드 1에 생성
ESP32 접속 → 경로 자동 전송 없음
서버 콘솔에서 demo12 입력 → 정확히 [1, 2]만 한 번 전송
ESP32는 BOOT 전까지 DISARMED
ESP32가 STATUS(node=2)와 ARRIVED(node=2)를 보내면 서버/Unity 갱신
후속 자동 작업 없음
```

## 1. `Shared/PhysicalDemoConfig.hpp` 추가

```cpp
#pragma once

#include <cstdint>

#define PHYSICAL_DEMO_MODE 1

namespace PhysicalDemo
{
    inline constexpr uint32_t kAgvID = 1;
    inline constexpr uint32_t kStartNodeID = 1;
    inline constexpr uint32_t kTargetNodeID = 2;
}
```

## 2. `Shared/MapData_PhysicalDemo.json` 추가

좌표 단위는 미터다. 1→2는 실제 직진 30 cm이고, 노드 3은 다음 L자 단계에서
사용할 위치다.

```json
{
  "nodes": [
    { "id": 1, "x": 0.0, "z": 0.0 },
    { "id": 2, "x": 0.3, "z": 0.0 },
    { "id": 3, "x": 0.3, "z": -0.3 }
  ],
  "links": [
    { "from": 1, "to": 2, "type": 0, "dist": 0.3 },
    { "from": 2, "to": 1, "type": 0, "dist": 0.3 },
    { "from": 2, "to": 3, "type": 0, "dist": 0.3 },
    { "from": 3, "to": 2, "type": 0, "dist": 0.3 }
  ]
}
```

현재 `Map.cpp`는 직선 링크의 제어점 필드가 빠져 있어도 0으로 초기화하므로
위 JSON이 유효하다.

## 3. `Shared/Map.cpp`: 데모 맵 선택

상단 include에 추가한다.

```cpp
#include "PhysicalDemoConfig.hpp"
```

`MapManager::Init()` 안의 기존 `mapPathCandidates` 블록 전체를 다음으로
교체한다. 한글 디버그 문자열은 수정하지 않는다.

```cpp
#if PHYSICAL_DEMO_MODE
    constexpr const char* mapFileName = "MapData_PhysicalDemo.json";
#else
    constexpr const char* mapFileName = "MapData.json";
#endif

    const std::vector<std::filesystem::path> mapPathCandidates = {
        std::filesystem::path("Shared") / mapFileName,
        std::filesystem::path("..") / "Shared" / mapFileName,
        std::filesystem::path("../..") / "Shared" / mapFileName,
        std::filesystem::path("../../..") / "Shared" / mapFileName
    };
```

## 4. `Server/TaskManager.cpp`: AGV 1 자동 작업 차단

include에 추가한다.

```cpp
#include "PhysicalDemoConfig.hpp"
```

다음 세 함수의 첫 줄에 같은 가드를 추가한다.

- `TaskManager::OnRobotIdle`
- `TaskManager::OnRobotLoadCompleted`
- `TaskManager::OnRobotUnloadCompleted`

```cpp
#if PHYSICAL_DEMO_MODE
    if (_e.agvID == PhysicalDemo::kAgvID)
        return;
#endif
```

## 5. `Server/RoutePlanner.hpp`: 전용 함수 선언

public 영역에 추가한다.

```cpp
bool TryCreatePhysicalDemoRoute(uint32_t agvID,
                                uint32_t expectedStartNodeID,
                                uint32_t targetNodeID,
                                float serverTime);
```

## 6. `Server/RoutePlanner.cpp`: 재시도 없는 정확한 `[1,2]` 경로

`TryReservePathTransaction()` 다음, 일반 `CreateRoute()` 전에 추가한다.

```cpp
bool RoutePlanner::TryCreatePhysicalDemoRoute(uint32_t agvID,
                                               uint32_t expectedStartNodeID,
                                               uint32_t targetNodeID,
                                               float serverTime)
{
    Robo* agv = dynamic_cast<Robo*>(AGVManager::GetInstance().FindAGV(agvID));
    if (!agv)
    {
        std::cout << "[PHYSICAL_DEMO] Rejected: AGV does not exist.\n";
        return false;
    }
    if (agv->GetState() != AGVState::IDLE)
    {
        std::cout << "[PHYSICAL_DEMO] Rejected: AGV is not IDLE.\n";
        return false;
    }
    if (agv->GetCurrentNodeID() != expectedStartNodeID)
    {
        std::cout << "[PHYSICAL_DEMO] Rejected: expected start node "
                  << expectedStartNodeID << ".\n";
        return false;
    }
    if (m_MasterPlans.find(agvID) != m_MasterPlans.end())
    {
        std::cout << "[PHYSICAL_DEMO] Rejected: active plan exists.\n";
        return false;
    }

    const auto pending = std::find_if(
        m_PendingRoutes.begin(), m_PendingRoutes.end(),
        [agvID](const PendingRoute& route) { return route.agvID == agvID; });
    if (pending != m_PendingRoutes.end())
    {
        std::cout << "[PHYSICAL_DEMO] Rejected: pending plan exists.\n";
        return false;
    }

    std::vector<PathStep> path;
    if (!TryFindPath(agvID, targetNodeID, serverTime, path)
        || path.size() != 2
        || path[0].nodeID != expectedStartNodeID
        || path[1].nodeID != targetNodeID)
    {
        std::cout << "[PHYSICAL_DEMO] Rejected: route is not exactly [1 -> 2].\n";
        return false;
    }

    // 일반 CreateRoute()와 달리 실패 시 m_PendingRoutes에 넣지 않는다.
    if (!TryReservePathTransaction(agvID, path, targetNodeID, serverTime))
    {
        std::cout << "[PHYSICAL_DEMO] Rejected: reservation failed.\n";
        return false;
    }

    HandlePathFound(agvID, targetNodeID, MissionPurpose::NONE, path);
    return true;
}
```

같은 파일의 `RoutePlanner::FinishRoute()`에서 목적지 도착 분기에
`MissionPurpose::NONE` 처리를 추가한다. `IDLE_READY`는 발행하지 않는다.

```cpp
else if (purpose == MissionPurpose::NONE)
{
    agv->ChangeState(AGVState::IDLE);
    agv->StartWorkTimer(0.0f);
    std::cout << "[PHYSICAL_DEMO] AGV " << agvID
              << " arrived at node " << _e.currentNodeID << ".\n";
}
```

기존 함수 끝의 `m_MasterPlans.erase(agvID)`는 그대로 둔다.

## 7. `Server/NetworkManagerServer.hpp`: 수동 시작 함수

public 영역에 추가한다.

```cpp
bool TryStartPhysicalDemoRoute();
```

## 8. `Server/NetworkManagerServer.cpp`

include에 추가한다.

```cpp
#include "PhysicalDemoConfig.hpp"
```

### 8-1. HELLO 시 자동 재전송 금지

`HandleRobotHelloPacket()` 끝의 기존
`ResendCurrentRouteToController(assignedAgvID)` 호출을 다음 구조로 감싼다.

```cpp
#if PHYSICAL_DEMO_MODE
    if (assignedAgvID == PhysicalDemo::kAgvID)
    {
        std::cout << "[PHYSICAL_DEMO] AGV 1 connected; route resend disabled.\n"
                  << "[PHYSICAL_DEMO] Type demo12 when ready.\n";
    }
    else
#endif
    if (!RoutePlanner::GetInstance().ResendCurrentRouteToController(assignedAgvID))
    {
        std::cout << "[RoutePlanner] No active route to resend for AGV "
                  << assignedAgvID << "\n";
    }
```

### 8-2. 물리 모드에서는 AGV 1대만 생성

`CreateSimulationWorld()`의 테스트케이스별 `spawnCount/initNodes` 선언 전체를
다음 전처리 구조로 감싼다.

```cpp
#if PHYSICAL_DEMO_MODE
    int spawnCount = 1;
    uint32_t initNodes[1] = { PhysicalDemo::kStartNodeID };
#else
    // 기존 _TESTCASE0 ~ _TESTCASE4 블록을 변경 없이 이곳에 둔다.
#endif
```

함수 아래쪽의 모든 AGV에 `IDLE_READY`를 발행하는 `for (Robo* agv : Robos)`
블록 전체는 다음으로 감싼다.

```cpp
#if !PHYSICAL_DEMO_MODE
    // 기존 IDLE_READY 발행 for 블록
#endif
```

### 8-3. 수동 시작 구현

`UpdateWorld()`보다 앞에 추가한다.

```cpp
bool NetworkManagerServer::TryStartPhysicalDemoRoute()
{
#if !PHYSICAL_DEMO_MODE
    std::cout << "[PHYSICAL_DEMO] Disabled at compile time.\n";
    return false;
#else
    const auto session = m_AgvIdToRobotSessionMap.find(PhysicalDemo::kAgvID);
    if (session == m_AgvIdToRobotSessionMap.end() || !session->second)
    {
        std::cout << "[PHYSICAL_DEMO] Rejected: ESP32 AGV 1 is not connected.\n";
        return false;
    }

    const bool started = RoutePlanner::GetInstance().TryCreatePhysicalDemoRoute(
        PhysicalDemo::kAgvID,
        PhysicalDemo::kStartNodeID,
        PhysicalDemo::kTargetNodeID,
        m_TotalElapsedServerTime);

    std::cout << (started
        ? "[PHYSICAL_DEMO] Sent exact route [1 -> 2].\n"
        : "[PHYSICAL_DEMO] demo12 rejected; no route was sent.\n");
    return started;
#endif
}
```

## 9. `Server/ServerMain.cpp`: Bind 오류 처리와 `demo12` 콘솔

include를 정리/추가한다. `STDIN_FILENO` 때문에 `<unistd.h>`가 반드시
필요하다.

```cpp
#include <cstdlib>
#include <iostream>
#include <poll.h>
#include <string>
#include <unistd.h>
#include "PhysicalDemoConfig.hpp"
```

소켓 생성, Bind, Listen의 `assert(...)`를 제거하고 다음처럼 실제 호출이
Release에서도 실행되게 한다.

```cpp
SocketAddressPtr serverAddr =
    SocketAddressFactory::CreateIPv4FromString("0.0.0.0:6666");
TCPSocketPtr sockServerTcp = SocketUtil::CreateTCPSocket(AF_INET);
if (!serverAddr || !sockServerTcp)
{
    std::cerr << "Socket/address creation failed\n";
    return EXIT_FAILURE;
}

int option = 1;
setsockopt(sockServerTcp->GetSocket(), SOL_SOCKET, SO_REUSEADDR,
           &option, sizeof(option));

if (sockServerTcp->Bind(*serverAddr) == ERROR)
{
    std::cerr << "Bind failed: TCP 6666 may already be in use\n";
    return EXIT_FAILURE;
}
if (sockServerTcp->Listen() == ERROR)
{
    std::cerr << "Listen failed on TCP 6666\n";
    return EXIT_FAILURE;
}
```

메인 루프 직전에 추가한다.

```cpp
#if PHYSICAL_DEMO_MODE
    pollfd commandInput{};
    commandInput.fd = STDIN_FILENO;
    commandInput.events = POLLIN;
    std::cout << "[PHYSICAL_DEMO] Automatic scheduling is OFF.\n"
              << "[PHYSICAL_DEMO] Connect ESP32, then type: demo12\n";
#endif
```

`while (g_LOOP)`의 가장 위에 추가한다.

```cpp
#if PHYSICAL_DEMO_MODE
    commandInput.revents = 0;
    if (poll(&commandInput, 1, 0) > 0 && (commandInput.revents & POLLIN))
    {
        std::string command;
        if (std::getline(std::cin, command))
        {
            if (!command.empty() && command.back() == '\r')
                command.pop_back();

            if (command == "demo12")
                NetworkManagerServer::sInstance->TryStartPhysicalDemoRoute();
            else if (!command.empty())
                std::cout << "[PHYSICAL_DEMO] Unknown command. Use: demo12\n";
        }
    }
#endif
```

## 10. 빌드 및 USB 전용 검증

서버가 실행 중이지 않은지 먼저 확인한 뒤 빌드한다.

```bash
cd /home/hwang-juhyun/FinalProject/SmartFactory_AGV_DigitalTwin_Server
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target AGV_Server -j
./build/Server/AGV_Server
```

성공 로그:

```text
맵 데이터 로드 완료 (노드: 3개, 링크: 4개)
[서버] Server-authoritative world created. AGV count=1
[PHYSICAL_DEMO] Automatic scheduling is OFF.
```

배터리, TB6612 VM, `VOUT+ → ESP32 5V`를 계속 분리한 USB 전용 상태에서
ESP32를 연결한다. 접속만으로 ROUTE가 오면 실패다. 서버 콘솔에 직접
`demo12`를 입력한 뒤에만 다음이 나와야 한다.

```text
[RobotProtocol] Send ROUTE_COMMAND agvID=1 ... nodes=2
[PHYSICAL_DEMO] Sent exact route [1 -> 2].
```

ESP32에서는 다음을 확인한다.

```text
node[0]=1
node[1]=2
ROBOT DISARMED
STBY=LOW PWM=0
```

이 단계에서는 아직 실제 주행하지 않는다. ESP32 실행기를 결합한 뒤 완료
시점에 `STATUS(currentNodeID=2, currentLinkID=0, progress=1)`를 먼저 보내고
`ARRIVED(currentNodeID=2)`를 보내야 Unity 위치와 서버 논리 노드가 함께
갱신된다.

## 실제 모터 전원 연결 전 남은 안전 게이트

현재 서버 `NetworkManagerServer::UpdateWorld()`는 `ERROR_SLIP`과
`EMERGENCY_STOP` 이벤트를 큐에서 꺼낸 뒤 처리하지 않는다. 실제 바퀴 구동
전에는 두 이벤트를 로그하고 경로를 취소하며 자동 재시도하지 않는 fault
상태로 처리해야 한다. 또한 ESP32에는 통신 단절 시 로컬 즉시 정지 watchdog이
필수다.
