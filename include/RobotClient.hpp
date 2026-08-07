#pragma once

#include "RobotProtocol.hpp"

#include <WiFi.h>
#include <functional>
#include <vector>

class RobotClient
{
public:
    enum class SessionState : uint8_t
    {
        DISCONNECTED,
        WAIT_HELLO_ACK,
        ACCEPTED,
        REJECTED
    };

    void begin(const char* ssid,
               const char* password,
               const char* serverHost,
               uint16_t serverPort,
               uint32_t requestedAgvID);

    void update();
    bool connected();
    bool accepted() const { return m_SessionState == SessionState::ACCEPTED; }
    SessionState sessionState() const { return m_SessionState; }
    uint32_t agvID() const { return m_AgvID; }
    bool sendStatus(const RobotProtocol::StatusPayload& status);
    bool sendArrived(uint32_t currentNodeID);
    bool sendPong(uint32_t timestampMs);
    bool sendError(RobotProtocol::ErrorCode errorCode, uint32_t detail);

    std::function<void(uint32_t agvID)> onAccepted;
    std::function<void()> onDisconnected;
    std::function<void(const RobotProtocol::RouteCommandPayload& route)> onRouteCommand;
    std::function<void()> onCancelRoute;
    std::function<void()> onEmergencyStop;

private:
    void connectIfNeeded(uint32_t nowMs);
    void dropConnection(const char* reason);
    void readIncoming();
    void processFrames();
    void handleBody(const uint8_t* body, size_t length);
    bool sendHello();
    bool sendPacket(RobotProtocol::PacketID packetID, const std::vector<uint8_t>& payload);

    const char* m_Ssid = nullptr;
    const char* m_Password = nullptr;
    const char* m_ServerHost = nullptr;
    uint16_t m_ServerPort = 0;
    uint32_t m_RequestedAgvID = 0;
    uint32_t m_AgvID = 0;
    uint32_t m_NextSequence = 1;
    uint32_t m_LastReconnectAttemptMs = 0;
    uint32_t m_HelloSentAtMs = 0;
    SessionState m_SessionState = SessionState::DISCONNECTED;
    bool m_SocketActive = false;
    WiFiClient m_Client;
    std::vector<uint8_t> m_RxBuffer;
};
