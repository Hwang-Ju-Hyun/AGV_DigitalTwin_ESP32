#include "RobotClient.hpp"

#include "Config.hpp"
#include <Arduino.h>

void RobotClient::begin(const char* ssid,
                        const char* password,
                        const char* serverHost,
                        uint16_t serverPort,
                        uint32_t requestedAgvID)
{
    m_Ssid = ssid;
    m_Password = password;
    m_ServerHost = serverHost;
    m_ServerPort = serverPort;
    m_RequestedAgvID = requestedAgvID;
    m_AgvID = requestedAgvID;
    m_RxBuffer.reserve(512);

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(m_Ssid, m_Password);
    Serial.printf("[WiFi] Connecting to %s\n", m_Ssid);
}

void RobotClient::update()
{
    connectIfNeeded(millis());
    if (!connected())
        return;
    readIncoming();
    processFrames();
}

bool RobotClient::connected()
{
    return m_Client.connected();
}

bool RobotClient::sendHello()
{
    RobotProtocol::HelloPayload hello;
    hello.requestedAgvID = m_RequestedAgvID;

    std::vector<uint8_t> payload;
    payload.reserve(7);
    RobotProtocol::PacketWriter writer(payload);
    RobotProtocol::writeHelloPayload(writer, hello);
    return sendPacket(RobotProtocol::PacketID::HELLO, payload);
}

bool RobotClient::sendStatus(const RobotProtocol::StatusPayload& status)
{
    std::vector<uint8_t> payload;
    payload.reserve(33);
    RobotProtocol::PacketWriter writer(payload);
    RobotProtocol::writeStatusPayload(writer, status);
    return sendPacket(RobotProtocol::PacketID::STATUS, payload);
}

bool RobotClient::sendArrived(uint32_t currentNodeID)
{
    RobotProtocol::ArrivedPayload arrived;
    arrived.currentNodeID = currentNodeID;

    std::vector<uint8_t> payload;
    payload.reserve(4);
    RobotProtocol::PacketWriter writer(payload);
    RobotProtocol::writeArrivedPayload(writer, arrived);
    return sendPacket(RobotProtocol::PacketID::ARRIVED, payload);
}

bool RobotClient::sendPong(uint32_t timestampMs)
{
    RobotProtocol::TimePayload pong;
    pong.timestampMs = timestampMs;

    std::vector<uint8_t> payload;
    payload.reserve(4);
    RobotProtocol::PacketWriter writer(payload);
    RobotProtocol::writeTimePayload(writer, pong);
    return sendPacket(RobotProtocol::PacketID::PONG, payload);
}

void RobotClient::connectIfNeeded(uint32_t nowMs)
{
    if (WiFi.status() != WL_CONNECTED)
    {
        if (m_Client.connected())
            m_Client.stop();
        if (m_WasConnected)
            Serial.println("[TCP] Disconnected");
        m_WasConnected = false;
        m_Accepted = false;
        return;
    }

    if (m_Client.connected())
    {
        m_WasConnected = true;
        return;
    }

    if (m_WasConnected)
        Serial.println("[TCP] Disconnected");
    m_WasConnected = false;
    m_Accepted = false;

    if (nowMs - m_LastReconnectAttemptMs < AppConfig::kReconnectIntervalMs)
        return;

    m_LastReconnectAttemptMs = nowMs;
    m_RxBuffer.clear();

    Serial.printf("[WiFi] Connected, IP=%s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[TCP] Connecting to %s:%u\n", m_ServerHost, m_ServerPort);
    if (!m_Client.connect(m_ServerHost, m_ServerPort))
    {
        Serial.println("[TCP] Connect failed");
        return;
    }

    m_WasConnected = true;
    Serial.println("[TCP] Connected, sending HELLO");
    if (!sendHello())
        Serial.println("[RobotProtocol] HELLO send failed");
}

void RobotClient::readIncoming()
{
    uint8_t temp[128];
    while (m_Client.available() > 0)
    {
        const int available = m_Client.available();
        const size_t requestSize = available < static_cast<int>(sizeof(temp))
            ? static_cast<size_t>(available)
            : sizeof(temp);
        const int count = m_Client.read(temp, requestSize);
        if (count <= 0)
            break;
        m_RxBuffer.insert(m_RxBuffer.end(), temp, temp + count);
        if (m_RxBuffer.size() > RobotProtocol::kMaxFrameSize * 2U)
        {
            Serial.println("[TCP] RX buffer limit exceeded; disconnecting");
            m_RxBuffer.clear();
            m_Client.stop();
            m_Accepted = false;
            return;
        }
    }
}

void RobotClient::processFrames()
{
    while (m_RxBuffer.size() >= RobotProtocol::kFrameHeaderSize)
    {
        const uint16_t frameSize = RobotProtocol::readFrameSize(m_RxBuffer.data());
        const uint16_t minimumSize = RobotProtocol::kFrameHeaderSize + RobotProtocol::kPacketBodyHeaderSize;
        if (frameSize < minimumSize || frameSize > RobotProtocol::kMaxFrameSize)
        {
            Serial.printf("[TCP] Invalid frame size: %u\n", frameSize);
            m_RxBuffer.clear();
            m_Client.stop();
            m_Accepted = false;
            return;
        }
        if (m_RxBuffer.size() < frameSize)
            return;

        const uint8_t* body = m_RxBuffer.data() + RobotProtocol::kFrameHeaderSize;
        const size_t bodyLength = frameSize - RobotProtocol::kFrameHeaderSize;
        handleBody(body, bodyLength);
        m_RxBuffer.erase(m_RxBuffer.begin(), m_RxBuffer.begin() + frameSize);
    }
}

void RobotClient::handleBody(const uint8_t* body, size_t length)
{
    RobotProtocol::PacketReader reader(body, length);
    RobotProtocol::PacketBodyHeader header;
    if (!RobotProtocol::readPacketBodyHeader(reader, header))
    {
        Serial.println("[RobotProtocol] Invalid packet header");
        return;
    }

    switch (header.packetID)
    {
    case RobotProtocol::PacketID::HELLO_ACK:
    {
        RobotProtocol::HelloAckPayload ack;
        if (!RobotProtocol::readHelloAckPayload(reader, ack))
        {
            Serial.println("[RobotProtocol] Invalid HELLO_ACK");
            return;
        }
        m_Accepted = ack.accepted != 0 && ack.protocolVersion == RobotProtocol::kProtocolVersion;
        if (ack.assignedAgvID != 0)
            m_AgvID = ack.assignedAgvID;
        Serial.printf("[RobotProtocol] HELLO_ACK accepted=%u agvID=%lu error=%u\n",
                      m_Accepted ? 1U : 0U,
                      static_cast<unsigned long>(m_AgvID),
                      static_cast<unsigned>(ack.errorCode));
        if (m_Accepted && onAccepted)
            onAccepted(m_AgvID);
        break;
    }
    case RobotProtocol::PacketID::ROUTE_COMMAND:
    {
        RobotProtocol::RouteCommandPayload route;
        if (!RobotProtocol::readRouteCommandPayload(reader, route))
        {
            Serial.println("[RobotProtocol] Invalid ROUTE_COMMAND");
            return;
        }
        Serial.printf("[RobotProtocol] ROUTE routeID=%lu nodes=%u\n",
                      static_cast<unsigned long>(route.routeID), route.nodeCount);
        if (onRouteCommand)
            onRouteCommand(route);
        break;
    }
    case RobotProtocol::PacketID::CANCEL_ROUTE:
        Serial.println("[RobotProtocol] CANCEL_ROUTE");
        if (onCancelRoute)
            onCancelRoute();
        break;
    case RobotProtocol::PacketID::PING:
    {
        RobotProtocol::TimePayload ping;
        if (RobotProtocol::readTimePayload(reader, ping))
            sendPong(ping.timestampMs);
        break;
    }
    case RobotProtocol::PacketID::EMERGENCY_STOP:
        Serial.println("[RobotProtocol] EMERGENCY_STOP");
        if (onEmergencyStop)
            onEmergencyStop();
        break;
    default:
        break;
    }
}

bool RobotClient::sendPacket(RobotProtocol::PacketID packetID, const std::vector<uint8_t>& payload)
{
    if (!connected())
        return false;

    std::vector<uint8_t> frame;
    if (!RobotProtocol::buildFrame(packetID, m_AgvID, m_NextSequence++, payload, frame))
        return false;

    size_t offset = 0;
    const uint32_t startedAt = millis();
    while (offset < frame.size() && connected() && millis() - startedAt < 1000)
    {
        const size_t written = m_Client.write(frame.data() + offset, frame.size() - offset);
        if (written == 0)
        {
            delay(1);
            continue;
        }
        offset += written;
    }
    return offset == frame.size();
}
