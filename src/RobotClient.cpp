#include "RobotClient.hpp"

#include "Config.hpp"
#include <Arduino.h>
#include <cerrno>
#include <lwip/sockets.h>

namespace
{
    // Bound network work per application loop so encoder and safety checks
    // cannot be starved by a burst of valid TCP data.
    constexpr size_t kMaxRxBytesPerUpdate = 512;
    constexpr size_t kMaxFramesPerUpdate = 4;
    constexpr size_t kMaxPendingTxBytes = 4096;
    constexpr uint32_t kTxStallTimeoutMs = 750;
}

void RobotClient::begin(const char* ssid,
                        const char* password,
                        const char* serverHost,
                        uint16_t serverPort,
                        uint32_t requestedAgvID,
                        uint32_t capabilities)
{
    m_Ssid = ssid;
    m_Password = password;
    m_ServerHost = serverHost;
    m_ServerPort = serverPort;
    m_RequestedAgvID = requestedAgvID;
    m_Capabilities = capabilities;
    m_AgvID = requestedAgvID;
    m_RxBuffer.reserve(512);
    m_TxBuffer.reserve(kMaxPendingTxBytes);

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(m_Ssid, m_Password);
    Serial.println("[WiFi] Connecting");
}

void RobotClient::update()
{
    connectIfNeeded(millis());
    if (!connected())
        return;

    flushOutgoing(millis());
    if (!connected())
        return;

    readIncoming();
    if (!connected())
        return;

    processFrames();
    if (!connected())
        return;

    flushOutgoing(millis());
    if (!connected())
        return;

    if (m_SessionState == SessionState::WAIT_HELLO_ACK
        && millis() - m_HelloSentAtMs >= AppConfig::kHelloAckTimeoutMs)
    {
        dropConnection("[RobotProtocol] HELLO_ACK timeout; reconnecting");
    }
}

bool RobotClient::connected()
{
    return m_Client.connected();
}

bool RobotClient::sendHello()
{
    RobotProtocol::HelloPayload hello;
    hello.requestedAgvID = m_RequestedAgvID;
    hello.capabilities = m_Capabilities;

    std::vector<uint8_t> payload;
    payload.reserve(m_Capabilities == RobotProtocol::CAPABILITY_NONE ? 7 : 11);
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

bool RobotClient::sendNodeCorrectionReport(
    const RobotProtocol::NodeCorrectionReportPayload& report)
{
    std::vector<uint8_t> payload;
    payload.reserve(RobotProtocol::kNodeCorrectionReportPayloadSize);
    RobotProtocol::PacketWriter writer(payload);
    RobotProtocol::writeNodeCorrectionReportPayload(writer, report);
    return sendPacket(RobotProtocol::PacketID::NODE_CORRECTION_REPORT,
                      payload);
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

bool RobotClient::sendError(RobotProtocol::ErrorCode errorCode, uint32_t detail)
{
    RobotProtocol::ErrorPayload error;
    error.errorCode = errorCode;
    error.detail = detail;

    std::vector<uint8_t> payload;
    payload.reserve(6);
    RobotProtocol::PacketWriter writer(payload);
    RobotProtocol::writeErrorPayload(writer, error);
    return sendPacket(RobotProtocol::PacketID::ERROR_PACKET, payload);
}

void RobotClient::connectIfNeeded(uint32_t nowMs)
{
    if (WiFi.status() != WL_CONNECTED)
    {
        if (m_SocketActive || m_Client.connected())
            dropConnection("[TCP] Disconnected: Wi-Fi unavailable");
        return;
    }

    if (m_Client.connected())
        return;

    if (m_SocketActive)
        dropConnection("[TCP] Disconnected");

    if (nowMs - m_LastReconnectAttemptMs < AppConfig::kReconnectIntervalMs)
        return;

    m_LastReconnectAttemptMs = nowMs;
    m_RxBuffer.clear();

    Serial.printf("[WiFi] Connected, IP=%s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[TCP] Connecting to %s:%u\n", m_ServerHost, m_ServerPort);
    if (!m_Client.connect(m_ServerHost, m_ServerPort))
    {
        // Some WiFiClient implementations retain a failed socket handle.
        // Always discard it so the next attempt creates a fresh TCP stream.
        m_Client.stop();
        Serial.println("[TCP] Connect failed");
        return;
    }

    m_SocketActive = true;
    m_SessionState = SessionState::WAIT_HELLO_ACK;
    Serial.println("[TCP] Connected");
    if (!sendHello())
        return;

    m_HelloSentAtMs = millis();
    Serial.println("[RobotProtocol] HELLO sent; waiting for HELLO_ACK");
}

void RobotClient::dropConnection(const char* reason)
{
    const bool notify = m_SocketActive;

    m_HelloSentAtMs = 0;
    m_SessionState = SessionState::DISCONNECTED;
    m_SocketActive = false;

    // Stop motion before socket cleanup or potentially slow serial logging.
    if (notify && onDisconnected)
        onDisconnected();

    m_Client.stop();
    m_RxBuffer.clear();
    m_TxBuffer.clear();
    m_TxOffset = 0;
    m_TxBlockedSinceMs = 0;
    m_AgvID = m_RequestedAgvID;

    if (reason != nullptr)
        Serial.println(reason);
}

void RobotClient::flushOutgoing(uint32_t nowMs)
{
    if (!m_SocketActive || !connected() ||
        m_TxOffset >= m_TxBuffer.size())
    {
        if (m_TxOffset >= m_TxBuffer.size())
        {
            m_TxBuffer.clear();
            m_TxOffset = 0;
            m_TxBlockedSinceMs = 0;
        }
        return;
    }

    const int socketFd = m_Client.fd();
    if (socketFd < 0)
    {
        dropConnection("[TCP] Invalid socket during send; reconnecting");
        return;
    }

    const int written = ::send(
        socketFd,
        m_TxBuffer.data() + m_TxOffset,
        m_TxBuffer.size() - m_TxOffset,
        MSG_DONTWAIT);
    if (written > 0)
    {
        m_TxOffset += static_cast<size_t>(written);
        m_TxBlockedSinceMs = 0;
        if (m_TxOffset >= m_TxBuffer.size())
        {
            m_TxBuffer.clear();
            m_TxOffset = 0;
        }
        return;
    }

    if (written < 0 &&
        (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
    {
        if (m_TxBlockedSinceMs == 0)
            m_TxBlockedSinceMs = nowMs;
        else if (nowMs - m_TxBlockedSinceMs >= kTxStallTimeoutMs)
            dropConnection("[TCP] TX stalled; reconnecting");
        return;
    }

    dropConnection("[TCP] Packet write failed; reconnecting");
}

void RobotClient::readIncoming()
{
    uint8_t temp[128];
    size_t remainingBudget = kMaxRxBytesPerUpdate;
    while (m_Client.available() > 0 && remainingBudget > 0)
    {
        const int available = m_Client.available();
        size_t requestSize = available < static_cast<int>(sizeof(temp))
            ? static_cast<size_t>(available)
            : sizeof(temp);
        if (requestSize > remainingBudget)
            requestSize = remainingBudget;

        const int count = m_Client.read(temp, requestSize);
        if (count <= 0)
        {
            if (!m_Client.connected() && m_SocketActive)
                dropConnection("[TCP] Read failed; reconnecting");
            break;
        }
        m_RxBuffer.insert(m_RxBuffer.end(), temp, temp + count);
        remainingBudget -= static_cast<size_t>(count);
        if (m_RxBuffer.size() > RobotProtocol::kMaxFrameSize * 2U)
        {
            dropConnection("[TCP] RX buffer limit exceeded; reconnecting");
            return;
        }
    }
}

void RobotClient::processFrames()
{
    size_t processedFrames = 0;
    while (m_RxBuffer.size() >= RobotProtocol::kFrameHeaderSize
           && processedFrames < kMaxFramesPerUpdate)
    {
        const uint16_t frameSize = RobotProtocol::readFrameSize(m_RxBuffer.data());
        const uint16_t minimumSize = RobotProtocol::kFrameHeaderSize + RobotProtocol::kPacketBodyHeaderSize;
        if (frameSize < minimumSize || frameSize > RobotProtocol::kMaxFrameSize)
        {
            dropConnection(nullptr);
            Serial.printf("[TCP] Invalid frame size: %u; reconnecting\n", frameSize);
            return;
        }
        if (m_RxBuffer.size() < frameSize)
            return;

        const uint8_t* body = m_RxBuffer.data() + RobotProtocol::kFrameHeaderSize;
        const size_t bodyLength = frameSize - RobotProtocol::kFrameHeaderSize;
        handleBody(body, bodyLength);
        if (!connected())
            return;
        m_RxBuffer.erase(m_RxBuffer.begin(), m_RxBuffer.begin() + frameSize);
        ++processedFrames;
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
        if (m_SessionState != SessionState::WAIT_HELLO_ACK)
        {
            Serial.println("[RobotProtocol] Unexpected HELLO_ACK ignored");
            return;
        }

        RobotProtocol::HelloAckPayload ack;
        if (!RobotProtocol::readHelloAckPayload(reader, ack))
        {
            Serial.println("[RobotProtocol] Invalid HELLO_ACK");
            return;
        }
        const bool accepted = ack.accepted != 0
            && ack.protocolVersion == RobotProtocol::kProtocolVersion;
        m_SessionState = accepted ? SessionState::ACCEPTED : SessionState::REJECTED;
        if (ack.assignedAgvID != 0)
            m_AgvID = ack.assignedAgvID;
        Serial.printf("[RobotProtocol] HELLO_ACK accepted=%u agvID=%lu error=%u\n",
                      accepted ? 1U : 0U,
                      static_cast<unsigned long>(m_AgvID),
                      static_cast<unsigned>(ack.errorCode));
        if (accepted && onAccepted)
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
    case RobotProtocol::PacketID::TRAJECTORY_COMMAND:
    {
        if (m_SessionState != SessionState::ACCEPTED
            || header.agvID != m_AgvID)
        {
            Serial.println("[RobotProtocol] Unauthorized TRAJECTORY_COMMAND ignored");
            return;
        }
        constexpr uint32_t kTrajectoryReceiveCapabilities =
            RobotProtocol::CAPABILITY_TRAJECTORY_COMMAND
            | RobotProtocol::CAPABILITY_TRAJECTORY_PREVIEW;
        if ((m_Capabilities & kTrajectoryReceiveCapabilities) == 0)
        {
            Serial.println(
                "[RobotProtocol] Unsupported TRAJECTORY_COMMAND ignored");
            return;
        }

        RobotProtocol::TrajectoryCommandPayload trajectory;
        if (!RobotProtocol::readTrajectoryCommandPayload(reader, trajectory))
        {
            Serial.println("[RobotProtocol] Invalid TRAJECTORY_COMMAND");
            return;
        }
        Serial.printf(
            "[RobotProtocol] TRAJECTORY routeID=%lu format=%u waypoints=%u\n",
            static_cast<unsigned long>(trajectory.routeID),
            trajectory.formatVersion,
            trajectory.waypointCount);
        if (onTrajectoryCommand)
            onTrajectoryCommand(trajectory);
        break;
    }
    case RobotProtocol::PacketID::NODE_CORRECTION_COMMAND:
    {
        if (m_SessionState != SessionState::ACCEPTED
            || header.agvID != m_AgvID)
        {
            Serial.println(
                "[RobotProtocol] Unauthorized NODE_CORRECTION_COMMAND ignored");
            return;
        }
        if ((m_Capabilities & RobotProtocol::CAPABILITY_NODE_CORRECTION) == 0)
        {
            Serial.println(
                "[RobotProtocol] Unsupported NODE_CORRECTION_COMMAND ignored");
            return;
        }

        RobotProtocol::NodeCorrectionCommandPayload correction;
        if (!RobotProtocol::readNodeCorrectionCommandPayload(
                reader, correction))
        {
            Serial.println(
                "[RobotProtocol] Invalid NODE_CORRECTION_COMMAND");
            return;
        }
        Serial.printf(
            "[RobotProtocol] CORRECTION routeID=%lu node=%lu "
            "commandID=%lu action=%u magnitude=%.3f\n",
            static_cast<unsigned long>(correction.routeID),
            static_cast<unsigned long>(correction.nodeID),
            static_cast<unsigned long>(correction.commandID),
            static_cast<unsigned>(correction.action),
            correction.magnitude);
        if (onNodeCorrectionCommand)
            onNodeCorrectionCommand(correction);
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
    if (!m_SocketActive || !connected())
    {
        if (m_SocketActive)
            dropConnection("[TCP] Send attempted on a closed socket; reconnecting");
        return false;
    }

    const size_t pendingBytes = m_TxBuffer.size() - m_TxOffset;
    if (packetID == RobotProtocol::PacketID::STATUS && pendingBytes != 0)
    {
        // STATUS is periodic. Keep an older queued frame instead of allowing
        // telemetry to crowd out ARRIVED or correction reports.
        return true;
    }

    std::vector<uint8_t> frame;
    if (!RobotProtocol::buildFrame(packetID, m_AgvID, m_NextSequence, payload, frame))
    {
        dropConnection("[RobotProtocol] Frame build failed; reconnecting");
        return false;
    }

    if (pendingBytes + frame.size() > kMaxPendingTxBytes)
    {
        dropConnection("[TCP] TX queue limit exceeded; reconnecting");
        return false;
    }

    if (m_TxOffset != 0)
    {
        m_TxBuffer.erase(
            m_TxBuffer.begin(), m_TxBuffer.begin() + m_TxOffset);
        m_TxOffset = 0;
    }
    m_TxBuffer.insert(m_TxBuffer.end(), frame.begin(), frame.end());
    ++m_NextSequence;

    // Try immediately, but retain a partial frame for later bounded flushes.
    // A real write failure or a sustained stall still drops the connection,
    // which invokes the existing fail-safe motion stop.
    flushOutgoing(millis());
    return m_SocketActive;
}
