#include "RobotProtocol.hpp"

#include <cmath>
#include <cstring>

namespace RobotProtocol
{
    PacketWriter::PacketWriter(std::vector<uint8_t>& buffer)
        : m_Buffer(buffer)
    {
    }

    void PacketWriter::writeUInt8(uint8_t value)
    {
        m_Buffer.push_back(value);
    }

    void PacketWriter::writeUInt16(uint16_t value)
    {
        m_Buffer.push_back(static_cast<uint8_t>(value & 0xFF));
        m_Buffer.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
    }

    void PacketWriter::writeUInt32(uint32_t value)
    {
        m_Buffer.push_back(static_cast<uint8_t>(value & 0xFF));
        m_Buffer.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
        m_Buffer.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
        m_Buffer.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
    }

    void PacketWriter::writeFloat(float value)
    {
        static_assert(sizeof(float) == sizeof(uint32_t), "ESP32 float must be 32-bit");
        uint32_t raw = 0;
        std::memcpy(&raw, &value, sizeof(raw));
        writeUInt32(raw);
    }

    void PacketWriter::writeBytes(const uint8_t* data, size_t length)
    {
        if (data != nullptr && length != 0)
            m_Buffer.insert(m_Buffer.end(), data, data + length);
    }

    PacketReader::PacketReader(const uint8_t* data, size_t length)
        : m_Data(data), m_Length(length)
    {
    }

    bool PacketReader::readUInt8(uint8_t& value)
    {
        if (remaining() < sizeof(uint8_t))
            return false;
        value = m_Data[m_Offset++];
        return true;
    }

    bool PacketReader::readUInt16(uint16_t& value)
    {
        if (remaining() < sizeof(uint16_t))
            return false;
        value = static_cast<uint16_t>(m_Data[m_Offset])
              | (static_cast<uint16_t>(m_Data[m_Offset + 1]) << 8);
        m_Offset += sizeof(uint16_t);
        return true;
    }

    bool PacketReader::readUInt32(uint32_t& value)
    {
        if (remaining() < sizeof(uint32_t))
            return false;
        value = static_cast<uint32_t>(m_Data[m_Offset])
              | (static_cast<uint32_t>(m_Data[m_Offset + 1]) << 8)
              | (static_cast<uint32_t>(m_Data[m_Offset + 2]) << 16)
              | (static_cast<uint32_t>(m_Data[m_Offset + 3]) << 24);
        m_Offset += sizeof(uint32_t);
        return true;
    }

    bool PacketReader::readFloat(float& value)
    {
        uint32_t raw = 0;
        if (!readUInt32(raw))
            return false;
        std::memcpy(&value, &raw, sizeof(value));
        return true;
    }

    size_t PacketReader::remaining() const
    {
        return m_Length - m_Offset;
    }

    bool isKnownPacketID(uint16_t rawPacketID)
    {
        switch (static_cast<PacketID>(rawPacketID))
        {
        case PacketID::ROUTE_COMMAND:
        case PacketID::CANCEL_ROUTE:
        case PacketID::TRAJECTORY_COMMAND:
        case PacketID::NODE_CORRECTION_COMMAND:
        case PacketID::STATUS:
        case PacketID::ARRIVED:
        case PacketID::NODE_CORRECTION_REPORT:
        case PacketID::PING:
        case PacketID::PONG:
        case PacketID::HELLO:
        case PacketID::HELLO_ACK:
        case PacketID::ERROR_PACKET:
        case PacketID::EMERGENCY_STOP:
            return true;
        default:
            return false;
        }
    }

    uint16_t readFrameSize(const uint8_t* data)
    {
        return static_cast<uint16_t>(data[0])
             | (static_cast<uint16_t>(data[1]) << 8);
    }

    void writePacketBodyHeader(PacketWriter& writer, PacketID packetID, uint32_t agvID, uint32_t sequence)
    {
        writer.writeUInt16(static_cast<uint16_t>(packetID));
        writer.writeUInt32(agvID);
        writer.writeUInt32(sequence);
    }

    bool readPacketBodyHeader(PacketReader& reader, PacketBodyHeader& outHeader)
    {
        uint16_t rawPacketID = 0;
        if (!reader.readUInt16(rawPacketID) || !isKnownPacketID(rawPacketID))
            return false;
        if (!reader.readUInt32(outHeader.agvID))
            return false;
        if (!reader.readUInt32(outHeader.sequence))
            return false;
        outHeader.packetID = static_cast<PacketID>(rawPacketID);
        return true;
    }

    void writeHelloPayload(PacketWriter& writer, const HelloPayload& payload)
    {
        writer.writeUInt16(payload.protocolVersion);
        writer.writeUInt8(static_cast<uint8_t>(payload.clientType));
        writer.writeUInt32(payload.requestedAgvID);
        // Keep the exact legacy 19-byte HELLO frame when no extension is
        // advertised. The Server accepts this optional trailing uint32.
        if (payload.capabilities != CAPABILITY_NONE)
            writer.writeUInt32(payload.capabilities);
    }

    bool readHelloAckPayload(PacketReader& reader, HelloAckPayload& outPayload)
    {
        uint16_t rawErrorCode = 0;
        if (!reader.readUInt16(outPayload.protocolVersion))
            return false;
        if (!reader.readUInt8(outPayload.accepted))
            return false;
        if (!reader.readUInt32(outPayload.assignedAgvID))
            return false;
        if (!reader.readUInt16(rawErrorCode))
            return false;
        outPayload.errorCode = static_cast<ErrorCode>(rawErrorCode);
        return true;
    }

    void writeStatusPayload(PacketWriter& writer, const StatusPayload& payload)
    {
        writer.writeUInt32(payload.currentNodeID);
        writer.writeUInt32(payload.currentLinkID);
        writer.writeFloat(payload.progress);
        writer.writeFloat(payload.x);
        writer.writeFloat(payload.z);
        writer.writeFloat(payload.heading);
        writer.writeFloat(payload.velocity);
        writer.writeFloat(payload.battery);
        writer.writeUInt8(static_cast<uint8_t>(payload.state));
    }

    void writeArrivedPayload(PacketWriter& writer, const ArrivedPayload& payload)
    {
        writer.writeUInt32(payload.currentNodeID);
    }

    bool readRouteCommandPayload(PacketReader& reader, RouteCommandPayload& outPayload)
    {
        if (!reader.readUInt32(outPayload.routeID))
            return false;
        if (!reader.readUInt16(outPayload.nodeCount))
            return false;
        if (outPayload.nodeCount > kMaxRouteNodes)
            return false;

        for (uint16_t i = 0; i < outPayload.nodeCount; ++i)
        {
            if (!reader.readUInt32(outPayload.nodes[i].nodeID))
                return false;
            if (!reader.readFloat(outPayload.nodes[i].arrivalTime))
                return false;
            if (!reader.readFloat(outPayload.nodes[i].departureTime))
                return false;
        }
        return true;
    }

    bool readTrajectoryCommandPayload(PacketReader& reader,
                                      TrajectoryCommandPayload& outPayload)
    {
        outPayload = {};
        if (!reader.readUInt32(outPayload.routeID))
            return false;
        if (!reader.readUInt8(outPayload.formatVersion))
            return false;
        if (outPayload.formatVersion != kTrajectoryFormatVersion)
            return false;
        if (!reader.readUInt16(outPayload.waypointCount))
            return false;
        if (outPayload.waypointCount == 0
            || outPayload.waypointCount > kMaxTrajectoryWaypoints)
        {
            return false;
        }
        if (!reader.readUInt32(outPayload.startNodeID))
            return false;
        if (!reader.readUInt32(outPayload.finalNodeID))
            return false;
        if (!reader.readFloat(outPayload.millimetersPerMapUnit)
            || !std::isfinite(outPayload.millimetersPerMapUnit)
            || outPayload.millimetersPerMapUnit <= 0.0f)
        {
            return false;
        }

        constexpr uint8_t kKnownFlags = TRAJECTORY_FLAG_NODE_BOUNDARY
            | TRAJECTORY_FLAG_STOP
            | TRAJECTORY_FLAG_ROTATE_IN_PLACE
            | TRAJECTORY_FLAG_FINAL;

        for (uint16_t i = 0; i < outPayload.waypointCount; ++i)
        {
            TrajectoryWaypoint& waypoint = outPayload.waypoints[i];
            if (!reader.readFloat(waypoint.forwardMm)
                || !reader.readFloat(waypoint.leftMm)
                || !reader.readFloat(waypoint.headingRad)
                || !reader.readFloat(waypoint.targetSpeedMmPerSecond)
                || !reader.readUInt32(waypoint.nodeID)
                || !reader.readUInt8(waypoint.flags))
            {
                return false;
            }
            if (!std::isfinite(waypoint.forwardMm)
                || !std::isfinite(waypoint.leftMm)
                || !std::isfinite(waypoint.headingRad)
                || !std::isfinite(waypoint.targetSpeedMmPerSecond)
                || waypoint.targetSpeedMmPerSecond < 0.0f
                || (waypoint.flags & static_cast<uint8_t>(~kKnownFlags)) != 0)
            {
                return false;
            }
        }

        return reader.remaining() == 0;
    }

    bool readNodeCorrectionCommandPayload(
        PacketReader& reader,
        NodeCorrectionCommandPayload& outPayload)
    {
        outPayload = {};
        uint8_t rawAction = 0;
        if (!reader.readUInt32(outPayload.routeID)
            || !reader.readUInt32(outPayload.nodeID)
            || !reader.readUInt32(outPayload.commandID)
            || !reader.readUInt8(rawAction)
            || !reader.readFloat(outPayload.magnitude)
            || !std::isfinite(outPayload.magnitude)
            || reader.remaining() != 0)
        {
            return false;
        }

        switch (static_cast<NodeCorrectionAction>(rawAction))
        {
        case NodeCorrectionAction::DRIVE_FORWARD:
        case NodeCorrectionAction::TURN_CW:
        case NodeCorrectionAction::TURN_CCW:
            outPayload.action = static_cast<NodeCorrectionAction>(rawAction);
            return true;
        default:
            return false;
        }
    }

    void writeNodeCorrectionReportPayload(
        PacketWriter& writer,
        const NodeCorrectionReportPayload& payload)
    {
        writer.writeUInt32(payload.routeID);
        writer.writeUInt32(payload.nodeID);
        writer.writeUInt32(payload.commandID);
        writer.writeUInt8(static_cast<uint8_t>(payload.result));
        writer.writeUInt32(payload.detail);
    }

    void writeErrorPayload(PacketWriter& writer, const ErrorPayload& payload)
    {
        writer.writeUInt16(static_cast<uint16_t>(payload.errorCode));
        writer.writeUInt32(payload.detail);
    }

    void writeTimePayload(PacketWriter& writer, const TimePayload& payload)
    {
        writer.writeUInt32(payload.timestampMs);
    }

    bool readTimePayload(PacketReader& reader, TimePayload& outPayload)
    {
        return reader.readUInt32(outPayload.timestampMs);
    }

    bool buildFrame(PacketID packetID,
                    uint32_t agvID,
                    uint32_t sequence,
                    const std::vector<uint8_t>& payload,
                    std::vector<uint8_t>& outFrame)
    {
        const size_t totalSize = kFrameHeaderSize + kPacketBodyHeaderSize + payload.size();
        if (totalSize > UINT16_MAX || totalSize > kMaxFrameSize)
            return false;

        outFrame.clear();
        outFrame.reserve(totalSize);
        PacketWriter writer(outFrame);
        writer.writeUInt16(static_cast<uint16_t>(totalSize));
        writePacketBodyHeader(writer, packetID, agvID, sequence);
        writer.writeBytes(payload.data(), payload.size());
        return true;
    }
}
