#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace RobotProtocol
{
    constexpr uint16_t kProtocolVersion = 1;
    constexpr uint16_t kMaxRouteNodes = 64;
    constexpr uint8_t kTrajectoryFormatVersion = 1;
    constexpr uint16_t kMaxTrajectoryWaypoints = 64;
    constexpr uint16_t kPacketBodyHeaderSize = sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t);
    constexpr uint16_t kFrameHeaderSize = sizeof(uint16_t);
    constexpr uint16_t kMaxFrameSize = 2048;
    constexpr uint16_t kTrajectoryFixedPayloadSize = 19;
    constexpr uint16_t kTrajectoryWaypointWireSize = 21;
    constexpr uint16_t kMaxTrajectoryPayloadSize =
        kTrajectoryFixedPayloadSize
        + kMaxTrajectoryWaypoints * kTrajectoryWaypointWireSize;
    static_assert(kFrameHeaderSize + kPacketBodyHeaderSize
                      + kMaxTrajectoryPayloadSize
                      <= kMaxFrameSize,
                  "Maximum trajectory frame exceeds the ESP32 receive limit");

    enum ClientCapability : uint32_t
    {
        CAPABILITY_NONE = 0,
        // Full follower support is intentionally not advertised by the
        // current firmware.
        CAPABILITY_TRAJECTORY_COMMAND = 1u << 0,
        // Parse/validate/store only. This cannot start any motor path.
        CAPABILITY_TRAJECTORY_PREVIEW = 1u << 1
    };

    enum class ClientType : uint8_t
    {
        UNKNOWN = 0,
        UNITY = 1,
        ESP32_ROBOT = 2,
        TOOL = 3,
        FAKE_ROBOT = 4
    };

    enum class PacketID : uint16_t
    {
        ROUTE_COMMAND = 100,
        CANCEL_ROUTE = 101,
        TRAJECTORY_COMMAND = 102,
        STATUS = 200,
        ARRIVED = 201,
        PING = 300,
        PONG = 301,
        HELLO = 400,
        HELLO_ACK = 401,
        ERROR_PACKET = 500,
        EMERGENCY_STOP = 501
    };

    enum class RobotState : uint8_t
    {
        UNKNOWN = 0,
        IDLE = 1,
        MOVING = 2,
        LOADING = 3,
        UNLOADING = 4,
        WAIT_REPLAN = 5,
        FAULT = 100,
        EMERGENCY_STOPPED = 101
    };

    enum class ErrorCode : uint16_t
    {
        NONE = 0,
        PROTOCOL_MISMATCH = 1,
        UNKNOWN_AGV = 2,
        MOTOR_FAULT = 100,
        LOW_BATTERY = 101,
        OBSTACLE_DETECTED = 102
    };

    struct PacketBodyHeader
    {
        PacketID packetID = PacketID::ERROR_PACKET;
        uint32_t agvID = 0;
        uint32_t sequence = 0;
    };

    struct RouteNodeTime
    {
        uint32_t nodeID = 0;
        float arrivalTime = 0.0f;
        float departureTime = 0.0f;
    };

    struct RouteCommandPayload
    {
        uint32_t routeID = 0;
        uint16_t nodeCount = 0;
        RouteNodeTime nodes[kMaxRouteNodes];
    };

    enum TrajectoryWaypointFlag : uint8_t
    {
        TRAJECTORY_FLAG_NONE = 0,
        TRAJECTORY_FLAG_NODE_BOUNDARY = 1u << 0,
        TRAJECTORY_FLAG_STOP = 1u << 1,
        TRAJECTORY_FLAG_ROTATE_IN_PLACE = 1u << 2,
        TRAJECTORY_FLAG_FINAL = 1u << 3
    };

    struct TrajectoryWaypoint
    {
        // Format v1 robot-local frame: +forward is the robot's trusted start
        // heading, +left is counter-clockwise 90 degrees from +forward.
        float forwardMm = 0.0f;
        float leftMm = 0.0f;
        float headingRad = 0.0f;
        float targetSpeedMmPerSecond = 0.0f;
        uint32_t nodeID = 0;
        uint8_t flags = TRAJECTORY_FLAG_NONE;
    };

    struct TrajectoryCommandPayload
    {
        uint32_t routeID = 0;
        uint8_t formatVersion = 0;
        uint16_t waypointCount = 0;
        uint32_t startNodeID = 0;
        uint32_t finalNodeID = 0;
        float millimetersPerMapUnit = 0.0f;
        TrajectoryWaypoint waypoints[kMaxTrajectoryWaypoints];
    };

    struct HelloPayload
    {
        uint16_t protocolVersion = kProtocolVersion;
        ClientType clientType = ClientType::ESP32_ROBOT;
        uint32_t requestedAgvID = 0;
        uint32_t capabilities = CAPABILITY_NONE;
    };

    struct HelloAckPayload
    {
        uint16_t protocolVersion = kProtocolVersion;
        uint8_t accepted = 0;
        uint32_t assignedAgvID = 0;
        ErrorCode errorCode = ErrorCode::NONE;
    };

    struct StatusPayload
    {
        uint32_t currentNodeID = 0;
        uint32_t currentLinkID = 0;
        float progress = 0.0f;
        float x = 0.0f;
        float z = 0.0f;
        float heading = 0.0f;
        float velocity = 0.0f;
        float battery = 100.0f;
        RobotState state = RobotState::UNKNOWN;
    };

    struct ArrivedPayload
    {
        uint32_t currentNodeID = 0;
    };

    struct ErrorPayload
    {
        ErrorCode errorCode = ErrorCode::NONE;
        uint32_t detail = 0;
    };

    struct TimePayload
    {
        uint32_t timestampMs = 0;
    };

    class PacketWriter
    {
    public:
        explicit PacketWriter(std::vector<uint8_t>& buffer);
        void writeUInt8(uint8_t value);
        void writeUInt16(uint16_t value);
        void writeUInt32(uint32_t value);
        void writeFloat(float value);
        void writeBytes(const uint8_t* data, size_t length);

    private:
        std::vector<uint8_t>& m_Buffer;
    };

    class PacketReader
    {
    public:
        PacketReader(const uint8_t* data, size_t length);
        bool readUInt8(uint8_t& value);
        bool readUInt16(uint16_t& value);
        bool readUInt32(uint32_t& value);
        bool readFloat(float& value);
        size_t remaining() const;

    private:
        const uint8_t* m_Data = nullptr;
        size_t m_Length = 0;
        size_t m_Offset = 0;
    };

    bool isKnownPacketID(uint16_t rawPacketID);
    uint16_t readFrameSize(const uint8_t* data);
    void writePacketBodyHeader(PacketWriter& writer, PacketID packetID, uint32_t agvID, uint32_t sequence);
    bool readPacketBodyHeader(PacketReader& reader, PacketBodyHeader& outHeader);
    void writeHelloPayload(PacketWriter& writer, const HelloPayload& payload);
    bool readHelloAckPayload(PacketReader& reader, HelloAckPayload& outPayload);
    void writeStatusPayload(PacketWriter& writer, const StatusPayload& payload);
    void writeArrivedPayload(PacketWriter& writer, const ArrivedPayload& payload);
    bool readRouteCommandPayload(PacketReader& reader, RouteCommandPayload& outPayload);
    bool readTrajectoryCommandPayload(PacketReader& reader,
                                      TrajectoryCommandPayload& outPayload);
    void writeErrorPayload(PacketWriter& writer, const ErrorPayload& payload);
    void writeTimePayload(PacketWriter& writer, const TimePayload& payload);
    bool readTimePayload(PacketReader& reader, TimePayload& outPayload);
    bool buildFrame(PacketID packetID,
                    uint32_t agvID,
                    uint32_t sequence,
                    const std::vector<uint8_t>& payload,
                    std::vector<uint8_t>& outFrame);
}
