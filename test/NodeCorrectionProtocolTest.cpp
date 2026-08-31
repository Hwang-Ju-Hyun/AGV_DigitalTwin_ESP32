#include "RobotProtocol.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace
{
    std::vector<uint8_t> buildCommand(
        RobotProtocol::NodeCorrectionAction action,
        float magnitude)
    {
        std::vector<uint8_t> bytes;
        RobotProtocol::PacketWriter writer(bytes);
        writer.writeUInt32(10);
        writer.writeUInt32(8);
        writer.writeUInt32(3);
        writer.writeUInt8(static_cast<uint8_t>(action));
        writer.writeFloat(magnitude);
        return bytes;
    }
}

int main()
{
    using namespace RobotProtocol;

    assert((CAPABILITY_TRAJECTORY_COMMAND | CAPABILITY_NODE_CORRECTION)
           == 0x5U);
    assert(static_cast<uint16_t>(PacketID::NODE_CORRECTION_COMMAND) == 103);
    assert(static_cast<uint16_t>(PacketID::NODE_CORRECTION_REPORT) == 202);
    assert(static_cast<uint8_t>(NodeCorrectionAction::DRIVE_FORWARD) == 2);
    assert(static_cast<uint8_t>(NodeCorrectionAction::TURN_CW) == 3);
    assert(static_cast<uint8_t>(NodeCorrectionAction::TURN_CCW) == 4);
    assert(static_cast<uint8_t>(NodeCorrectionResult::COMPLETED) == 2);
    assert(static_cast<uint8_t>(NodeCorrectionResult::REJECTED) == 3);
    assert(static_cast<uint8_t>(NodeCorrectionResult::FAULT) == 4);

    assert(isKnownPacketID(
        static_cast<uint16_t>(PacketID::NODE_CORRECTION_COMMAND)));
    assert(isKnownPacketID(
        static_cast<uint16_t>(PacketID::NODE_CORRECTION_REPORT)));

    std::vector<uint8_t> commandBytes = buildCommand(
        NodeCorrectionAction::TURN_CCW, 0.25f);
    assert(commandBytes.size() == kNodeCorrectionCommandPayloadSize);

    PacketReader commandReader(commandBytes.data(), commandBytes.size());
    NodeCorrectionCommandPayload command;
    assert(readNodeCorrectionCommandPayload(commandReader, command));
    assert(command.routeID == 10);
    assert(command.nodeID == 8);
    assert(command.commandID == 3);
    assert(command.action == NodeCorrectionAction::TURN_CCW);
    assert(command.magnitude == 0.25f);

    commandBytes.push_back(0);
    PacketReader trailingReader(commandBytes.data(), commandBytes.size());
    assert(!readNodeCorrectionCommandPayload(trailingReader, command));

    std::vector<uint8_t> invalidAction = buildCommand(
        static_cast<NodeCorrectionAction>(1), 20.0f);
    PacketReader invalidActionReader(invalidAction.data(),
                                     invalidAction.size());
    assert(!readNodeCorrectionCommandPayload(invalidActionReader, command));

    std::vector<uint8_t> invalidMagnitude = buildCommand(
        NodeCorrectionAction::DRIVE_FORWARD,
        std::numeric_limits<float>::quiet_NaN());
    PacketReader invalidMagnitudeReader(invalidMagnitude.data(),
                                        invalidMagnitude.size());
    assert(!readNodeCorrectionCommandPayload(invalidMagnitudeReader,
                                             command));

    NodeCorrectionReportPayload report;
    report.routeID = 10;
    report.nodeID = 8;
    report.commandID = 3;
    report.result = NodeCorrectionResult::COMPLETED;
    report.detail = 0x12345678U;

    std::vector<uint8_t> reportBytes;
    PacketWriter reportWriter(reportBytes);
    writeNodeCorrectionReportPayload(reportWriter, report);
    assert(reportBytes.size() == kNodeCorrectionReportPayloadSize);

    PacketReader reportReader(reportBytes.data(), reportBytes.size());
    uint32_t routeID = 0;
    uint32_t nodeID = 0;
    uint32_t commandID = 0;
    uint8_t result = 0;
    uint32_t detail = 0;
    assert(reportReader.readUInt32(routeID));
    assert(reportReader.readUInt32(nodeID));
    assert(reportReader.readUInt32(commandID));
    assert(reportReader.readUInt8(result));
    assert(reportReader.readUInt32(detail));
    assert(reportReader.remaining() == 0);
    assert(routeID == report.routeID);
    assert(nodeID == report.nodeID);
    assert(commandID == report.commandID);
    assert(result == static_cast<uint8_t>(report.result));
    assert(detail == report.detail);
    return 0;
}
