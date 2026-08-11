#pragma once

#include "RobotProtocol.hpp"

#include <cstdint>

class TrajectoryCommandStore
{
public:
    enum class StoreResult : uint8_t
    {
        STORED,
        DUPLICATE_IGNORED,
        REJECTED_INVALID,
        REJECTED_BUSY
    };

    StoreResult store(const RobotProtocol::TrajectoryCommandPayload& command);
    void clear();

    bool hasCommand() const { return m_HasCommand; }
    const RobotProtocol::TrajectoryCommandPayload& command() const
    {
        return m_Command;
    }

    static const char* resultName(StoreResult result);

private:
    static bool isSemanticallyValid(
        const RobotProtocol::TrajectoryCommandPayload& command);
    static bool isSameCommand(
        const RobotProtocol::TrajectoryCommandPayload& lhs,
        const RobotProtocol::TrajectoryCommandPayload& rhs);

    RobotProtocol::TrajectoryCommandPayload m_Command{};
    bool m_HasCommand = false;
};
