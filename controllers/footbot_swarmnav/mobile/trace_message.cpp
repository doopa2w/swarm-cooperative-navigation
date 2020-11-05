#include "trace_message.h"
#include <string>
#include <sstream>

using namespace argos;

CTraceMessage::CTraceMessage(UInt8 robotId) : RobotId(robotId) {}

std::string CTraceMessage::Format(UInt32 time) {
    std::stringstream result;
    result << time << " -> " << RobotId << ", " << GetMessageType();
    return result.str();
}

std::string CTraceMessage::GetRobotId() {
    std::stringstream result;
    result << RobotId;
    return result.str();
}

CRandomExplorationTrace::CRandomExplorationTrace(UInt8 robotId) : CTraceMessage(robotId) {}

EState CRandomExplorationTrace::GetMessageType() { return RANDOM_EXPLORATION; }

CMoveToGoalTrace::CMoveToGoalTrace(UInt8 robotId) : CTraceMessage(robotId) {}

EState CMoveToGoalTrace::GetMessageType() { return MOVE_TO_GOAL; }

CRestingTrace::CRestingTrace(UInt8 robotId) : CTraceMessage(robotId) {}

EState CRestingTrace::GetMessageType() { return RESTING; }

CCollisionTrace::CCollisionTrace(UInt8 robotId) : CTraceMessage(robotId) {}

EState CCollisionTrace::GetMessageType() { return AVOIDING_COLLISION; }