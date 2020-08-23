/*
 * AUTHOR: Darryl Tan <dtzl.added@gmail.com>
 * 
 * TODO: Description for this header
 * 
 */

// Include some necessary headers
#include "trace_message.h"
#include <string>
#include <sstream>

using namespace argos;

CTraceMessage::CTraceMessage(UInt32 robotId) :
    RobotId(robotId) {}

std::string CTraceMessage::Format(UInt32 time) {
    std::stringstream result;
    result  << "["<< time << "] " << RobotId << ", " << GetMessageType(); 
    return result.str();
}

std::string CTraceMessage::GetRobotId() {
    std::stringstream result;
    result << RobotId;
    return result.str();
}

CRandomExplorationTrace::CRandomExplorationTrace(UInt32 robotId) :
    CTraceMessage(robotId) {}

EState CRandomExplorationTrace::GetMessageType() { return RANDOM_EXPLORATION; }

CAggressiveExplorationTrace::CAggressiveExplorationTrace(UInt32 robotId) :
    CTraceMessage(robotId) {}

EState CAggressiveExplorationTrace::GetMessageType() { return AGGRESSIVE_EXPLORATION; }

CMoveToGoalTrace::CMoveToGoalTrace(UInt32 robotId) :
    CTraceMessage(robotId) {}

EState CMoveToGoalTrace::GetMessageType() { return MOVE_TO_GOAL; }

CRestingTrace::CRestingTrace(UInt32 robotId) :
    CTraceMessage(robotId) {}

EState CRestingTrace::GetMessageType() { return RESTING; }

CCollisionTrace::CCollisionTrace(UInt32 robotId) :
    CTraceMessage(robotId) {}

EState CCollisionTrace::GetMessageType() { return AVOIDING_COLLISION; }