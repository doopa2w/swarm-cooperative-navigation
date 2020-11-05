#ifndef TRACE_MESSAGE_H
#define TRACE_MESSAGE_H

#include "state.h"
#include <string>
#include <argos3/core/utility/datatypes/datatypes.h>

using namespace argos;

class CTraceMessage {
    public:

        CTraceMessage(UInt8 robotId);
        virtual ~CTraceMessage() {}

        // by default, message type is 0 or RANDOM_EXPLORATION
        virtual EState GetMessageType() = 0;
        virtual std::string Format(UInt32 time);
        std::string GetRobotId();

    protected:

        UInt8 RobotId;
};

class CRandomExplorationTrace : public CTraceMessage {
    public:
        EState GetMessageType();
        CRandomExplorationTrace(UInt8 robotId);
};

class CMoveToGoalTrace : public CTraceMessage {
    public:
        EState GetMessageType();
        CMoveToGoalTrace(UInt8 robotId);
};

class CRestingTrace : public CTraceMessage {
    public:
        EState GetMessageType();
        CRestingTrace(UInt8 robotId);
};

class CCollisionTrace : public CTraceMessage {
    public:
        EState GetMessageType();
        CCollisionTrace(UInt8 robotId);
};


#endif // !TRACE_MESSAGE_H
