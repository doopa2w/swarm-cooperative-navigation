/*
 * AUTHOR: Darryl Tan <dtzl.added@gmail.com>
 * Log messages
 * 
 */

#ifndef TRACE_MESSAGE_H
#define TRACE_MESSAGE_H

// Include some necessary headers
#include "state.h"
#include <string>
#include <argos3/core/utility/datatypes/datatypes.h>

using namespace argos;

class CTraceMessage {
    public:

        // Constructor and Destructor
        CTraceMessage(UInt32 robotId);
        virtual ~CTraceMessage() {}

        /*
         * Determine type of message
         * By default, its 0 or RANDOM_EXPLORATION.
         *
         */
        virtual EState GetMessageType() = 0;
        // The string should be written to the trace output
        virtual std::string Format(UInt32 time);
        // Get Robot ID as string type
        std::string GetRobotId();

    protected:
        
        // The Id for the robot that is doing the message logging
        UInt32 RobotId;
};

class CRandomExplorationTrace : public CTraceMessage {
    public:
        EState GetMessageType();
        CRandomExplorationTrace(UInt32 robotId);
};

class CAggressiveExplorationTrace : public CTraceMessage {
    public:
        EState GetMessageType();
        CAggressiveExplorationTrace(UInt32 robotId);
};

class CMoveToGoalTrace : public CTraceMessage {
    public:
        EState GetMessageType();
        CMoveToGoalTrace(UInt32 robotId);
};

class CRestingTrace : public CTraceMessage {
    public:
        EState GetMessageType();
        CRestingTrace(UInt32 robotId);
};

class CCollisionTrace : public CTraceMessage {
    public:
        EState GetMessageType();
        CCollisionTrace(UInt32 robotId);
};

#endif // !TRACE_MESSAGE_H
