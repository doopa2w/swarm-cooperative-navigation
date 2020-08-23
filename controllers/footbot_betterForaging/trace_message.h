#ifndef TRACE_MESSAGE_H
#define TRACE_MESSAGE_H

#include "state.h"
#include <string>
#include <argos2/common/utility/datatypes/datatypes.h>

using namespace argos;

// A single log message
//
class CTraceMessage {
 public:
  // What type of message is this?
  virtual EState GetMessageType() = 0;
  // The string that should be written to the trace output
  virtual std::string Format(UInt32 time); 
  // The Id of the robot, as string
  std::string GetRobotId();
  // Constructor and destructor
  CTraceMessage(UInt32 robotId);
  virtual ~CTraceMessage();

 protected:
  // Id of the robot doing the logging
  UInt32 RobotId;
};

class CExploreTrace : public CTraceMessage {
 public:
  EState GetMessageType();
  CExploreTrace(UInt32 robotId);
};

class CPickUpItemTrace : public CTraceMessage {
 public:
  EState GetMessageType();
  CPickUpItemTrace(UInt32 robotId);
};

class CReturnTrace : public CTraceMessage {
 public:
  EState GetMessageType();
  CReturnTrace(UInt32 robotId);
};

class CDropItemTrace : public CTraceMessage {
 public:
  EState GetMessageType();
  CDropItemTrace(UInt32 robotId);
};

class CSearchRestingPlaceTrace : public CTraceMessage {
 public:
  EState GetMessageType();
  CSearchRestingPlaceTrace(UInt32 robotId);
};

class CRestTrace : public CTraceMessage {
 public:
  EState GetMessageType();
  CRestTrace(UInt32 robotId);
};

class CCollisionTrace : public CTraceMessage {
 public:
  EState GetMessageType();
  CCollisionTrace(UInt32 robotId);
};

#endif /* TRACE_MESSAGE_H */
