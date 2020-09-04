#include "trace_message.h"
#include <string>
#include<sstream>

using namespace argos;

CTraceMessage::CTraceMessage(UInt32 robotId) :
  RobotId(robotId) {}

CTraceMessage::~CTraceMessage() {
}

std::string CTraceMessage::Format(UInt32 time) {
  std::stringstream result;
  result << time << ","
	 << RobotId << ","
	 << GetMessageType();
  return result.str();
}

std::string CTraceMessage::GetRobotId() {
  std::stringstream result;
  result << RobotId;
  return result.str();
}


CExploreTrace::CExploreTrace(UInt32 robotId) :
  CTraceMessage(robotId) {}

EState CExploreTrace::GetMessageType() {
  return EXPLORING;
}


CPickUpItemTrace::CPickUpItemTrace(UInt32 robotId) :
  CTraceMessage(robotId) {}

EState CPickUpItemTrace::GetMessageType() {
  return PICKING_UP_ITEM;
}


CReturnTrace::CReturnTrace(UInt32 robotId) :
  CTraceMessage(robotId) {}

EState CReturnTrace::GetMessageType() {
  return RETURNING_TO_NEST;
}


CDropItemTrace::CDropItemTrace(UInt32 robotId) :
  CTraceMessage(robotId) {}

EState CDropItemTrace::GetMessageType() {
  return DROPPING_ITEM;
}


CSearchRestingPlaceTrace::CSearchRestingPlaceTrace(UInt32 robotId) :
  CTraceMessage(robotId) {}

EState CSearchRestingPlaceTrace::GetMessageType() {
  return SEARCHING_RESTING_PLACE;
}


CRestTrace::CRestTrace(UInt32 robotId) :
  CTraceMessage(robotId) {}

EState CRestTrace::GetMessageType() {
  return RESTING;
}


CCollisionTrace::CCollisionTrace(UInt32 robotId) :
  CTraceMessage(robotId) {}

EState CCollisionTrace::GetMessageType() {
  return AVOIDING_COLLISION;
}

