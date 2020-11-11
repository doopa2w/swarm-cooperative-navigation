#include "navigation_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/datatypes/any.h>
#include <controllers/footbot_swarmnav/mobile/trace_message.h>

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

CNavigationLoopFunctions::CNavigationLoopFunctions() :
   AggregatedEvents(100, 0) {}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void CNavigationLoopFunctions::Init(TConfigurationNode& node) {
   TConfigurationNode& navigation = GetNode(node, "navigation");

   // get the trace output from the argos file
   GetNodeAttribute(navigation, "trace_output", strTraceOutput);
   // Open the file, erasing exisiting content
   TraceOutput.open(strTraceOutput.c_str(),
      std::ios_base::trunc | std::ios_base::out);

   // Get the collision output
   GetNodeAttribute(navigation, "collision_output", strCollisionOutput);
   // Open the file, erasing exisiting content
   TraceOutput.open(strCollisionOutput.c_str(),
      std::ios_base::trunc | std::ios_base::out);
   // Get the id for the robot for which we are logging collisions
   GetNodeAttribute(navigation, "id_for_collision_output", IdForCollisionOutput);

   GetNodeAttribute(navigation, "number_of_mobile_robots", numberOfMobileRobots);

   //  Get the summary output file name from XML 
   GetNodeAttribute(navigation, "summary_output", strSummaryOutput);
   //  Open the file, erasing its contents 
   SummaryOutput.open(strSummaryOutput.c_str(),
            std::ios_base::trunc | std::ios_base::out);
}
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void CNavigationLoopFunctions::Reset() {
   // zero counters
   InitializeSummaryData();

   // close the output streams
   TraceOutput.close();
   CollisionOutput.close();
   SummaryOutput.close();
   //  Open the output streams, erasing their contents 
   TraceOutput.open(strTraceOutput.c_str(),
            std::ios_base::trunc | std::ios_base::out);
   CollisionOutput.open(strCollisionOutput.c_str(),
               std::ios_base::trunc | std::ios_base::out);
   SummaryOutput.open(strSummaryOutput.c_str(),
            std::ios_base::trunc | std::ios_base::out);
}

void CNavigationLoopFunctions::Destroy() {
   // close the files
   TraceOutput.close();
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void CNavigationLoopFunctions::WriteTraceMessages(CFootBotEntity *footBot, CFootBotMobile *controller) {
   std::vector<CTraceMessage*> *traceMessages = controller->GetTraceMessages();
   for (std::vector<CTraceMessage*>::iterator sit = traceMessages->begin();
      sit != traceMessages->end();
      ++sit) {
   // Trace messages were gathered in the previous loop...
   TraceOutput << (*sit)->Format(GetSpace().GetSimulationClock() - 1)  << std::endl;
   }
}

void CNavigationLoopFunctions::WriteCollisionMessages(CFootBotEntity *footBot, CFootBotMobile *controller) {
   std::vector<CTraceMessage*> *collisionMessages = controller->GetCollisionMessages();
   if (controller->GetId() == IdForCollisionOutput) {
      for (std::vector<CTraceMessage*>::iterator sit = collisionMessages->begin();
      sit != collisionMessages->end();
      ++sit) {
      // Collision messages were gathered in the previous loop...
      CollisionOutput << (*sit)->Format(GetSpace().GetSimulationClock() - 1)  << std::endl;
      }
   }
}

void CNavigationLoopFunctions::ClearAllMessages(CFootBotMobile *controller) {
  controller->GetTraceMessages()->clear();
  controller->GetCollisionMessages()->clear();
}

void CNavigationLoopFunctions::InitializeSummaryData() {
  for (UInt8 i = 0; i < numberOfMobileRobots; ++i) {
     AggregatedEvents[i] = 0;
  }
}

void CNavigationLoopFunctions::AddSummaryData(CFootBotEntity *footBot,CFootBotMobile *controller, size_t i) {
  AggregatedEvents[i] = controller->GetStateData().TimeTakenToReachGoal;
}

void CNavigationLoopFunctions::WriteSummaryOutput() {
  SummaryOutput << GetSpace().GetSimulationClock() - 1;
  for (UInt32 i = 0; i < AggregatedEvents.size(); ++i) {
    SummaryOutput << "," << AggregatedEvents[i];
  }
  SummaryOutput << std::endl;
}

void CNavigationLoopFunctions::FlushOutputStreams() {
  TraceOutput << std::flush;
  CollisionOutput << std::flush;
  SummaryOutput << std::flush;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void CNavigationLoopFunctions::PreStep() {

   UInt32 currentClock = GetSpace().GetSimulationClock();
   // Get a collection of all footbots
   CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");

   size_t cnt = 0;

   InitializeSummaryData();
   for (CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it) {
      // Get handle to footbot entity and controller
      CFootBotEntity& footBot = *any_cast<CFootBotEntity*>(it->second);
      CFootBotMobile& controller = dynamic_cast<CFootBotMobile&>(footBot.GetControllableEntity().GetController());
      // Get nav data
      CFootBotMobile::SStateData& stateData = controller.GetStateData();



      WriteTraceMessages(&footBot, &controller);
      WriteCollisionMessages(&footBot, &controller);
      AddSummaryData(&footBot, &controller, cnt);
      ClearAllMessages(&controller);

      ++cnt;
   }

   WriteSummaryOutput();
   FlushOutputStreams();
}




////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

REGISTER_LOOP_FUNCTIONS(CNavigationLoopFunctions, "navigation_loop_functions")