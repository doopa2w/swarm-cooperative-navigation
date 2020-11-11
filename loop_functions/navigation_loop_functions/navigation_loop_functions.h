#ifndef NAVIGATION_LOOP_FUNCTIONS_H
#define NAVIGATION_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <controllers/footbot_swarmnav/mobile/state.h>
#include <controllers/footbot_swarmnav/mobile/footbot_mobile.h>

using namespace argos;

class CNavigationLoopFunctions : public CLoopFunctions {

   public:

      CNavigationLoopFunctions();
      virtual ~CNavigationLoopFunctions() {}

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();
      virtual void PreStep();

   private:


      // number of mobile robots
      UInt8 numberOfMobileRobots;

      // trace output files and stream
      std::string strTraceOutput;
      std::ofstream TraceOutput;
      void WriteTraceMessages(CFootBotEntity *footBot, CFootBotMobile *controller);


      UInt8 IdForCollisionOutput;
      std::string strCollisionOutput;
      std::ofstream CollisionOutput;
      void WriteCollisionMessages(CFootBotEntity *footBot, CFootBotMobile *controller);

      // The output file and stream for aggregated results
      std::string strSummaryOutput;
      std::ofstream SummaryOutput;
      void InitializeSummaryData();
      void AddSummaryData(CFootBotEntity *footBot, CFootBotMobile *controller, size_t i);
      void WriteSummaryOutput();

      void ClearAllMessages(CFootBotMobile *controller);
      void FlushOutputStreams();

      // summary of all events
      std::vector<UInt32> AggregatedEvents;
};


#endif // !NAVIGATION_LOOP_FUNCTIONS_H