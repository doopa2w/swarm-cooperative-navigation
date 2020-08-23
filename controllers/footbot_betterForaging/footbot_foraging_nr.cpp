//  AUTHORS: Carlo Pinciroli <cpinciro@ulb.ac.be>
//           Matthias Hölzl <tc@xantira.com>
// 
//  An example foraging controller for the foot-bot.
//  Include the controller definition 
#include "footbot_foraging_nr.h"
//  Function definitions for XML parsing 
#include <argos2/common/utility/configuration/argos_configuration.h>
//  2D vector definition 
#include <argos2/common/utility/math/vector2.h>
//  Logging 
#include <argos2/common/utility/logging/argos_log.h>

CFootBotForagingNR::CFootBotForagingNR() {}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void CFootBotForagingNR::Init(TConfigurationNode& t_node) {
   try {
     // 
     //  Set Id
     // /
     Id = s_unIdCounter++;

      // 
      //  Initialize sensors/actuators
      // /
      Wheels    = dynamic_cast<CCI_FootBotWheelsActuator* >  (GetRobot().GetActuator("footbot_wheels"      ));
      LEDs      = dynamic_cast<CCI_FootBotLedsActuator* >    (GetRobot().GetActuator("footbot_leds"        ));
      RABA      = dynamic_cast<CCI_RangeAndBearingActuator*> (GetRobot().GetActuator("range_and_bearing"   ));
      RABS      = dynamic_cast<CCI_RangeAndBearingSensor*>   (GetRobot().GetSensor  ("range_and_bearing"   ));
      Proximity = dynamic_cast<CCI_FootBotProximitySensor*>  (GetRobot().GetSensor  ("footbot_proximity"   ));
      Light     = dynamic_cast<CCI_FootBotLightSensor*>      (GetRobot().GetSensor  ("footbot_light"       ));
      Ground    = dynamic_cast<CCI_FootBotMotorGroundSensor*>(GetRobot().GetSensor  ("footbot_motor_ground"));
      // 
      //  Parse XML parameters
      // /
      //  Diffusion algorithm 
      DiffusionParams.Init(GetNode(t_node, "diffusion"));
      //  Wheel turning 
      WheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      //  Controller state 
      StateData.Init(GetNode(t_node, "state"));
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing the foot-bot foraging controller for robot \""
				  << GetRobot().GetRobotId() << "\"", ex);
   }
   // Initialize other stuff
   // Create a random number generator. We use the 'argos' category so
   // that creation, reset, seeding and cleanup are managed by ARGoS.
   RNG = CARGoSRandom::CreateRNG("argos");
   Reset();
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

bool CFootBotForagingNR::IsOverNest() {
  // The loop function uses this to determine whether to drop the food
  // item. 
  return true;
}

void CFootBotForagingNR::ControlStep() {
  UpdateState();
  switch(StateData.State) {
  case RESTING: {
    Rest();
    break;
  }
  case EXPLORING: {
    Explore();
    break;
  }
  case PICKING_UP_ITEM: {
    StartReturningToNest();
    break;
  }
  case RETURNING_TO_NEST: {
    ReturnToNest();
    break;
  }
  case DROPPING_ITEM: {
    // No need to search for a resting place; rest where you are.
    StartResting();
    break;
  }
  case SEARCHING_RESTING_PLACE: {
    // We should not arrive here, but just to be sure...
    StartResting();
    break;
  }
  default: {
    LOGERR << "We can't be here, there's a bug!" << std::endl;
  }
  }
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void CFootBotForagingNR::Explore() {
  // This should be implemented differently: check whether we are on a
  // food item, and if so, pick it up.
  if (IsOverFoodItem() && !IsCarryingFood()) {
    PickUpItem();
  }
  else {
    bool collision;
    CVector2 diffusion = DiffusionVector(collision);
    SetWheelSpeedsFromVector(WheelTurningParams.MaxSpeed * diffusion);
  }
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void CFootBotForagingNR::ReturnToNest() {
  // Don't actually return, just drop what you hold.
  DropItem();
}

void CFootBotForagingNR::SearchForRestingPlace() {
  // Rest immediately
  StartResting();
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

void CFootBotForagingNR::Rest() {
  if (StateData.RestingPeriodIsOver()) {
    StartExploring();
  }
  else {
    ++StateData.TimeRested;
  }
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

// This statement notifies ARGoS of the existence of the
// controller. It binds the class passed as first argument to the
// string passed as second argument. The string is then usable in the
// XML configuration file to refer to this controller. When ARGoS
// reads that string in the XML file, it knows which controller class
// to instantiate. See also the XML configuration files for an example
// of how this is used.
//
REGISTER_CONTROLLER(CFootBotForagingNR, "footbot_foraging_controller_nr")
