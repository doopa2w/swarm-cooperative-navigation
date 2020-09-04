//  AUTHORS: Carlo Pinciroli <cpinciro@ulb.ac.be>
//           Matthias Hölzl <tc@xantira.com>
// 
//  An example foraging controller for the foot-bot.
// 
//  This controller is meant to be used with the XML file:
//     xml/foraging.xml

#ifndef FOOTBOT_FORAGING_NR_H
#define FOOTBOT_FORAGING_NR_H

//  Include some necessary headers.

// Include the superclass header.
#include "footbot_foraging.h"
// Definition of the CCI_Controller class. 
#include <argos2/common/control_interface/ci_controller.h>
// Definition of the foot-bot wheel actuator 
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_wheels_actuator.h>
// Definition of the foot-bot LEDs actuator 
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_leds_actuator.h>
// Definition of the range and bearing actuator 
#include <argos2/common/control_interface/swarmanoid/ci_range_and_bearing_actuator.h>
// Definition of the range and bearing sensor 
#include <argos2/common/control_interface/swarmanoid/ci_range_and_bearing_sensor.h>
// Definition of the foot-bot proximity sensor 
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_proximity_sensor.h>
// Definition of the foot-bot light sensor 
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_light_sensor.h>
// Definition of the foot-bot motor ground sensor 
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_motor_ground_sensor.h>
// Definitions for random number generation 
#include <argos2/common/utility/argos_random.h>

using namespace argos;

class CFootBotForagingNR : public CFootBotForaging {

 public:

  //  Class constructor. 
  CFootBotForagingNR();
  //  Class destructor. 
  virtual ~CFootBotForagingNR() {}

  //  This function initializes the controller.
  //  The 't_node' variable points to the <parameters> section in the XML file
  //  in the <controllers><footbot_foraging_controller> section.
  virtual void Init(TConfigurationNode& t_node);

  // Returns true if the robot is currently in the nest.
  virtual bool IsOverNest(); 

  //  This function is called once every time step.
  //  The length of the time step is set in the XML file.
  virtual void ControlStep();

 private:

  // Executes the exploring state.
  void Explore();

  // Executes the return to nest state.
  void ReturnToNest();

  // Executes the search for resting place state.
  void SearchForRestingPlace();

  // Executes the resting state.
  void Rest();
};

#endif
