//  AUTHORS: Carlo Pinciroli <cpinciro@ulb.ac.be>
//           Matthias Hölzl <tc@xantira.com>
// 
//  An example foraging controller for the foot-bot.
// 
//  This controller is meant to be used with the XML file:
//     xml/foraging.xml

#ifndef FOOTBOT_FORAGING_H
#define FOOTBOT_FORAGING_H

//  Include some necessary headers.

// Definition of the state enumeration.
#include "state.h"
// Definition of the ground sensor result.
#include "ground.h"
// Support for generating runtime traces. 
#include "trace_message.h"
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

class CFootBotForaging : public CCI_Controller {

 public:
  
  // This structure holds data about food collecting by the robots
  struct SFoodData {

    // true when the robot is carrying a food item
    bool HasFoodItem;

    // the total number of food items carried by this robot during the
    // experiment
    size_t TotalFoodItems; 

    SFoodData();
    void Reset();
  };

  // The following variables are used as parameters for the diffusion
  // algorithm. You can set their value in the <parameters> section of
  // the XML configuration file, under the
  // <controllers><footbot_foraging_controller><parameters><diffusion>
  // section.
  struct SDiffusionParams {

    // Maximum tolerance for the proximity reading between the robot
    // and the closest obstacle.  The proximity reading is 0 when
    // nothing is detected and grows exponentially to 1 when the
    // obstacle is touching the robot.
    Real Delta;

    // Angle tolerance range to go straight. 
    CRange<CRadians> GoStraightAngleRange;

    // Constructor 
    SDiffusionParams();

    // Parses the XML section for diffusion 
    void Init(TConfigurationNode& t_tree);
  };

  // The following variables are used as parameters for turning during
  // navigation. You can set their value in the <parameters> section
  // of the XML configuration file, under the
  // <controllers><footbot_foraging_controller><parameters><wheel_turning>
  // section.
  struct SWheelTurningParams {

    // The turning mechanism.
    // The robot can be in three different turning states.
    enum ETurningMechanism
      {
	NO_TURN = 0, // go straight
	SOFT_TURN,   // both wheels are turning forwards, but at different speeds
	HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;

    // Angular thresholds to change turning state.
    CRadians HardTurnOnAngleThreshold;
    CRadians SoftTurnOnAngleThreshold;
    CRadians NoTurnAngleThreshold;

    // Maximum wheel speed 
    Real MaxSpeed;
    
    void Init(TConfigurationNode& t_tree);
  };
  
  // Contains all the state information about the controller.
  struct SStateData {

    // The three possible states in which the controller can be 

    EState State;
    EState PreviousState;

    // Information obtained from the ground sensors
    EGroundSensorInfo PreviousGroundSensorInfo;
    EGroundSensorInfo GroundSensorInfo;

    // Current probability to switch from resting to exploring 
    Real RestToExploreMean;
    // Used as a range for uniform number generation 
    CRange<Real> ProbRange;
     
    // The number of steps in resting state 
    size_t TimeRested;
    // The time when we will wake up again 
    size_t WakeUpTime;

    // If the robots switched to resting as soon as it enters the
    // nest, there would be overcrowding of robots in the border
    // between the nest and the rest of the arena. To overcome this
    // issue, the robot spends some time looking for a place in the
    // nest before finally settling. The following variable contains
    // the minimum time the robot must spend in state 'return to nest'
    // looking for a place in the nest before switching to the resting
    // state.
    size_t MinimumSearchForPlaceInNestTime;
    // The time spent searching for a place in the nest 
    size_t TimeSearchingForPlaceInNest;

    SStateData();
    void Init(TConfigurationNode& t_node);
    void Reset();
    void SetNewWakeUpTime(UInt32 newWakeUpTime);
    void SaveState();
    bool RestingPeriodIsOver();
  };

 public:

  //  Class constructor. 
  CFootBotForaging();
  //  Class destructor. 
  virtual ~CFootBotForaging() {}

  //  This function initializes the controller.
  //  The 't_node' variable points to the <parameters> section in the XML file
  //  in the <controllers><footbot_foraging_controller> section.
  virtual void Init(TConfigurationNode& t_node);

  //  This function is called once every time step.
  //  The length of the time step is set in the XML file.
  virtual void ControlStep();

  //  This function resets the controller to its state right after the Init().
  //  It is called when you press the reset button in the GUI.
  virtual void Reset();

  //  Called to cleanup what done by Init() when the experiment finishes.
  //  In this example controller there is no need for clean anything up, so
  //  the function could have been omitted. It's here just for completeness.
  virtual void Destroy() {}

  // Returns true if the robot is currently on empty ground.
  inline bool IsOverEmptyGround() const {
    return
      StateData.GroundSensorInfo == OVER_EMPTY_GROUND;
  }

  // Returns true if the robot is currently over a food item.
  inline bool IsOverFoodItem() const {
    return StateData.GroundSensorInfo == OVER_FOOD_ITEM;
  }

  // Returns true if the robot is currently in the nest.
  virtual bool IsOverNest(); 

  //  Returns true if the robot is currently exploring.
  inline bool IsExploring() const {
    return StateData.State == EXPLORING;
  }

  //  Returns true if the robot is currently resting.
  inline bool IsResting() const {
    return StateData.State == RESTING;
  }

  // Returns true if the robot is picking up a food item.
  inline bool IsPickingUpFoodItem() const {
    return StateData.State == PICKING_UP_ITEM;
  }

  //  Returns true if the robot is currently returning to the nest.
  inline bool IsReturningToNest() const {
    return StateData.State == RETURNING_TO_NEST;
  }

  //  Returns the food data
  inline SFoodData& GetFoodData() {
    return FoodData;
  }

  // Returns true if the robot is currently carrying a food item
  inline bool IsCarryingFood() {
    return FoodData.HasFoodItem;
  }

  //  Returns the Id
  inline UInt32 GetId() {
    return Id;
  }

  // Returns the State
  inline EState GetState() {
    return StateData.State;
  }

  //  Returns the tracing messages for this robot
  inline std::vector<CTraceMessage*> *GetTraceMessages() {
    return &TraceMessages;
  }

  //  Returns the collision messages for this robot
  inline std::vector<CTraceMessage*> *GetCollisionMessages() {
    return &CollisionMessages;
  }

 protected:

  // Evaluates the information obtained from the ground sensors
  void EvaluateGroundSensorInfo();

  // Updates the state information.
  // In pratice, it sets the InNest flag.  Future, more
  // complex implementations should add their state update code here.
  void UpdateState();

  // Calculates the vector to the light. Used to perform phototaxis
  // and antiphototaxis.
  CVector2 CalculateVectorToLight();

  // Calculates the diffusion vector. If there is a close obstacle, it
  // points away from it; it there is none, it points forwards.  The
  // b_collision parameter is used to return true or false whether a
  // collision avoidance just happened or not. It is necessary for the
  // collision rule.
  CVector2 DiffusionVector(bool& b_collision);

  // Gets a direction vector as input and transforms it into wheel actuation.
  void SetWheelSpeedsFromVector(const CVector2& c_heading);

  // Enters the exploring state.
  void StartExploring();

  // Collect an item.
  void PickUpItem();

  // Enters the "return to nest" state.
  void StartReturningToNest();

  // Drop an item.
  void DropItem();

  // Enters the "search for a resting place" state.
  void StartSearchingForRestingPlace();

  // Enters the resting state.
  void StartResting();

  // Executes the exploring state.
  virtual void Explore();

  // Executes the return to nest state.
  virtual void ReturnToNest();

  // Executes the search for resting place state.
  virtual void SearchForRestingPlace();

  // Executes the resting state.
  virtual void Rest();

 protected:

  // Pointer to the foot-bot wheels actuator 
  CCI_FootBotWheelsActuator*  Wheels;
  // Pointer to the foot-bot LEDs actuator 
  CCI_FootBotLedsActuator*  LEDs;
  // Pointer to the range and bearing actuator 
  CCI_RangeAndBearingActuator*  RABA;
  // Pointer to the range and bearing sensor 
  CCI_RangeAndBearingSensor* RABS;
  // Pointer to the foot-bot proximity sensor 
  CCI_FootBotProximitySensor* Proximity;
  // Pointer to the foot-bot light sensor 
  CCI_FootBotLightSensor* Light;
  // Pointer to the foot-bot motor ground sensor 
  CCI_FootBotMotorGroundSensor* Ground;

  static UInt32 s_unIdCounter;
  UInt32 Id;

  // The random number generator 
  CARGoSRandom::CRNG* RNG;

  // The controller state information 
  SStateData StateData;
  // The turning parameters 
  SWheelTurningParams WheelTurningParams;
  //  The diffusion parameters 
  SDiffusionParams DiffusionParams;
  //  The food data 
  SFoodData FoodData;

  std::vector<CTraceMessage*> TraceMessages;
  std::vector<CTraceMessage*> CollisionMessages;

};

#endif
