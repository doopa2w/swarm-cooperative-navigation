# ARGos-Foraging



## Folder Structure

1. controllers
   - footbot_foraging.cpp
   - footbot_foraging.h
   - ground.h
   - state.h
   - trace_message.cpp
   - trace_message.h
2. loop_functions
   * foraging_loop_functions.cpp
   * foraging_loop_functions.h
   * foraging_qt_user_functions.cpp
   * foraging_qt_user_functions.h

### state.h

This header contains enumerate which allows certain integers to represent one of the state for the mobile robot. Here are the following states along with its respective integer representation:

* EXPLORING = 0
* PICKING_UP_ITEM =1
* RETURNING_TO_NEST = 2
* DROPPING_ITEM = 3
* SEARCHING_RESTING_PLACE = 4
* RESTING = 5
* AVOIDING_COLLISION = 6
* STATE_SIZE // the size of an array indexed by non-debugging message types
* CONTROL_LOOP_START = -1
* MESSAGE_QUEUE_LENGTH = -2

### ground.h

This header contains enumerate which allows certain integers to represent one of the information picked up from the ground sensors. Here are the following information representation:

* OVER_EMPTY_GROUND = 1 << 0 // 1, but using bit shift instead for ease of bit mask operation
* OVER_FOOD_ITEM = 1 << 1 //  2
* OVER_NEST = 1 << 2 // 4

### footbot_foraging.h

* **class CFootBotForaging : public CCI_Controller**
  * SFoodData holds data about food collected by the mobile robots
    * bool HasFoodItem // true when the robot carries a food item
    * size_t TotalFoodItems // the total number of food carried by the robot
  * SWheelTurningParams refers to the turning mechanism whereby the robot can be in three different turning state:
    * enum ETurningMechanism
      * NO_TURN = 0 // go straight
      * SOFT_TURN // both wheels are turning forward but at different speed
      * HARD_TURN // wheels are turning with opposite speed
    * Angular threshold to change turning state:
      * (CRadians) HardTurnOnAngleThreshold, SoftTurnOnAngleThreshold, NoTurnAngleThreshold
    * ((Real) MaxSpeed // define maximum wheel speed
  * SStateDate contains all the state information about the controller where the robot has all three possible states as follows:
    * (EState) State, PreviousState // possible states
    * (EGroundSensorInfo) PreviousGroundSensorInfo, GroundSensorInfo / information obtained from the ground sensors
    * (Real) RestToExplorerMean // current probability to switch from resting state -> exploring state
    * (size_t) TimeRested // the number of steps in resting state
    * (size_t) WakeUpTime // the time on how long the robot will take before waking up
    * (size_t) MinimumSearchForPlaceInNestTIme // if the robots switched to resting as soon as it enters the nest, there would be overcrowding of robots in the border between the nest and the rest of the arena.
      * To overcome this issue, the robot must spend some time looking for a place in the nest before finally settling. The variable contains the minimum time the robot must spend in the state 'return to nest' before switching to resting state.
    * (size_t) TimeSearchingForPlaceInNest
  * inline bool IsOverEmptyGround() returns true if the robot is currently on empty ground
  * inline bool IsOverFoodItem() returns true if the robot is currently over a food item.
  * virtual bool IsOverNest() returns true if the robot is currenlty in the nest.
  * inline bool IsExploring() returns ture if the robot is currently exploring.
  * inline bool isResting() returns true if the robot is currently resting.
  * inline bool IsPickingUpFoodItem() returns true if the robot is picking up a food item.
  * inline bool IsReturningToNest() returns true if the robot is currenlty returning to the nest.
  * inline SFoodData& GetFoodData() returns food data.
  * inline bool IsCarryingFood() returns true if the robot is currently carrying a food item
  * inline UInt32 GetId() returns the robot's ID
  * inline Estate GetState() returns the robots' state.
  * GetTraceMessage() returns the tracing message for this robot
  * GetCollisionMessage() returns the collision message for this robot
  * void EvaluateGroundSensorInfo() evaluates the information obtained from the ground sensors.
  * void UpdateState() upates the state information.
  * CVector2 CalculateVectorToLight() calcaulates the diffusion vector.
    * If there is a close obstalce, it points away from it
    * If there is no obstalce, it points forward
    * b_collsion paramter is used to return ture of false number whether the collision avoidance just happed or not.
    * It is vital for the collision avoidance rule.
  * void SetWheelSpeedFromVector() gets a direction vector.
  * All possible robot's state:
    * void StartExploring()
    * void PickupItem() 
    * void StartREeturningToNest() // entres the search for a searcnhing 
    * void StartResting() entres the resting state.
    * virtual void Explore()
    * virtual void ReturnToNest() executes the return to nest state.
    * void StartResting() // enters the resting state.
    * virtual void Rest() executes the resting state.

### footbot_foraging.cpp

* Includes:
  * "footbot_foraging.h" // controller definition
  * argos_configuration.h // function definitions for XML parsing
  * vector2.h // 2D vector definition
  * argos_log.h // logging for ARGos
* CFootBotForaging::SFoodDate::SFoodData()
  * Initialize with HasFoodItem = false; TotalFoodItems = 0;
* CFootBotForaging::SFoodData::Reset()
  * Resets with HasFoodItem = false; TotalFoodItems = 0;
* CFootBotForaging::SDiffusionParams::SDiffusionParams()
  * GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f))
* CFootBotForaging::SDiffusionParams::Init()
  * Initializing with a try catch block
    * try: cGoStraightAngleRangeDegrees(CDegress(-10.0f), CDegrees(10.0f))
    * catch (CARGoSException ex) { "Error Initializing controller diffusion parameters"}
* CFootBotForaging::SWheelTurningParams::Init() 
  * Initializing with a try catch block
    * try: HardTurnOnAngleThreshold; SoftTurnOnAngleThreshold; NoTurnAngleThreshold
    * catch (CARGoSException ex) {"Error Initializing controller wheel turning parameters"}
* CFootBotForaging::SStaeData::SStateData()
  * ProbRange(0.0f, 1.0f) 
* CFootBotForaging::SstateData::Init()
  * Initialize with try-catch block
    * try: GetNodeAttribute for "rest_to_explore_mean", "minimum_search_for_place_in_nest_time"
    * catch {"Error Initializing controller state parameters"}
* CFootBotForaging::SStateData::Reset()
  * State = RESTING; PreviousGroundSensorInfo = OVER_EMPTY_GROUND; GroundSensorInfo = OVER_EMPTY_GROUND; WakeUpTime = 0; TimeRested = 0; TimeSearchingForPlaceInNest = 0
* CFootBotForaging::SStateData::SetNewWakeUpTime(UInt32 newTime)
  * WakeUpTime = newTime; TimeRested  = 0
* CFootBotForaging::SStateData::SaveState() 
  * PreviousState = State
  * CFootBotForaging::SStateData::RestingPeriodIsOver()
    * returns true if TimeRested >= WakeUpTime, else false
* CFootBotForaging::CFootBotForaging() :
  * Wheels, LEDs, RABA, RABS, Proximity, Light, Ground, RNG = NULL
* (UInt32) CFootBotForaging::s_unIdCounter = 0
* CFootBotForaging::Init()
  * try-catch block for initliazing sensors and actuators, diffusion algorithm, wheel turning, controller state
  * RNG = CARGoSRandom::CreateRNG('argos"') for a random number generator
  * Reset()
* (bool) CFootBotForaging::IsOverNest()
  * returns StateData.GroundSensorInfo == OVER_NEST
* CFootBotForaging::ControlStep()
  * UpdateState() // get a new state
  * switch case blocks for all the possible robot states; executes respective function based on the state
  * default: LOGGER << "We can't be here, there's a bug!"
* CFootBotForaging::Reset()
  * StateData.Reset() // reset robot state
  * FoodData.Reset() // reset food data
  * LEDs -> SetAllColors(CColor::RED) // set LED color back to RED
* CFootBotForaging::EvaluateGroundSensorInfo()
  * read stuff from the ground sensor first and set it as PreviousGroundSensorInfo then set the new one as OVER_EMPTY_GROUND
  * You can say whether you are in the nest by checking the ground sensor.
  * It will return a value between 0 and 1.
  * It is 1 when the robot is on white area; 0 when the robot is on a black area; around 0.5 when the robot is on a gray area
  * The foot-bot has 4 ground sensors; the algorihtms checks whether all the sensors shows black first; if so we assume that we are over a food item that we can pick up
  * Otherwise we want 2 of the sensors to tell us whether we are on a black, gray or white surface; If so, the robot is completely in the nest, otherwise it's over empty ground.
  * if readings >= 3 then set GroundSensorInfo = OVER_NEST
* CFootBotForaging::UpdateState()
  * EvaluateGroundSensorInfo() // called the first thing inside the ControlStep
* CFootBotForaging::CalculateVectorToLight()
  * Get readings from the light sensor first then calculate it
    * CVector2 cAccumulator // sum all readings from all the light sensor
  * If the light was pereived return the vector, otherwise returns 0
* CFootBotForaging::DiffusionVector(bool& collision)
  * Get readings from the proximity sensor then calculate it
    * CVector2 diffusionVecotr // sum all the readings from the proximity sensors
  * If the angle of the vector calculated is smalle enough and the closest obstacle is far enough, ignore the vector and proceeds to go straight since collision = false; otherwise collision = true
  * Normalie diffusionVector and returns it -diffusionVetor
* CFootBotForaging::SetWheelSpeedsFromVector(const CVector2& heading)
  * Get the heading angle and length of the heading vector
  * Clamp the speed so that it's not greater than MaxSpeed
  * if-else blocks for Turning State SwitchConditions
    * if abs(heading angle) <= NoTurnAngleThreshold then NO_TURN
    * else if abs(heading angle) > HardTurnOnAngleThreshold then HARD_TURN
    * else if NO_TURN && abs(heading angle) > SoftTurnOnAngleThreshold then SOFT_TURN since its between two cases
  * Another if blocks for collision
    * if the new TurningMechanism != NO_TURN && old TurningMechanism != new TurningMechanism then CollisionMessages.push_back(new CollisionTrace(Id))
      * since the current state of robot is forced to do either SOFT_TURN or HARD_TURN and that the previous state was different from current state; -> it is assumed that the robot is facing possible collision; hence LOG it only at the start 
      * as for the proceeding steps, ignore due to previous state == current state as long the robot is forced to do either SOFT_TURN or HARD_TURN
  * Wheel speeds tuned based on current turning state using switch-case blocks
    * NO_TURN
      * fSpeed1, fSpeed2 = fBaseAngularWheelSpeed (same speed for both wheels)
    * SOFT_TURN
      * both wheels go straight but one is faster than the other based on which direction the robot should proceed next to avoid collision
    * HARD_TURN
      * opposite wheel speed 
      * fSped1 = -MaxSpeed; fSpeed2 = MaxSpeed
  * Apply the calculate wheel speeds to the appropriate wheels
    * if heading angle > CRadisns::ZERO  then turn left
      * Left = fSpeed1; Right fSpeed2
    * else turn right
      * Left = fSpeed2; Right = fSpeed1
    * Wheels -> SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed) // finally set the wheel speeds
* CFootBotForaging::StartExploring()
  * if StateData.State == EXPLORING then LOG it
  * StateDate.SaveState() // save previous state
  * StateData.State = EXPLORING // set EXPLORING state again
  * LEDs -> GREEN
  * TraceMessage.push_back(new CExploreTrace(Id))
* CFootBotForaging::PickUpItem()
  * if StateData.State == PICKING_UP_ITEM then LOG IT
  * if FoodData.HasFoodItem == true then LOG IT
  * StateData.SaveState()
  * StateData.State = PICKING_UP_ITEM
  * LEDs -> Yellow
  * TraceMessage.push_back(new CPickUpItemTrace(Id))
* CFootBotForaging::StartReturningToNest()
  * if StateData.State == RETURNING_TO_NEST then LOG IT
  * StateData.SaveState()
  * StateData.State = RETURNING_TO_NEST
  * LEDs -> BLUE
  * TraceMessage.push_back(new CReturnTracce(Id))
* CFootBotForaging::DropItem()
  * if StateData.State == DROPPING_ITEM then LOG IT
  * StateData.SaveState()
  * StateData.State = DROPPING_ITEM
  * LEDs -> YELLOW
  * TraceMessages.push_back(new CDropItemTrace(Id))
* CFootBotForaging::StartSearchingForRestingPlace()
  * if StateDate.State = SEARCHING_RESTING_PLACE then LOG IT
  * StateData.SaveState()
  * StateDate.State = SEARCHING_RESTING_PLACE
  * LEDs -> MAGNETA
  * TraceMessage.push_back(new CSearchingRestingPlaceTrace(Id))
* CFootBotForaging::StartResting()
  * if StateData.State == RESTING then LOG IT
  * StateData.SaveState()
  * StateData.State = RESTING
  * LEDs - > RED
  * TraceMEssages.push_back(new CRestTrace(Id))
  * Wheels -> SetLInearVelocity(0.0f, 0.0f) // static
  * Real restingTime = RNG -> Exponential(StateData.RestToExploreMean) // RNG
  * StateData.SetNewWakeUpTime(restingTime) // set the wakup time as resting time (sort of like countdown)
* CFootBotForaging::Explore()
  * This should be implemented differently; Check whether we are on a food item, if so, pick it up.
  * if robot is over food item and not carrying any food then PickUpItem() and return. // exit out of the function
  * else:
    * bool collision // Get the diffusion vector to perform obstacle avoidance
    * CVector2 diffusion = DiffusionVector(collision) // perform if collision == true
    * If we are in the nest, we combine antiphototaxis with obstalce avoidance. Outside the nest, we just use the diffusion vector instead.
    * if the robot is over the nest:
      * if collision is true, then change wheel speed.
        * The vector calculated by CalculateVectorToLight() points to the light. Thus the minus sign used in the SetWheelsSpeedFromVector() will led the robot steer away from the light.
      * else do the same thing but with different parameter for SetWheelSpeedFromVector(). The minus sign for vector calcualted in CalculateVectorToLight() still stay true in here.
    * else: // if not over the nest
      * SetWheelSpeedsFromVector(WheelTurningParams.MaxSpeed * diffusion) // use the diffusion vector only.
* CFootBotForaging::ReturnToNest()
  * Check if the robot is over the nest; if so, drop item if there is any.
  * else // not over the nest
    * bool collision; CVector2 diffusion = DiffusionVector(collision); // calculate diffusion vector
    * if collision == true then call SetWheelSpeedsFromVector() with different argument. This time no minus sign for CalculateVectorToLight() since we want to steer towards the light which represents nest.
    * else call CalculateVectorToLight() with different argument as well.
* CFootBotForaging::SearchForRestingPlace()
  * Check whether the robot has took to long to search for a place in nest
  * if StateData.TimeSearchingForPlaceInNest > StateData.MinimumSearchForPlaceInNestTime, then StartResting()
  * else ++StateData.TimeSearchingForPlaceInNest; // keep looking
* CFootBotForaging::Rest()
  * if StateData.RestingPeriodIsOver() == true, then StartExploring() // go work lazy bum
  * else ++StateData.TimeRested // keep resting you deserved it

### trace_message.h

A single log message.

* class CTraceMessage 
  * virtual EState GetMessageType() = 0; // what type of message is this?
  * virtual std::string Format(UInt32 time); // the string should be written to the trace output
  * std::string GetRobotId(); // the Id of the robot as string
  * CTraceMessage(UInt32 robotId); virtual ~CTraceMessage(); // constructor and destructor
  * UInt32 RobotId; // Id of the robot doing the logging
* class CExploreTrace : public CTraceMessage
  * Estate GetMessageType(); CExploreTrace(UInt 32 robotId);
* class CPickUpItemTrace : public CTraceMessage // same thing as usual inside the CExploreTrace
* class CReturnTrace : public CTraceMessage
* class CDropItemTrace : public CTraceMessage
* class CSearchRestingPlaceTrace : public CTraceMessage
* class CRestTrace : public CTraceMessage
* class CCollisionTrace : public CTraceMessage

### trace_message.cpp

* CTraceMessage::CTraceMessage(UInt32 robotId) : RobotId(robotId) {}
* std::string CTraceMessage::Format(UInt32 time)
  * time, RobotId, GetMessageType();
* std::string CTraceMessage::GetRobotId()
* CExploreTrace::CExploreTrace(UInt32 robotId) : CTraceMessage(robotId) {}
* EState CExploreTrace::GetMessageType() returns EXPLORING
* same thing for CPickUpItemTrace, CReturnTrace, CDropItemTrace, CSearchRestingPlaceTrace, CRestTrace, CCollisionTrace



## loop functions

### foraging_loop_functions.h

* includes:
  * <simulator/dynamic_linking/loop_functions.h>
  * <simulator/space/entities/floor_entity.h>
  * <common/utility/math.range.h>
  * <common/utility/argos_random.h>
  * <controllers/footbot_foraging/state.h>
  * <controllers/footbot_foraging/footbot_foraging.h>
* class CForagingLoopFunctions : public CLoopFunctions
  * CForagingLoopFunctions(); virtual ~CForagingLoopFunctions()
  * inline CSpace& Space() returns m_cSpace;
  * Real FoodSquareRadius; Crange<Real> ForagingArenaSideX, CRange<Real> ForagingArenaSideY;
  * std::vector<CVector2> FoodPos; CFloorEntity* Floor; CARGoSRandom::CRNG* RNG;
  * strTraceOutput; TraceOutput; // the trace output files and streams
  * writeTraceMessages(CFootBotEntity *footBot, CFootBotForaging *controller);
  * UInt32 IdForCollisionOutput; // the id for robot which we are tracing collisions
  * strCollisionOutput; CollisionOutput; // collision output files and streams
  * WriteCollisionMessages(CFootBotEntity *footBot, CFootBotForaging *controller);
  * strSummaryOutput; SummaryOutput; // the output file and stream for aggregated results
  * InitializeSummaryData(); AddSummaryData(); writeSummaryOutput();
  * ClearAllMessages(*controller); FlushOutputStream()'
  * UInt32 InitialFoodItems; // the number of food items in the arena at the start of the experiment
  * UInt32 NextFoodDrop; // the time when the next food item should be dropped
  * Real FoodDropMean; // th emean value for the experimential distributions between food drops
  * std::vector<UInt32> AggregatedEvents; // summary of all events occured.
  * FindFoodItem(CFootBotForaging::SfoodData& foodData, CVector2 pos);

### foraging_loop_functions.cpp

* includes:
  * "foraging_loop_functions.h"
  * <simulator/simulator.h>
  * <common/utility/configuration/argos_configuration.h>
  * <common/utitlity/datatypes/any.h>
  * <simulator/space/entities/footbot_entity.h>
  * <controllers/footbot_foraging/trace_message.h>
* CForagingLoopFunctions::CForagingLoopFunctions() : 
  * ForagingArenaSideX, ..SideY, Floor, RNG, AggregratedEvents
* CForagingLoopFunctions::Init()
  * try
    * Get a pointer to the floor entity
    * Get the number of food items we want to be scattered from the XML
    * Create a new RNG and distribute the food tiems uniformly in the environement
    * Get the mean time for food drops from XML as well and initializes the next food drop
    * Get the trace output file name from XML; Open the file and erase any existing contents
    * Get the collision output file name from XML; Open the file and erase its contents as well
    * Get the ID for the robot for which we are logging collisions
    * Get the summary output file name from XML; Open the file and erase its contents as well
  * catch
    * "Error parsing loop functions!"
* CForagingLoopFunctions::Reset()
  * NextFoodDrop =  RNG -> Exponential(FoodDropMean) // reset the next drop time
  * InitializeSummaryData() // zero the counters
  * Close th eoutput streams for TraceOutput, CollisionOutput, SummaryOutput
  * Open the output streams and erases their contents
  * FoodPos.clear() // reset the food position and distribute the items uniformly in the environment
* CForagingLoopFunctions::Destroy()
  * Close the TraceOutput file
* CColor CForagingLoopFunctions::GetFloorColor(const CVector2& positionOnPlane)
  * returns GRAY, BLACK, WHITE depending on positionOnPlane.GetX(), FoodPos, FoodSquareRadius
* CForagingLoopFunctions::WriteTraceMessage(*footBot, *controller)
  * Get TraceMessage
  * Trace message were gathered in the previous loop
* CForagingLoopFunctions::WriteCollisionMessages(*footbot, *conroller)
  * Get CollisoinMessages
  * CollisionMessages were gathered in the previous loop
* CForagingLoopFunctions::ClearAllMessages(*controller)
  * clear all TraceMessage and CollisionMessages
* CForagingLoopFunctions::InitializeSummaryData()
  * zero all AggregatedEvents
* CForagingLoopFunctions::AddSummaryData(*footbot, *controller)
  * ++AggregatedEvents[controller->GetState()] // get state
* CForagingLoopFunctions::WriteSummaryOutput()
  * Get Space().GetSimulationClock()
  * Write all AggregatedEvents, FoodPos.size()
* CForagingLoopFunctions::FlushOutputStreams()
  * flush TraceOutput, CollisionOutput, SummaryOutput
* CForagingLoopFunctions::FindFoodItems(foodData, pos)
  * delate food item if foot-bot pick it up; must also update the floor texture
  * LOG if can't find the food item
* CForagingLoopFunctions::PrePhysicsEngineStep()
  * Contains logic to pick and drop food items
  * If the time has come to drop a new food item, do so
  * If a robot is in the nest, drop the food item
  * If a robot is on a food item, pick it
  * Each robot can only carry one food at a time