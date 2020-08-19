# ARGos-Foraging



## Folder Structure

1. controllers
   - footbot_foraging.cpp
   - footbot_foraging.h
   - footbot_foraging_nr.cpp
   - footbot_foraging_nr.h
   - ground.h
   - state.h
   - trace_message.cpp
   - trace_message.h

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
  * 