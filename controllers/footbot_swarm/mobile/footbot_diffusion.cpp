#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

CFootBotDiffusion::SDiffusionParams::SDiffusionParams() :
    GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void CFootBotDiffusion::SDiffusionParams::Init(TConfigurationNode& t_node) {
    CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
    GetNodeAttribute(t_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
    // Set the threshold range for determining if the robot should proceed straight forward
    // based on relative distance between the robot and the closest obstacle detected
    GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                            ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
    // parse it as delta in XML file
    GetNodeAttribute(t_node, "delta", Delta);   

    
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void CFootBotDiffusion::SWheelTurningParams::Init(TConfigurationNode& t_node) {
    CDegrees cAngle;
    // Init and parse all relevant params into XML file for the wheel manipulation
    GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
    NoTurnAngleThreshold = ToRadians(cAngle);
    GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
    SoftTurnOnAngleThreshold = ToRadians(cAngle);
    GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
    HardTurnOnAngleThreshold = ToRadians(cAngle);
    GetNodeAttribute(t_node, "max_speed", MaxSpeed);
    GetNodeAttribute(t_node, "random_exploration_speed", RESpeed);
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

CFootBotDiffusion::SStateData::SStateData() :
    ProbRange(0.0f, 1.0f) {}

void CFootBotDiffusion::SStateData::Init(TConfigurationNode& t_node) {
    GetNodeAttribute(t_node, "max_time_in_random_exploration", MaximumTimeInRandomExploration);
}

void CFootBotDiffusion::SStateData::Reset() {
    // Always start with REST
    State = RESTING;
    TimeRandomExplore = 0;
    FoundDesignatedGoal = false;
    ReachedSender = false;
    ReachedGoal = false;
}

void CFootBotDiffusion::SStateData::SaveState() {
    PreviousState = State;
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

CFootBotDiffusion::SNavigationData::SNavigationData() :
    NumberOfGoals(1) {}

void CFootBotDiffusion::SNavigationData::Init(TConfigurationNode& t_node) {
    GetNodeAttribute(t_node, "size_of_message", SizeOfMessage);
    GetNodeAttribute(t_node, "number_of_goals", NumberOfGoals);
}

void CFootBotDiffusion::SNavigationData::Reset(UInt8 local_id) {

    // Assigned goal id based on a formulae
    GoalId = (local_id % NumberOfGoals) - 1;
    // TODO: #29 Temporary fix for unsigned modulo issue
    if (GoalId == 255) GoalId = NumberOfGoals - 1;

        
    PreviousMovement = CVector2(0,0);
    SenderInfo = CVector2(0,0); 
    GoalInfo = {{"Age", 0}, {"Angle", 0}, {"Distance", 0}};


    // Debug
    LOG << "Mobile " << local_id << " initializing with Goal " << GoalId 
        << ", Previous Movement" << PreviousMovement << " and Number of Goals = " 
        << NumberOfGoals << std::endl;
    
    for (size_t i = 0; i < NumberOfGoals; ++i) {
        LOG << "Generated Row " << i << "   ";
        NavigationTable[i] = {{"Age", 0}, {"Angle", 0}, {"Distance", 0}};
    }
    LOG << "." << std::endl;
    

    
}

std::map<UInt8, std::map<std::string, Real>> CFootBotDiffusion::SNavigationData::ConvertFromByte(CByteArray& b_array) {
    std::map<UInt8, std::map<std::string, Real>> NeighbourTable;
    CByteArray cBuf = b_array;
    // Get first 8 bytes as UInt8
    UInt8 identifier;
    cBuf >> identifier;

    for (size_t i = 0; i < NumberOfGoals; ++i) {
        Real age, distance, angle;
        UInt8 factor = i * 48;
        /*
         * Proceed to extract the info accordinly
         * 0      Identifier (Mobile = 0; Static = GoalID) ~ Extracted out!
         * 
         * 0-3      0
         * 4-15     Angle
         * 16-19     0
         * 20-31    EstimateDistance
         * 32-35    0
         * 36-47    Seq
         * 48-51    0
         * 
         * 52-63    Angle
         * 64-67    0
         * 68-79    EstimateDistance
         * 80-83    0
         * 84-95    Seq
         * 96-99    0
         */
        *cBuf(4 + factor, 16 + factor) >> age;
        *cBuf(20 + factor, 32 + factor) >> angle;
        *cBuf(36 + factor, 48 + factor) >> distance;
        // Append relevant info into respective goal
        NeighbourTable[i] = {{
                {"Age", age},
                {"Angle", angle},
                {"Distance", distance}       
            }};
    }
    return NeighbourTable;
}

CByteArray CFootBotDiffusion::SNavigationData::ConvertToByte(std::map<UInt8, std::map<std::string, Real>>& m_info) {
    CByteArray cBuf;
    UInt8 identifier = 99;
    cBuf << identifier;
    cBuf << '\0';

    LOG << "Mobile Robot is broadcasting this table: \n";

    for (auto & outer_pair : m_info) {
        
        // Debug
        LOG << "[Goal " << outer_pair.first << ": ";
        for (auto & inner_pair : outer_pair.second) {

            LOG << "[" << inner_pair.first << ": " << inner_pair.second << "], ";

            cBuf << inner_pair.second;
            cBuf << '\0';
        }

        LOG << "\n";
    }
    return cBuf;
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

CFootBotDiffusion::CFootBotDiffusion() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL),
    m_pcLEDs(NULL),
    m_RNG(NULL) {}

UInt8 CFootBotDiffusion::s_unMobileCounter = 0; 

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
    // Set robotId based on the s_unIdCounter(static variable)
    Id = s_unMobileCounter++;
    // Init actuators/ sensors handlers with the string as label to be used inside the XML file
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
    m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
    m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
    m_RNG = CRandom::CreateRNG("argos"); 
    /*
    * Parses the configuration file
    */
    DiffusionParams.Init(GetNode(t_node, "diffusion"));
    WheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
    StateData.Init(GetNode(t_node, "state_data"));
    NavData.Init(GetNode(t_node, "nav_data"));

    Reset();

}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
/*
 * The control step is called every time step and contains all the actions
 * that will be executed by the robot itself.
 *
 */

void CFootBotDiffusion::ControlStep() {

    UpdateState();

    switch (StateData.State) {
        case RESTING: {
            LOG << "Mobile Robot " << Id << " is RESTING!\n" << std::endl;
            Rest();
            break;
        }
        case RANDOM_EXPLORATION: {
            LOG << "Mobile Robot " << Id << " is RANDOM_EXPLORE!\n" << std::endl;
            RandomExplore();
            break;
        }
        case AGGRESSIVE_EXPLORATION: {
            LOG << "Mobile Robot " << Id << " is AGGRESSIVE_EXPLORE!\n" << std::endl;
            AggressiveExplore();
            break;
        }
        case MOVE_TO_SENDER: {
            LOG << "Mobile Robot " << Id << " is MOVE_TO_SENDER!\n" << std::endl;
            MoveToSender();
            break;
        }
        case MOVE_TO_GOAL: {
            LOG << "Mobile Robot " << Id << " is MOVE_TO_GOAL!\n" << std::endl;
            MoveToGoal();
            break;
        }
        default: {
            LOGERR << "But Why? Undefined State" << std::endl;
        }
    }

    BroadcastNavigationTable();

}


void CFootBotDiffusion::Reset() {
    // Reset robot state data
    StateData.Reset();
    // Reset Navigation data
    NavData.Reset(Id);

    // Empty the buffer
    m_pcRABA->ClearData();
    // Set LED Color
    m_pcLEDs->SetAllColors(CColor::BLACK);
    
    // Debug = Pass
    LOG << "Mobile Robot " << Id << " successfully reseted! " << std::endl;
}


/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void CFootBotDiffusion::UpdateOdometry() {
    // for every row in table, update navigation info regardless of states

    LOG << "Mobile Robot " << Id << " Printing current info in table ....\n";

    // If current_state == MOVE_TO_SENDER, update the odometry for SenderInfo as well
    if (StateData.State == MOVE_TO_SENDER) {
        NavData.SenderInfo+=(NavData.PreviousMovement);
    }
    else if (StateData.State == MOVE_TO_GOAL) {
        CVector2 currentGoalInfo = CVector2(NavData.GoalInfo["Distance"], CRadians(NavData.GoalInfo["Angle"]));
        currentGoalInfo+=(NavData.PreviousMovement);
        NavData.GoalInfo["Angle"] = currentGoalInfo.Angle().GetValue();
        NavData.GoalInfo["Distance"] = currentGoalInfo.Length();
    }
    else {
        for (auto & goal : NavData.NavigationTable) {
                
            // Debug
            LOG << "Current Goal " << goal.first << ": [Distance]: " << goal.second["Distance"]
                << " [Angle]: " << goal.second["Angle"] << "\n";

            /*
            * If age is 0, it means that there was no info for the goal. Hence, skip it.
            * This is because only target robot is allowed to increase age for its info
            * Those with age is 0 is immediately assumed to contain no vital info
            * 
            */
            if (goal.second["Age"] == 0) {
                LOG << "Not Updating Odometry!\n";
                continue;   
            }
            else {
                // transform into vector then only sum it with PreviousMovement
                CVector2 currentInfo = CVector2(goal.second["Distane"], CRadians(goal.second["Angle"]));
                currentInfo+=(NavData.PreviousMovement);
                // format it back to Real to be stored inside the table
                goal.second["Angle"] = currentInfo.Angle().GetValue();
                goal.second["Distance"] = currentInfo.Length();
            }

        }
    }
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void CFootBotDiffusion::UpdateNavigationTable(const CCI_RangeAndBearingSensor::TReadings& t_packets) {
    /*
     * This method is different from UpdateOdometry() whereby it is responsible
     * for updating the goal info if there is a better info sent by other robots.
     * 
     * However, if found info for designated goal, call UpdateNavigationBehaviour()
     * since the function is responsible for updating other info beside the table as well
     * as compared to the UpdateNavigationTable()
     */

    // Create a temporary container to hold neighbour tables
    std::map<UInt8, std::map<std::string, Real>> neighboursTable;
    // Iterate over every packets recieved in a time step
    for (CCI_RangeAndBearingSensor::SPacket t_packet : t_packets) {
        // Put the packet content into a byte array container
        CByteArray cBuf = t_packet.Data;
        UInt8 identifier;
        // Extract the first 8 bytes (Identifier) in the packet
        cBuf >> identifier;
        
        /*
         * Based on the identifier, do the following:
         * identifier == 99 (mobile robot) -> read the rest of content and compare + replace if need be
         * identifier != 99 (target robot) -> ignore the rest and compare + replace using the relative
         *                                      distance and angle between local robot and target robot
         */
        if (identifier == 99) {
            neighboursTable = NavData.ConvertFromByte(t_packet.Data);
            // Iterate Both neigboursTable and local table simulataneously
            for (auto it1 = NavData.NavigationTable.begin(), it2 = neighboursTable.begin();
                it1 != NavData.NavigationTable.end(); ++it1, ++it2)
                {
                    // Make sure to skip if info is age 0 - > Does not contain any info hence save time
                    // by not checking further
                    if (it2->second["Age"] == 0) continue;
                    /*
                    * If not goal, compute the new vector of neighbour table info and relative distance and
                    * angle between local robot and neighbour robot's position to get the accurate info
                    * to compare with.
                    */
                    CVector2 neighbourGoalInfo = CVector2(it2->second["Distance"], CRadians(it2->second["Angle"]));
                    neighbourGoalInfo+=(CVector2(t_packet.Range, t_packet.HorizontalBearing));
                    /*
                    * Compare both infos
                    * For now, compare based on distance and age of info for simplicity.
                    * Angle is not considered since rotation speed for robot should not be detrimental
                    * to the robot's required time to reach a destination.
                    * 
                    */
                    if (it2->second["Age"] >= it1->second["Age"] && neighbourGoalInfo.Length() < it1->second["Distance"]) {
                        /*
                        * If comparing designated goal infos, update navigation behaviour if new info
                        * is better than local info and switch state to MOVE_TO_ROBOT and move towards
                        * sender robot position when it sent the table containing that better info
                        *
                        */

                        // Replace the goal info in local table
                        it1->second["Age"] = it2->second["Age"];
                        it1->second["Angle"] = neighbourGoalInfo.Angle().GetValue();
                        it1->second["Distance"] = neighbourGoalInfo.Length();
                    } // Otherwise, retain info in local table by doing nothing.

                   /*
                    * If the infos related to local robot's designated goal, then compare GoalInfo if its not empty
                    * Else, just replace sender's info in the local table and update flags to start switching to
                    * MOVE_TO_ROBOT and later MOVE_TO_GOAL
                    * 
                    */
                    if (it1->first == NavData.GoalId) {
                        if (!StateData.FoundDesignatedGoal) {
                            // If not found yet, the goal info is empty so skip comparison and replace immediately.
                            NavData.GoalInfo["Age"] = it2->second["Age"];
                            NavData.GoalInfo["Angle"] = it2->second["Angle"];
                            NavData.GoalInfo["Distance"] = it2->second["Distance"];
                            // Update sender info as well;
                            NavData.SenderInfo = CVector2(t_packet.Range, t_packet.HorizontalBearing);
                            // Update relevant flags here only if there waas no goal info from target robot
                            if (StateData.State != MOVE_TO_GOAL) {
                                StateData.FoundDesignatedGoal = true;
                                StateData.State = MOVE_TO_SENDER;
                            }
                        }
                        else {
                            if (it2->second["Age"] >= NavData.GoalInfo["Age"] && it2->second["Distance"] < NavData.GoalInfo["Distance"]) {
                                NavData.GoalInfo["Age"] = it2->second["Age"];
                                NavData.GoalInfo["Angle"] = it2->second["Angle"];
                                NavData.GoalInfo["Distance"] = it2->second["Distance"]; 
                                // Update sender info as well;
                                NavData.SenderInfo = CVector2(t_packet.Range, t_packet.HorizontalBearing);
                            } // Otherwise, retain it 
                        }                        
                    }
                    
                } // end inner for loop

        } // Otherwise, do accordinly for target robot
        else {
            /*
             * No need for comparison since we are receiving goal infos directly from the target robot itself
             * This is to ensure that the local robot will always update the goal info even if it move further 
             * away from the target robot since comparing infos will only take the best info which maybe outdated
             */
            Real age;
            *cBuf(4, 16) >> age;
            NavData.NavigationTable[identifier]["Age"] = age;
            NavData.NavigationTable[identifier]["Angle"] = t_packet.HorizontalBearing.GetValue();
            NavData.NavigationTable[identifier]["Distance"] = t_packet.Range;
            // If found target robot that represents local robot's deisgnated goal
            if (identifier == NavData.GoalId) {
                /*
                 * Update relevant flags and state regardless since info given from target robot is prioritized
                 */
                StateData.FoundDesignatedGoal = true;
                StateData.State = MOVE_TO_GOAL; // Skip moving to sender since this is a target robot
                NavData.GoalInfo["Age"] = age;
                NavData.GoalInfo["Angle"] = t_packet.HorizontalBearing.GetValue();
                NavData.GoalInfo["Distance"] = t_packet.Range;
            } // Otherwise, retain info.
        }             

    } // end outer for loop

}

void CFootBotDiffusion::BroadcastNavigationTable() {
    CByteArray cBuf = NavData.ConvertToByte(NavData.NavigationTable);
    m_pcRABA->SetData(cBuf);
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void CFootBotDiffusion::UpdateState() {
    // One of the important methods since this encompases the action of robot each time step
    const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
    // Update Odometry firstmost for both table, goalInfo, SenderInfo
    if (StateData.State != RESTING) 
        UpdateOdometry();
    else
        return;
    // Update Table
    UpdateNavigationTable(tPackets);

    // Check whether reachedSender based on SenderInfo
    if (StateData.State == MOVE_TO_SENDER && NavData.SenderInfo.Length() < 5.0f) {
        StateData.ReachedSender = true;
        StateData.State = MOVE_TO_GOAL;
    }

    // Check whether reachedDesignatedGoal based on GoalInfo 
    if (StateData.State == MOVE_TO_GOAL && NavData.GoalInfo["Distance"] < 20.0f) {
        StateData.ReachedGoal = true;
        StateData.State = RESTING;
    }
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

CVector2 CFootBotDiffusion::CalculateVectorToGoal(bool b_goalorsender) {
    CVector2 cAccumulator;
    if (b_goalorsender)
        cAccumulator = CVector2(NavData.GoalInfo["Distance"], CRadians(NavData.GoalInfo["Angle"]));
    else
        cAccumulator = NavData.SenderInfo;
    
    // Debug
    LOG << "Mobile Robot " << Id << " Calculated the vector: " << cAccumulator.Length() << ", "
        << cAccumulator.Angle() << std::endl;

    // if the range is > 0.0f then return the vector
    if (cAccumulator.Length() > 0.0f) {
        cAccumulator.Normalize();
        return cAccumulator;

    }
    // Otherwise, returns zero
    else {
        return CVector2();
    }
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

CVector2 CFootBotDiffusion::DiffusionVector(bool& collision) {
    // Get readings from the proximity sensor
    const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    // sum them up together since we need to take all readings from every sensors positioned at different parts
    // of the body
    CVector2 diffusionVector;
    for (size_t i = 0; i < tProxReads.size(); ++i) {
        diffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
    }
    /*
     * If the angle of the vector is small enough + closest obstacle is far apart = IGNORE VECTOR and GO STRAIGHT
     * Otherwise, return it
     * 
     * Params delta is threshold for the distance apart to actually ignore the respective obstacle
     */
    if (DiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(diffusionVector.Angle()) &&
            diffusionVector.Length() < DiffusionParams.Delta) {
        collision = false;
        return CVector2::X;
    }
    else {
        collision = true;
        diffusionVector.Normalize();
        return -diffusionVector;
    }
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void CFootBotDiffusion::SetWheelSpeedsFromVector(const CVector2& heading) {
    // Get the heading angle and normalized it in the range of [-pi, pi]
    CRadians cHeadingAngle = heading.Angle().SignedNormalize();
    // Get the length of the heading vector
    Real fHeadingLength = heading.Length();
    // Clamp the speed so that it does not go beyond MaxSpeed
    Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, WheelTurningParams.MaxSpeed);
    // Get the previous Turning Mechanism
    SWheelTurningParams::ETurningMechanism oldTurningMechanism = WheelTurningParams.TurningMechanism;

    // Turning State Switching Conditions
    if (Abs(cHeadingAngle) <= WheelTurningParams.NoTurnAngleThreshold) {
        // No Turn + vecry small heading angle
        WheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
    }
    else if (Abs(cHeadingAngle) > WheelTurningParams.HardTurnOnAngleThreshold) {
        // Hard Turn + very large heading angle
        WheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
    }
    else if (Abs(cHeadingAngle) > WheelTurningParams.SoftTurnOnAngleThreshold &&
         WheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
        // Soft Turn + heading angle is in between the NO_TURN and HARD_TURN
        WheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
    }
    /*
     * An if block for collision
     * 
     * Since the current state of robot is forced to do either SOFT_TURN or HARD_TURN and 
     * that the previous state was different from current state.
     * It is assumed that the robot  is facing possible collision; hence LOG it only at the start.
     * 
     * As for the proceeding steps, ignore due to previous state == current state as long the 
     * robot is forced to do either SOFT_TURN or HARD_TURN.
     *
     */
    if (WheelTurningParams.TurningMechanism != SWheelTurningParams::NO_TURN &&
         oldTurningMechanism != WheelTurningParams.TurningMechanism) {
        CollisionMessages.push_back(new CCollisionTrace(Id));
    }
    /*
     * Update Wheel Speeds to executes turning mechanism
     * 1) Switch case for all turning mechanisms.
     * 2) Apply the new wheel speeds to the appropriate wheels
     * 
     */
    Real fSpeed1, fSpeed2;
    switch (WheelTurningParams.TurningMechanism) {
        case SWheelTurningParams::NO_TURN: {
            // Just go straight with no speed differences between the left and right wheels
            fSpeed1 = fBaseAngularWheelSpeed;
            fSpeed2 = fBaseAngularWheelSpeed;
            
            // Debug = pass
            LOG << "Mobile Robot " << Id << " employing NO_TURN! "<< std::endl;

            break;
        }
        case SWheelTurningParams::SOFT_TURN: {
            // Both wheels still go straight but one of the wheels is faster than the other
            Real fSpeedFactor = (WheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) /
                WheelTurningParams.HardTurnOnAngleThreshold;
            fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);   // Slower
            fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);   // Faster

            // Debug = pass
            LOG << "Mobile Robot " << Id << " employing SOFT_TURN! "<< std::endl;

            break;
        }
        case SWheelTurningParams::HARD_TURN: {
            // Both wheels have opposite speeds
            fSpeed1 = -WheelTurningParams.MaxSpeed;
            fSpeed2 = WheelTurningParams.MaxSpeed;

            // Debug = pass
            LOG << "Mobile Robot " << Id << " employing HARD_TURN! "<< std::endl;

            break;
        }
    }
    // Apply the new fSpeed1, fSpeed2 to the wheels
    Real fLeftWheelSpeed, fRightWheelSpeed;
    if (cHeadingAngle > CRadians::ZERO) {
        // TUrn Left
        fLeftWheelSpeed = fSpeed1;
        fRightWheelSpeed = fSpeed2;
    }
    else {
        // Turn Right
        fLeftWheelSpeed = fSpeed2;
        fRightWheelSpeed = fSpeed1;
    }
    // Set the wheel speeds to the handlers
    m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void CFootBotDiffusion::StartRandomExploration() {
    if (StateData.State == RANDOM_EXPLORATION) {
        LOGERR << "Already in RANDOM_EXPLORATION!" << std::endl;
    }
    StateData.SaveState();
    StateData.State = RANDOM_EXPLORATION;
    TraceMessages.push_back(new CRandomExplorationTrace(Id));
}

void CFootBotDiffusion::StartAggressiveExploration() {
    if (StateData.State == AGGRESSIVE_EXPLORATION) {
        LOGERR << "Already in AGGRESSIVE_EXPLORATION!" << std::endl;
    }
    StateData.SaveState();
    StateData.State = AGGRESSIVE_EXPLORATION;
    TraceMessages.push_back(new CAggressiveExplorationTrace(Id));
}

void CFootBotDiffusion::StartMoveToSender() {
    if (StateData.State == MOVE_TO_SENDER) {
        LOGERR << "Already in MOVE_TO_SENDER!" << std::endl;
    }
    StateData.SaveState();
    StateData.State = MOVE_TO_SENDER;
    TraceMessages.push_back(new CMoveToGoalTrace(Id));
}

void CFootBotDiffusion::StartMoveToGoal() {
    if (StateData.State == MOVE_TO_GOAL) {
        LOGERR << "Already in MOVE_TO_GOAL!" << std::endl;
    }
    StateData.SaveState();
    StateData.State = MOVE_TO_GOAL;
    TraceMessages.push_back(new CMoveToGoalTrace(Id));
}

void CFootBotDiffusion::StartResting() {
    if (StateData.State == RESTING) {
        LOGERR << "Already in RESTING!" << std::endl;
    }
    StateData.SaveState();
    StateData.State = RESTING;
    TraceMessages.push_back(new CRestingTrace(Id));
    m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void CFootBotDiffusion::RandomExplore() {
    // Check if we found the designated goal info?
    if (StateData.FoundDesignatedGoal == true && StateData.ReachedSender == true) {
        StartMoveToGoal();
        return;
    }
    else if (StateData.FoundDesignatedGoal == true && StateData.ReachedSender == false) {
        StartMoveToSender();
        return;
    }
    else {
        // if robot spent too long in random explore, start aggressive epxlore
        if (StateData.TimeRandomExplore > StateData.MaximumTimeInRandomExploration) {
            StartAggressiveExploration();
            return;
        }
        else {
            // Get the diffusion vector to perform the obstacle avoidance
            bool collision;
            CVector2 diffusion = DiffusionVector(collision);
            ++StateData.TimeRandomExplore;
            // control the speed by MaxSpeed * 0.5
            SetWheelSpeedsFromVector(WheelTurningParams.RESpeed * diffusion);
            // Update PreviousMovement
            NavData.PreviousMovement = WheelTurningParams.RESpeed * diffusion;
        }
    }
    
}

void CFootBotDiffusion::AggressiveExplore() {
    // Check if we found the designated goal info yet?
    if (StateData.FoundDesignatedGoal == true && StateData.ReachedSender == true) {
        StartMoveToGoal();
        return;
    }
    else if (StateData.FoundDesignatedGoal == true && StateData.ReachedSender == false) {
        StartMoveToSender();
        return;
    }
    else {
        // Get the diffusion vector to perform the obstacle avoidance
        bool collision;
        CVector2 diffusion = DiffusionVector(collision);
        // use MaxSpeed (twice the speed of Random Explore)
        SetWheelSpeedsFromVector(WheelTurningParams.MaxSpeed * diffusion);
        // Update PreviousMovement
        NavData.PreviousMovement = WheelTurningParams.MaxSpeed * diffusion;
    }
}

void CFootBotDiffusion::MoveToSender() {
    // Check if we have reached the sender
    if (StateData.ReachedSender) {
        StartMoveToGoal();
        return;
    }
    else {
        bool collision;
        CVector2 diffusion = DiffusionVector(collision);
        // if faced possible collision
        if (collision) {
            CRange<Real> range(0.2f, 0.7f);
            double r = m_RNG->Uniform(range);
            CVector2 NewVector = r * diffusion + (1.0 - r) * CalculateVectorToGoal(false);
            SetWheelSpeedsFromVector(WheelTurningParams.MaxSpeed * NewVector);
            // Upadte odometry info inside the table
            NavData.PreviousMovement = WheelTurningParams.MaxSpeed * NewVector;
        }
        else {
            CRange<Real> range(0.0f, 0.2f);
            double r = m_RNG->Uniform(range);
            CVector2 NewVector = r * diffusion + (1.0 - r) * CalculateVectorToGoal(false);
            SetWheelSpeedsFromVector(WheelTurningParams.MaxSpeed * NewVector);
            // Upadte odometry info inside the table
            NavData.PreviousMovement = WheelTurningParams.MaxSpeed * NewVector;
        } 
    }

}

void CFootBotDiffusion::MoveToGoal() {
    // Check if we are close to the goal?
    if (StateData.ReachedGoal) {
        StartResting();
        return;
    }
    else {
        bool collision;
        CVector2 diffusion = DiffusionVector(collision);
        // if faced possible collision
        if (collision) {
            CRange<Real> range(0.2f, 0.7f);
            double r = m_RNG->Uniform(range);
            CVector2 NewVector = r * diffusion + (1.0 - r) * CalculateVectorToGoal(true);
            SetWheelSpeedsFromVector(WheelTurningParams.MaxSpeed * NewVector);
            // Upadte odometry info inside the table
            NavData.PreviousMovement = WheelTurningParams.MaxSpeed * NewVector;
        }
        else {
            CRange<Real> range(0.0f, 0.2f);
            double r = m_RNG->Uniform(range);
            CVector2 NewVector = r * diffusion + (1.0 - r) * CalculateVectorToGoal(true);
            SetWheelSpeedsFromVector(WheelTurningParams.MaxSpeed * NewVector);
            // Upadte odometry info inside the table
            NavData.PreviousMovement = WheelTurningParams.MaxSpeed * NewVector;
        }
    }
}

void CFootBotDiffusion::Rest() {
    if (!StateData.ReachedGoal) {
        // Debug = pass
        LOG << "Mobile Robot " << Id << " Starting Random Exploration!" << std::endl;
        StartRandomExploration();
    }
    else {
        // Debug = Pass
        LOG << "Mobile Robot " << Id << ": Done tasks!\n The goal was: " << NavData.GoalId << std::endl;
    }

}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */

REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")
