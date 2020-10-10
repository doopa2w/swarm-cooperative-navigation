#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* 3D vector definition */
#include <argos3/core/utility/math/vector3.h>
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
    DuplicateGoal(false),
    DuplicatedGoalId(0) {}

void CFootBotDiffusion::SNavigationData::Init(TConfigurationNode& t_node) {
    GetNodeAttribute(t_node, "size_of_message", SizeOfMessage);
    GetNodeAttribute(t_node, "number_of_goals", NumberOfGoals);
    GetNodeAttribute(t_node, "duplicate_goal_or_not", DuplicateGoal);
    GetNodeAttribute(t_node, "duplicated_goal_id", DuplicatedGoalId);
}

void CFootBotDiffusion::SNavigationData::Reset(UInt32 local_id) {

    // Check whether duplicated goal or not
    if (DuplicateGoal)
        GoalId = DuplicatedGoalId;
    // else assigned based on a formulae
    else 
        GoalId = (local_id % NumberOfGoals) - 1;
    PreviousMovement = CVector2(0,0);
    GoalInfo = {{"Age", 0}, {"Angle", 0}, {"Distance", 0}};
    for (size_t i = 0; i < NumberOfGoals; ++i) {
        NavigationTable[i] = {{"Age", 0}, {"Angle", 0}, {"Distance", 0}};
    }
    SenderInfo = CVector2(0,0);

    // Debug
    LOG << "Mobile " << local_id << " initializing with Goal " << GoalId 
        << ", Duplication is " << DuplicateGoal << ", Previous Movement"
        << PreviousMovement << std::endl;     
}

void CFootBotDiffusion::SNavigationData::ConvertFromByte(CByteArray& b_array) {
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

CByteArray CFootBotDiffusion::SNavigationData::ConvertToByte(std::map<UInt32, std::map<std::string, Real>>& m_info) {
    CByteArray cBuf;
    UInt8 identifier = 99;
    cBuf << identifier;
    cBuf << '\0';

    for (auto & outer_pair : m_info) {
        for (auto & inner_pair : outer_pair.second) {
            cBuf << inner_pair.second;
            cBuf << '\0';
        }
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
    m_pcPositionS(NULL),
    m_RNG(NULL) {}

UInt32 CFootBotDiffusion::s_unIdCounter = 0; 

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
    // Set robotId based on the s_unIdCounter(static variable)
    Id = s_unIdCounter++;
    // Init actuators/ sensors handlers with the string as label to be used inside the XML file
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
    m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
    m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
    m_pcPositionS = GetSensor<CCI_PositioningSensor>("positioning");
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
            Rest();
            break;
        }
        case RANDOM_EXPLORATION: {
            RandomExplore();
            break;
        }
        case AGGRESSIVE_EXPLORATION: {
            AggressiveExplore();
            break;
        }
        case MOVE_TO_SENDER: {
            MoveToSender();
            break;
        }
        case MOVE_TO_GOAL: {
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
    NavData.Reset();

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
    // for every row in table, update navigation info

    LOGERR << "Mobile Robot " << Id << "Printing current info in table ....\n";

    for (auto & goal : NavData.NavigationTable) {
        
        // Debug
        LOGERR << "Current Goal " << goal.first << ": [Distance]: " << goal.second["Distance"]
            << " [Angle]: " << goal.second["Angle"] << "\n";

        // transform into vector then only sum it with PreviousMovement
        CVector2 currentInfo = CVector2(goal.second["Distane"], CRadians(goal.second["Angle"]));
        currentInfo+=(NavData.PreviousMovement);
        // format it back to Real to be stored inside the table
        goal.second["Angle"] = currentInfo.Angle().GetValue();
        goal.second["Distance"] = currentInfo.Length();


        LOGERR << "New Goal " << goal.first << ": [Distance]: " << goal.second["Distance"]
            << " [Angle]: " << goal.second["Angle"] << "\n";

    }
    
}

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
                            // Update relevant flags here
                            StateData.FoundDesignatedGoal = true;
                            StateData.State = MOVE_TO_SENDER;

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
            UInt32 age;
            cBuf >> age;
            NavData.NavigationTable[identifier]["Age"] = age;
            NavData.NavigationTable[identifier]["Angle"] = t_packet.HorizontalBearing.GetValue();
            NavData.NavigationTable[identifier]["Distance"] = t_packet.Range;
            // If found target robot that represents local robot's deisgnated goal
            if (identifier == NavData.GoalId) {
                // Update relevant flags and state
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


    // Update Odometry firstmost for both table, goalInfo, SenderInfo

    // Update Table

    // Check whether foundDesignatedGoal

    // Check whether reachedSender based on SenderInfo

    // Check whether reachedDesignatedGoal based on GoalInfo 
}

