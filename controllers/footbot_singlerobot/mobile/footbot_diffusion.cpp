/* Includes the controller's definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

/**************************************************************************/
/**************************************************************************/

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

/**************************************************************************/
/**************************************************************************/

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
}

/**************************************************************************/
/**************************************************************************/

CFootBotDiffusion::SStateData::SStateData() :
    ProbRange(0.0f, 1.0f),
    IdRange(0, NumberOfGoals-1) {}

void CFootBotDiffusion::SStateData::Init(TConfigurationNode& t_node) {
    // TODO: possible new implementation for usng random factor to swtich from RE to AE as well
    GetNodeAttribute(t_node, "max_time_in_random_epxloration", MaximumTimeInRandomExploration);
    GetNodeAttribute(t_node, "number_of_goals", NumberOfGoals);
    // TODO: Experimental feature
    RNG = CRandom::CreateRNG("argos");
    GoalId = RNG->Uniform(IdRange);
}

void CFootBotDiffusion::SStateData::Reset() {
    // Always start with RESTING
    State = RESTING;
    TimeRandomExplore = 0;
    // Let's reset the goalID as well
    GoalId = RNG->Uniform(IdRange);
    // TODO: For now we are doing the scuffed way of init table!
    for (size_t i = 0; i < NumberOfGoals; i++) {
        NavigationalTable.push_back({0,0,0});
    }
    GoalNavigationalInfo = {0,0,0};
    FoundDesignatedGoal = false;
    ReachedDesignatedGoal = false;
}

void CFootBotDiffusion::SStateData::SaveState() {
    PreviousState = State;
}

std::vector<std::vector<Real>> CFootBotDiffusion::SStateData::ByteToReal(CByteArray& b_array) {
    /*
     * Format from CByteArray sent by either a robot or goal
     * The first four byte will determined if its from a robot or a goal
     * Based on that, format accordingly
     * 
     */

}

/**************************************************************************/
/**************************************************************************/

CFootBotDiffusion::CFootBotDiffusion() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL),
    m_pcLEDs(NULL),
    m_RNG(NULL) {}

UInt32 CFootBotDiffusion::s_unIdCounter = 0;

/**************************************************************************/
/**************************************************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
    // Set robotId based on the s_unIdCounter(static variable)
    Id = s_unIdCounter++;
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
    StateData.Init(GetNode(t_node, "state"));

    Reset();

}

/**************************************************************************/
/**************************************************************************/

/*
 * The control step is called every time step and contains all the actions
 * that will be executed by the robot itself.
 * 
 * Pseudo
 *      1) Broadcast Navigational Table
 *      2) Update State
 *          2.1) Update Navigational Table 
 *              2.1.1) Take packets from every local neighbour in comm. range
 *              2.1.2) For every packet, format it from byte array to float/Real
 *              2.1.3) Compare goal by goal (row by row) based on a defined scoring system
 *                      TODO: #19 Possible ideas for determining the appeal of a navigational info
 *                      Scoring System = Sequence Number (Old is worse) + Range 
 *                      For now, angle is not a factor; But might have to include if the robot
 *                      turning takes too long (Smaller Angle -> Less Turning -> Better)
 *              2.1.4) Lack of goal info {0,0,0} will be updated with newly discovered info
 *                      from other local neighbours
 *              2.1.5) If at any chance, that the info required to reach the designated goal
 *                      is given by the designated goal (target robot); Terminate immediately - > Preference of 
 *                      original (target robot) info over third-party (other mobile robots) info
 *                      TODO: Might be an issue if a goal can be represented with two target robots
 *          2.2) Evaluate Navigational Table and find out if the designated goal info exists inside the table
 *              2.2.1) If found, update the goalNavigationalInfo
 *                  2.2.1.1) StartMoveToGoal(); State.FoundDesignatedGoal = true flag
 *              2.2.2) Else, check if TimeRandomExplore < 120 then StartRandomExplore(); else StartAggressiveExplore()
 *              2.2.3) If GoalNavigationalInfo.Range < Certain Threshold value to indicate if robot reached goal
 *                  2.2.3.1) bool ReachedDesignatedGoal = true
 */

void CFootBotDiffusion::ControlStep() {
    BroadcastNavigationalTable();
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
        case MOVE_TO_GOAL: {
            MoveToGoal();
            break;
        }
        default: {
            LOGERR << "But Why? Undefined State" << std::endl;
        }
    }

}

/**************************************************************************/
/**************************************************************************/

void CFootBotDiffusion::Reset() {
    // Reset robot state data
    StateData.Reset();
    // Empty the buffer
    m_pcRABA->ClearData();
    // Set LED Color
    m_pcLEDs->SetAllColors(CColor::BLACK);
}

/**************************************************************************/
/**************************************************************************/

void CFootBotDiffusion::EvaluateNavigationalTable() {
    // Read from Navigational table
    // can just access the goal straight by index since the table is initialize/ sorted
    // according to goalId ++
    std::vector<Real> NewGoalInfo = StateData.NavigationalTable[StateData.GoalId];
    /*
     * Check if the goal info is valid or not
     * if {0,0,0} -> No goal info hence there is no goal info for this time step
     * else -> update state's goal info with that goal info found inside the table
     * Then, update relevant flags and also checked if the goal info's range < 20cm
     * to determined that the robot had reached its goal and can switch to RESTING
     * 
     * GoalInfo = {Sequence Number, Range, Horizontal Bearing}
     */
    // a hardcoded vector to check if the goal info is valid or not
    std::vector<Real> Checker = {0,0,0};
    // TODO: #18 Possible issue where the robot might spawned right on the same spot with the target robot
    if (NewGoalInfo == Checker) {
        // goal info not valid so terminate this method straight
        return;
    }

    // goal info is valid, update relevant flag
    StateData.FoundDesignatedGoal = true;
    // Now compare both goal info from the table and the current goal info
    // Might be useless
    std::vector<Real> OldGoalInfo = StateData.GoalNavigationalInfo;
    // Update the GoalInfo with the better one among those two infos
    StateData.GoalNavigationalInfo = StateData.CompareGoalInfos(NewGoalInfo, OldGoalInfo);


    //TODO: #17 This component can be moved into the State = MOVE_TO_GOAL
    // if (GoalInfo[1] < 20) {
    //     // update relevant flags
    //     StateData.ReachedDesignatedGoal = true;
    // }
    // // TODO: Might be useless
    // else {
    //     StateData.ReachedDesignatedGoal = false;
    // }
}

/**************************************************************************/
/**************************************************************************/

void CFootBotDiffusion::UpdateState() {
    /*
     * 1) Update Navigational Table
     * 2) Evaluate Navigational Table
     * 3) TODO: #21 Unsure if i should just : Now do some if else to switch to states
     *                                          or do it inside the states
     */
    const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
    // update the table with the new tables/ info sent by the robots/ target if any
    UpdateNavigationalTable(tPackets);
    EvaluateNavigationalTable();
}

/**************************************************************************/
/**************************************************************************/

CVector2 CFootBotDiffusion::CalculateVectorToGoal() {
    // will only be used if in MOVE_TO_GOAL state
    // get the goal info
    // TODO: #22 Experimental feature
    // variable name might not reflect the value or how it was obtained
    // 
    CVector2 cAccumulator = CVector2(StateData.GoalNavigationalInfo[1], StateData.GoalNavigationalInfo[2]);
    // if the range is > 0.0f then return the vector
    if (cAccumulator.Length() > 0.0f) {
        return CVector2(1.0f, cAccumulator.Angle());
    }
    // Otherwise, returns zero
    // TODO: Might be redundant since less than 20 we would have ReachedGoal = true and State = RESTING
    else {
        LOGERR << "Checkers for less than 20cm did not work as intended!" << std::endl;
        return CVector2();
    }
}

/**************************************************************************/
/**************************************************************************/

// TODO: #23 Need to understand this clearly too
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
    }
    else {
        collision = true;
    }
    // Ensures that the vector will be always have lenght 0 for case CVector2(0,0)
    diffusionVector.Normalize();
    return -diffusionVector;
}

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////