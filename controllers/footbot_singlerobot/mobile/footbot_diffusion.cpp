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
// Possible error on the IdRange(min, max)
CFootBotDiffusion::SStateData::SStateData() :
    ProbRange(0.0f, 1.0f),
    IdRange(0, NumberOfGoals) {}

void CFootBotDiffusion::SStateData::Init(TConfigurationNode& t_node) {
    // TODO: possible new implementation for usng random factor to swtich from RE to AE as well
    GetNodeAttribute(t_node, "max_time_in_random_exploration", MaximumTimeInRandomExploration);
    GetNodeAttribute(t_node, "number_of_goals", NumberOfGoals);
    GetNodeAttribute(t_node, "size_of_message", SizeOfMessage);
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
    GoalNavigationalInfo = {
        {"SequenceNumber", 0}, {"EstimateDistance", 0}, {"Angle", 0}
    };
    for (size_t i = 0; i < NumberOfGoals; ++i) {
        NavigationalTable[i] = GoalNavigationalInfo;
    }
    NeighboursNavigationalInfo = {
         {"EstimateDistance", 0}, {"Angle", 0}
    };
    FoundDesignatedGoal = false;
    ReachedDesignatedGoal = false;
}

void CFootBotDiffusion::SStateData::SaveState() {
    PreviousState = State;
}

/**************************************************************************/
/**************************************************************************/

// TODO: Might Need to Name Change the two following methods
std::map<UInt32, std::map<std::string, Real>> CFootBotDiffusion::SStateData::ByteToReal(CByteArray& b_array) {
    std::map<UInt32, std::map<std::string, Real>> NeighboursTable;
    CByteArray cBuf = b_array;
    // Get first byte 
    UInt32 identifer = cBuf[0];

    for (size_t i = 0; i < NumberOfGoals; ++i) {
        UInt32 SeqNum;
        Real Range, Angle;
        // TODO: Experimental feature: a step size of 45
        UInt16 factor = i * 45;
        /*
        * Proceed to extract the info accordinly
        * 0        Identifier (Mobile = 0; Static = GoalID)
        * 1-4      0
        * 5-8      Seq
        * 9-12     0
        * 13-24    EstimateDistance
        * 25-28    0
        * 29-40    Angle
        * 41-44    0
        * 
        * 45       Placeholder
        * 46-49    0
        * 50-53    Seq
        * 54-57    0
        * 58-69    EstimateDistance
        * 70-73    0
        * 74-85    Angle
        * 86-89    0
        * 
        * Size for a navigational info = 45 bytes (0 to 44)   
        */
        *cBuf(5 + factor, 9 + factor) >> SeqNum;
        *cBuf(13 + factor, 25 + factor) >> Range;
        *cBuf(29 + factor, 41 + factor) >> Angle;
        // Finally insert into the respective goal/ row
        NeighboursTable[i] = {
            {
                {"SequenceNumber", SeqNum},
                {"EstimateDistance", Range},
                {"Angle", Angle}        
            }
        };
    }
    return NeighboursTable;
}

/**************************************************************************/
/**************************************************************************/

CByteArray CFootBotDiffusion::SStateData::RealToByte(std::map<UInt32, std::map<std::string, Real>>& m_info) {
    CByteArray cBuf;
    // For mobile robots, append 0 else append 1 + GoalId that the target robot represents for target/goal robots
    UInt32 placeHolder = 0;
    cBuf << placeHolder;
    cBuf << '\0';
    
    for (auto & outer_pair : m_info) {
        /*
         * For every row/ goal, do:
         * Append the goalId first followed by four 0's
         */
        cBuf << placeHolder;
        cBuf << '\0';
        for (auto & inner_pair : outer_pair.second) {
            /*
             * For every goal's info, do:
             * Append the Seq followed by four 0's
             * Repeat this for distance and angle as well
             */
            cBuf << inner_pair.second;
            cBuf << '\0';
        }
        /*
         * Before going to the next row/goal, do:
         * Append a '\0' to separate the current and the next goal
         */
        cBuf << '\0';
    }
    return cBuf;
}

/**************************************************************************/
/**************************************************************************/

std::map<std::string, Real> CFootBotDiffusion::SStateData::CompareGoalInfos(std::map<std::string, Real>& m_info1, std::map<std::string, Real>& m_info2) {
    /*
     * Scoring function to calculate quality of info based on
     * Sequence Number (Relative age of the info) & EstimateDistance (Relative Distance)
     * The lower the score, the better the info.
     * 
     * TODO: #26 Might add Angle as another factor as well?
     * 
     * AScore, BScore > AScore 
     */
    UInt32 AScore, BScore;
    AScore = m_info1["SequenceNumber"] * m_info1["EstimateDistance"] + m_info1["EstimateDistance"];
    BScore = m_info2["SequenceNumber"] * m_info2["EstimateDistance"] + m_info2["EstimateDistance"];

    if (AScore > BScore) {
        return m_info2;
    }
        
    else if (AScore < BScore){
        return m_info1;
    }
    // if both equal, return either one 
    else {
        return m_info1;
    }
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
    UpdateState();
    BroadcastNavigationalTable();

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
    std::map<std::string, Real> NewGoalInfo = StateData.NavigationalTable[StateData.GoalId];
    /*
     * Check if the goal info is valid or not
     * if {0,0,0} -> No goal info hence there is no goal info for this time step
     * else -> update state's goal info with that goal info found inside the table
     * Then, update relevant flags and also checked if the goal info's range < 20cm
     * to determined that the robot had reached its goal and can switch to RESTING
     * 
     * GoalInfo = {Sequence Number, EstimateDistance, Angle}
     */
    // a hardcoded map to check if the goal info is valid or not
    std::map<std::string, Real> Checker = {
        {"SequenceNumber", 0},
        {"EstimateDistance", 0},
        {"Angle", 0}
    };
    // TODO: #18 Possible issue where the robot might spawned right on the same spot with the target robot
    if (NewGoalInfo == Checker) {
        // goal info not valid so terminate this method straight
        return;
    }
    // goal info is valid, update relevant flag
    StateData.FoundDesignatedGoal = true;
    // Now compare both goal info from the table and the current goal info
    // Might be useless
    std::map<std::string, Real> OldGoalInfo = StateData.GoalNavigationalInfo;
    // Update the GoalInfo with the better one among those two infos
    // Ensured that OldGoalInfo which is {0, 0, 0} is ignored instead
    if (OldGoalInfo["EstimateDistance"] == 0 && OldGoalInfo["Angle"] == 0) {
        StateData.GoalNavigationalInfo = NewGoalInfo;
    }
    else {
        StateData.GoalNavigationalInfo = StateData.CompareGoalInfos(NewGoalInfo, OldGoalInfo);
    }

    // Check whether the robot reached the goal (within 20cm proximity) based on the new info
    if (StateData.GoalNavigationalInfo["EstimateDistance"] <= 20.0f) {
        StateData.ReachedDesignatedGoal = true; // switch to true
    } // otherwise, don't change since default = false
}

/**************************************************************************/
/**************************************************************************/

void CFootBotDiffusion::UpdateNavigationalTable(const CCI_RangeAndBearingSensor::TReadings& t_packets) {
    /*
     * This is where own table is updated accordinly based on the local neihgbours tables
     * 
     * 
     * TOOD: Issue: Target Robot appending relative distance/ angle into the table's info
     *              Might cause an issue since we are moving first based on the NeighboursNavigationalInfo
     *              before proceeding towards the goal using info from the updated table.
     * 
     */
    std::map<UInt32, std::map<std::string, Real>> NeighboursTable;

    for (CCI_RangeAndBearingSensor::SPacket t_packet : t_packets) {
        // StateData.NeighboursNavigationalInfo["EstimateDistance"] = t_packet.Range;
        // StateData.NeighboursNavigationalInfo["Angle"] = t_packet.HorizontalBearing;
        if (t_packet.Data[0] == 0) {
            // This is mobile robot
            NeighboursTable = StateData.ByteToReal(t_packet.Data);
        }
        else {
            // This is target robot
            NeighboursTable = StateData.ByteToReal(t_packet.Data);
            /*
             * Update the target robot's goal info with its navigational info
             */
            NeighboursTable[t_packet.Data[0]-1]["EstimateDistance"] = t_packet.Range;
            NeighboursTable[t_packet.Data[0]-1]["Angle"] = t_packet.HorizontalBearing.GetValue();
        }

        // Iterate two maps per row since we know the keys are identical and in order with one another
        for (auto it1 = StateData.NavigationalTable.begin(), it2 = NeighboursTable.begin();
             it1 != StateData.NavigationalTable.end();
             ++it1, ++it2) {
            // Get the better info and update the info
            it1->second = StateData.CompareGoalInfos(it1->second, it2->second);
            // TODO: Scuffed way of getting the better info's sender's relative position
            if (it1->second == it2->second) {
                StateData.NeighboursNavigationalInfo = {{"EstimateDistance", t_packet.Range},{"Angle", t_packet.HorizontalBearing.GetValue()}};
            } // otherwise, retain the current NeighboursNavigationalInfo
        
        }
        
    }

}

/**************************************************************************/
/**************************************************************************/

void CFootBotDiffusion::BroadcastNavigationalTable() {
    CByteArray cBuf = StateData.RealToByte(StateData.NavigationalTable);
    // TODO: Might be pointless in filling up the byte array
    // while (cBuf.Size() < StateData.SizeOfMessage) cBuf << '\0';
    m_pcRABA->SetData(cBuf);
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
    CVector2 cAccumulator = CVector2(StateData.GoalNavigationalInfo["EstimateDistance"], 
                                CRadians(StateData.GoalNavigationalInfo["Angle"]));
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

/**************************************************************************/
/**************************************************************************/

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
            fSpeed1 = fBaseAngularWheelSpeed;
            break;
        }
        case SWheelTurningParams::SOFT_TURN: {
            // Both wheels still go straight but one of the wheels is faster than the other
            Real fSpeedFactor = (WheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) /
                WheelTurningParams.HardTurnOnAngleThreshold;
            fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);   // Slower
            fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);   // Faster
            break;
        }
        case SWheelTurningParams::HARD_TURN: {
            // Both wheels have opposite speeds
            fSpeed1 = -WheelTurningParams.MaxSpeed;
            fSpeed2 = WheelTurningParams.MaxSpeed;
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

/**************************************************************************/
/**************************************************************************/

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

/**************************************************************************/
/**************************************************************************/

void CFootBotDiffusion::RandomExplore() {
    // Check if we found the designated goal info?
    if (StateData.FoundDesignatedGoal == true) {
        StartMoveToGoal();
        return;
    }
    // if robot spent too long in random explore, start aggressive epxlore
    if (StateData.TimeRandomExplore > StateData.MaximumTimeInRandomExploration) {
        StartAggressiveExploration();
        return;
    }
    else {
        // Get the diffusion vector to perform the obstacle avoidance
        bool collision;
        CVector2 diffusion = DiffusionVector(collision);
        // control the speed by MaxSpeed * 0.5
        SetWheelSpeedsFromVector(WheelTurningParams.MaxSpeed * diffusion);
        ++StateData.TimeRandomExplore;
    }
}

/**************************************************************************/
/**************************************************************************/

void CFootBotDiffusion::AggressiveExplore() {
    // Check if we found the designated goal info yet?
    if (StateData.FoundDesignatedGoal == true) {
        StartMoveToGoal();
        return;
    }
    else {
        // Get the diffusion vector to perform the obstacle avoidance
        bool collision;
        CVector2 diffusion = DiffusionVector(collision);
        // use MaxSpeed (twice the speed of Random Explore)
        SetWheelSpeedsFromVector(WheelTurningParams.MaxSpeed * diffusion);
    }
}

/**************************************************************************/
/**************************************************************************/

void CFootBotDiffusion::MoveToGoal() {
    // Check if we are close to the goal?
    if (StateData.ReachedDesignatedGoal == true) {
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
            SetWheelSpeedsFromVector(WheelTurningParams.MaxSpeed * (r * diffusion + (1.0 - r) * CalculateVectorToGoal()));
        }
        else {
            CRange<Real> range(0.0f, 0.2f);
            double r = m_RNG->Uniform(range);
            SetWheelSpeedsFromVector(WheelTurningParams.MaxSpeed * (r * diffusion + (1.0 - r) * CalculateVectorToGoal()));
        }
    }
}

/**************************************************************************/
/**************************************************************************/

// TODO: #27 Pointless Rest() but might add some implementation later on
void CFootBotDiffusion::Rest() {
    if (StateData.ReachedDesignatedGoal == false) {
        StartRandomExploration();
    }
    else {

        LOG << Id << ": Done tasks!\n The goal was: " << StateData.GoalId << std::endl;
    }

}

/**************************************************************************/
/**************************************************************************/

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