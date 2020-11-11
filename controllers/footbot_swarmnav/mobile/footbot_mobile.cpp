#include "footbot_mobile.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* 3D vector definition */
#include <argos3/core/utility/math/vector3.h>
/* Quarternion definition for calculating orientation */
#include <argos3/core/utility/math/quaternion.h>
/* Angles definition */
#include <argos3/core/utility/math/angles.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
CFootBotMobile::SDiffusionParams::SDiffusionParams() :
    GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void CFootBotMobile::SDiffusionParams::Init(TConfigurationNode& t_node) {
    CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-10.0f), CDegrees(10.0f));
    GetNodeAttribute(t_node, "go_straight_angle_range", cGoStraightAngleRangeDegrees);
    // Set the threshold range for determining if the robot should proceed straight forward
    // based on relative distance between the robot and the closest obstacle detected
    GoStraightAngleRange.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                            ToRadians(cGoStraightAngleRangeDegrees.GetMax()));
    // parse it as delta in XML file
    GetNodeAttribute(t_node, "delta", Delta);   
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CFootBotMobile::SWheelTurningParams::Init(TConfigurationNode& t_node) {
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
CFootBotMobile::SStateData::SStateData() :
    ProbRange(CRadians(-1.0f), CRadians(1.0f)) {}

void CFootBotMobile::SStateData::Init(TConfigurationNode& t_node) {
    GetNodeAttribute(t_node, "act_as_target", TargetRobot);
    GetNodeAttribute(t_node, "goal_id_representative", GoalIdRepresentative);
}

void CFootBotMobile::SStateData::Reset() {
    if (TargetRobot) {
        State = RESTING;
        FoundDesignatedGoal = true;
        ReachedGoal = true;
        TimeRandomExplore = 0;
    }
    else {
        State = RESTING;
        TimeRandomExplore = 0;
        FoundDesignatedGoal = false;
        ReachedGoal = false;
    }
}

void CFootBotMobile::SStateData::SaveState() {
    PreviousState = State;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
CFootBotMobile::SNavigationData::SNavigationData() :
    NumberOfGoals(1) {}

void CFootBotMobile::SNavigationData::Init(TConfigurationNode& t_node) {
    GetNodeAttribute(t_node, "size_of_message", SizeOfMessage);
    GetNodeAttribute(t_node, "number_of_goals", NumberOfGoals);
}

void CFootBotMobile::SNavigationData::Reset(UInt8 local_id) {
    // Assigned goal id based on a formulae
    GoalId = (local_id % NumberOfGoals) - 1;
    // TODO: #29 Temporary fix for unsigned modulo issue
    if (GoalId == 255) GoalId = NumberOfGoals - 1;
    DistanceTravelled = 0;
    TotalRotation =  CRadians(0);
    
    for (size_t i = 0; i < NumberOfGoals; ++i) {
        NavigationTable[i] = {{"Age", 0}, {"Angle", 0}, {"Distance", 0}};
    }
    LOG << std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::map<UInt8, std::map<std::string, Real>> CFootBotMobile::SNavigationData::ConvertFromByte(CByteArray& b_array) {
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

CByteArray CFootBotMobile::SNavigationData::ConvertToByte(std::map<UInt8, std::map<std::string, Real>>& m_info) {
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
CFootBotMobile::CFootBotMobile() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL),
    m_pcLEDs(NULL),
    m_pcPosS(NULL),
    m_pcEncoder(NULL),
    m_RNG(NULL) {}

UInt8 CFootBotMobile::s_unMobileCounter = 0; 

void CFootBotMobile::Init(TConfigurationNode& t_node) {
    // Set robotId based on the s_unIdCounter(static variable)
    Id = s_unMobileCounter++;
    // Init actuators/ sensors handlers with the string as label to be used inside the XML file
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
    m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
    m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
    m_pcPosS = GetSensor<CCI_PositioningSensor>("positioning");
    m_pcEncoder = GetSensor<CCI_DifferentialSteeringSensor>("differential_steering");
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CFootBotMobile::Reset() {
    // Reset robot state data
    StateData.Reset();
    // Reset Navigation data
    NavData.Reset(Id);
    // Empty the buffer
    m_pcRABA->ClearData();
    // Set LED Color
    if (StateData.TargetRobot) {
        m_pcLEDs->SetAllColors(CColor::RED);
        m_pcWheels->SetLinearVelocity(0,0);  
    }
    else {
        m_pcLEDs->SetAllColors(CColor::BLACK);      
    }
    
    // Debug = Pass
    LOG << "Mobile Robot " << Id << " successfully reseted! " << std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * The control step is called every time step and contains all the actions
 * that will be executed by the robot itself.
 *
 */

void CFootBotMobile::ControlStep() {
    StateData.TimeStep++;
    if (StateData.TargetRobot) {
        // These are specifically for target robots, otherwise ignore these and execute normally
        CByteArray cBuf;
        cBuf << NavData.GoalId;
        cBuf << '\0';
        // Everytime target robot broadcast (every time step), increase age
        cBuf << StateData.TimeStep;
        while (cBuf.Size() < NavData.SizeOfMessage)
            cBuf << '\0';
        m_pcRABA->SetData(cBuf);
    }
    else {
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
            case MOVE_TO_GOAL: {
                MoveToGoal();
                break;
            }
            default: {
                LOGERR << "Undefined State" << std::endl;
            }
        }

        BroadcastNavigationTable();
    }
    

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CFootBotMobile::UpdateOdometry() {
    // Get the readings from wheel encoders and update DistanceTravlled + Readings from positioning sensor
    const CCI_DifferentialSteeringSensor::SReading& sEncoderData = m_pcEncoder->GetReading();
    const CCI_PositioningSensor::SReading& sPosData = m_pcPosS->GetReading();
    CRadians cZAngle, cYAngle, cXAngle;
    sPosData.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
    NavData.DistanceTravelled = (sEncoderData.CoveredDistanceLeftWheel + sEncoderData.CoveredDistanceRightWheel)/sEncoderData.WheelAxisLength;
    NavData.PreviousOrientation = NavData.CurrentOrientation;
    NavData.CurrentOrientation = cZAngle;
    // Get the total rotation occur during the previous time step by getting the differences betweem
    // CurrentOrientation and PreviousOrientation
    NavData.TotalRotation = NormalizedDifference(NavData.CurrentOrientation, NavData.PreviousOrientation);
    
    // A fail-safe to ensure that invalid info will be remove since it will impede the 
    // robot's movement
    if (WheelTurningParams.PreviousTurningMechanisms.size() == 5) {
        // Remove duplicates
        WheelTurningParams.PreviousTurningMechanisms.unique();
        if (WheelTurningParams.PreviousTurningMechanisms.size() == 2 &&
            WheelTurningParams.PreviousTurningMechanisms.back() != WheelTurningParams.NO_TURN) {
            // If left a HARD_TURN or SOFT_TURN, it meant that the robot had been 
            // doing those turn for 15 time steps which only happens in a loop
            if (StateData.State == MOVE_TO_GOAL or StateData.State == RANDOM_EXPLORATION) {
                // Reset robot
                Reset();
            }
        }
        return;
    }

    // Update navigation info for every row in any state unless no info (Age == 0)
    for (auto & goal : NavData.NavigationTable) {
        if (goal.second["Age"] == 0)  continue; // skip if no info
        // Otherwise, update info based on odometry
        CVector2 NewVector, PreviousVector;
        CVector2 CurrentVector = CVector2(goal.second["Distance"], CRadians(goal.second["Angle"]));
        // Update the info based on previous turning states
        SWheelTurningParams::ETurningMechanism oldTurningMechanism = WheelTurningParams.TurningMechanism;
        /*
         * If no turn, it means that no rotation involved. Hence simple subtraction with distance
         * If soft turn, it means that rotation is involved but minor with minor distance 
         * If hard turn, it means that rotation is involved but no distance covered
         * 
         */
        if (oldTurningMechanism == SWheelTurningParams::NO_TURN) {
            PreviousVector = CVector2(NavData.DistanceTravelled, CRadians(0));
            NewVector = CurrentVector - PreviousVector;
            // TODO: May not work as intended; If that's the case, subtract the distance or add directly
        }
        else if (oldTurningMechanism == SWheelTurningParams::SOFT_TURN) {
            PreviousVector = CVector2(NavData.DistanceTravelled, NavData.TotalRotation);
            NewVector = CVector2(CurrentVector.Length() - PreviousVector.Length(), 
                NormalizedDifference(CurrentVector.Angle(), PreviousVector.Angle()));
        }
        else if (oldTurningMechanism == SWheelTurningParams::HARD_TURN) {
            PreviousVector = CVector2(1.0f, NavData.TotalRotation);
            CRadians NewAngle = NormalizedDifference(CurrentVector.Angle(), PreviousVector.Angle());
            CRange<CDegrees> cGoStraightAngleRangeDegrees(CDegrees(-30), CDegrees(30));
            CRange<CRadians> cGoStraightAngleRangeRadians;
            cGoStraightAngleRangeRadians.Set(ToRadians(cGoStraightAngleRangeDegrees.GetMin()),
                                                ToRadians(cGoStraightAngleRangeDegrees.GetMax()));

            if (cGoStraightAngleRangeRadians.WithinMinBoundIncludedMaxBoundIncluded(NewAngle)) {
                NewVector = CVector2(CurrentVector.Length(), CRadians(0));
            }
            else {
                NewVector = CVector2(CurrentVector.Length(), NormalizedDifference(CurrentVector.Angle(), PreviousVector.Angle()));
            }
        }
        
        goal.second["Angle"] = NewVector.Angle().GetValue();
        goal.second["Distance"] = NewVector.Length();
    } 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CFootBotMobile::UpdateNavigationTable(const CCI_RangeAndBearingSensor::TReadings& t_packets) {
    // Responsible for only comparing info with external info and update the info with the best one
    // Flags and state changes according to table is done on UpdateNavigationBehaviour()
    std::map<UInt8, bool> ReceivedFromTarget;
    for (UInt8 i = 0; i < NavData.NumberOfGoals; ++i) {
        ReceivedFromTarget[i] = false;
    }
    for (CCI_RangeAndBearingSensor::SPacket t_packet : t_packets) {
        // Put the packet's data into a byte array 
        CByteArray cBuf = t_packet.Data;
        UInt8 un_Identifier;
        // Extract the first byte from the packet
        *cBuf(0,1) >> un_Identifier;
        // Identifier == 99 (mobile) > read the rest else if Identifier != 99 (target) > ignore the rest
        if (un_Identifier != 99) {
            // Skips comparison and replace the info directly since it comes from the goal robot itself
            UInt64 un_Age;
            *cBuf(5, 13) >> un_Age;
            if (ReceivedFromTarget[un_Identifier] == false) {
                NavData.NavigationTable[un_Identifier]["Age"] = un_Age;
                NavData.NavigationTable[un_Identifier]["Angle"] = t_packet.HorizontalBearing.GetValue();
                NavData.NavigationTable[un_Identifier]["Distance"] = t_packet.Range;
                ReceivedFromTarget[un_Identifier] = true;
            }
            else {
                if (NavData.NavigationTable[un_Identifier]["Distance"] > t_packet.Range) {
                    NavData.NavigationTable[un_Identifier]["Age"] = un_Age;
                    NavData.NavigationTable[un_Identifier]["Angle"] = t_packet.HorizontalBearing.GetValue();
                    NavData.NavigationTable[un_Identifier]["Distance"] = t_packet.Range;
                }
            }
            

            // Check to determine whether the robot received any message from target every 5 timestep
            if (StateData.TimeStep % 5 == 0 and un_Identifier == NavData.GoalId)
                NavData.NearTarget = true;
        } 
        else {
            std::map<UInt8, std::map<std::string, Real>> OtherTable = NavData.ConvertFromByte(cBuf);
            // Iterates both tables simultaneously
            for (auto local_table = NavData.NavigationTable.begin(), other_table = OtherTable.begin();
                local_table != NavData.NavigationTable.end(); ++local_table, ++other_table) {
                if (other_table->second["Age"] == 0) continue; // Skip comparison and move to next row
                // Compute the relative vector between local robot and target by adding other's info
                // and sender's relative vector from local robot
                // CVector2 OtherInfo = CVector2(other_table->second["Distance"], CRadians(other_table->second["Angle"]));
                // // Addition of Other's To Target + Local to Other
                // CVector2 LocalToOther = CVector2(t_packet.Range, t_packet.HorizontalBearing);
                // CVector2 LocalToTarget = LocalToOther + OtherInfo;

                // Just compute the total distance from local robot to target
                Real TotalDistance = other_table->second["Distance"] + local_table->second["Distance"];


                // Compare based on age and distance for now
                // Use neighbour's info if there is no local info
                if (local_table->second["Age"] == 0 && ReceivedFromTarget[local_table->first] == false) {
                    local_table->second["Age"] = other_table->second["Age"];
                    local_table->second["Angle"] = t_packet.HorizontalBearing.GetValue();
                    local_table->second["Distance"] = TotalDistance;
                }

                // Age holds a lot of weightage since outdated info comes hand in hand with accumulated error due to 
                // odometry
                if (other_table->second["Age"] > local_table->second["Age"] && TotalDistance < local_table->second["Distance"]
                    && ReceivedFromTarget[local_table->first] == false) {
                    // Replaces the local info with neighbour's info instead
                    local_table->second["Age"] = other_table->second["Age"];
                    local_table->second["Angle"] = t_packet.HorizontalBearing.GetValue();
                    local_table->second["Distance"] = TotalDistance;
                } // Othewise, retain local info

            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CFootBotMobile::UpdateNavigationBehaviour() {
    // Access the goal info directly based on goal id
    if (NavData.NavigationTable[NavData.GoalId]["Age"] != 0) {
        // Got goal info
        StateData.FoundDesignatedGoal = true;

        // Every 15 timestep, check whether robot received any message from target/ goal to determine
        // whether the robot truly reached the goal since the estimate distance may not be reliable
        if (NavData.NearTarget) {
            if (NavData.NavigationTable[NavData.GoalId]["Distance"] < 40.0f) {
                StateData.ReachedGoal = true;
            }
        }
    
        
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CFootBotMobile::BroadcastNavigationTable() {
    CByteArray cBuf;
    // If reached goal and resting state
    // The robot will act as a beacon similar to original target/ goal
    if (StateData.ReachedGoal && StateData.State == RESTING) {
        UInt64 Age = StateData.TimeStep;
        cBuf << NavData.GoalId;
        cBuf << '\0';
        cBuf << Age;
        while (cBuf.Size() < NavData.SizeOfMessage) cBuf << '\0';        
    }
    else {
        // Broadcast local table
        cBuf = NavData.ConvertToByte(NavData.NavigationTable);
    }
    m_pcRABA->SetData(cBuf);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CFootBotMobile::UpdateState() {
    // One of the important methods since this encompases the action of robot each time step
    const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();

    // Reset flag NearTarget every 5 timestep
    NavData.NearTarget = false;

    // So long it's not resting, update local info with odometry
    if (StateData.State != RESTING) 
        UpdateOdometry();
    // Else, don't need to update the state at all since RESTING is the end state
    else 
        return;
    // Always update navigation table unless RESTING
    UpdateNavigationTable(tPackets);

    // Update navigation behaviour based on the updated local table
    UpdateNavigationBehaviour();

    // Check flags and update states accordingly
    StateData.SaveState();
    if (StateData.FoundDesignatedGoal)
        StateData.State = MOVE_TO_GOAL;
    else if (StateData.ReachedGoal) {
        // Only save the timestep for the first time that it reached the goal
        
        StateData.State = RESTING;
    }
        
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
CVector2 CFootBotMobile::VectorToGoal() {
    CVector2 cAccumulator;
    // Calculate vector towards target based on local info
    cAccumulator = CVector2(NavData.NavigationTable[NavData.GoalId]["Distance"], 
        CRadians(NavData.NavigationTable[NavData.GoalId]["Angle"]));
    // If the range > 0.0f, then return the vector with length == 1.0f
    if (cAccumulator.Length() > 0.0f) {
        cAccumulator.Normalize();
        return cAccumulator;
    }
    else {
        return CVector2();
    }

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
CVector2 CFootBotMobile::DiffusionVector(bool& collision) {
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
        // deviate further if stuck in random explore for too long by turning into any direction at every 50 timestep
        return CVector2::X;

        // // Random chance of turning 
        // if (StateData.State == RANDOM_EXPLORATION && (WheelTurningParams.PreviousTurningMechanisms.front() == WheelTurningParams.NO_TURN
        //     && WheelTurningParams.PreviousTurningMechanisms.back() == WheelTurningParams.NO_TURN)) {
        //     return CVector2(10.0f, m_RNG->Uniform(CRadians::SIGNED_RANGE));
        // }
        // else {
        //     return CVector2::X; 
        // }
        
    }
    else {
        collision = true;
        diffusionVector.Normalize();
        return -diffusionVector;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CFootBotMobile::SetWheelSpeedsFromVector(const CVector2& heading) {
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


    // Assign current turning mechanism into the list
    if (WheelTurningParams.PreviousTurningMechanisms.size() == 5) {
        // Pop front to mantain the list of size 6 before adding new one at back
        WheelTurningParams.PreviousTurningMechanisms.pop_front();
    } // Otherwise, append at the back
    WheelTurningParams.PreviousTurningMechanisms.push_back(WheelTurningParams.TurningMechanism);

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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CFootBotMobile::StartRandomExploration() {
    if (StateData.State == RANDOM_EXPLORATION) {
        LOGERR << "Already in RANDOM_EXPLORATION!" << std::endl;
    }
    StateData.SaveState();
    StateData.State = RANDOM_EXPLORATION;
    TraceMessages.push_back(new CRandomExplorationTrace(Id));
}

void CFootBotMobile::StartMoveToGoal() {
    if (StateData.State == MOVE_TO_GOAL) {
        LOGERR << "Already in MOVE_TO_GOAL!" << std::endl;
    }
    StateData.SaveState();
    StateData.State = MOVE_TO_GOAL;
    TraceMessages.push_back(new CMoveToGoalTrace(Id));
}

void CFootBotMobile::StartResting() {
    if (StateData.State == RESTING) {
        LOGERR << "Already in RESTING!" << std::endl;
    }
    // Debug = Pass
    LOG << "Mobile Robot " << Id << ": Done tasks!\n The goal was: " << NavData.GoalId << " with navigational delay of "
        << StateData.TimeTakenToReachGoal << std::endl;
    StateData.SaveState();
    StateData.State = RESTING;
    TraceMessages.push_back(new CRestingTrace(Id));
    StateData.TimeRested = NavData.NavigationTable[NavData.GoalId]["Age"];
    m_pcLEDs->SetAllColors(CColor::RED);
    m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void CFootBotMobile::RandomExplore() {
    // Check if we found the designated goal info?
    if (StateData.FoundDesignatedGoal) {
        StartMoveToGoal();
        return;
    }
    else if (StateData.ReachedGoal) {
        StartResting();
        return;
    }
    else {
        bool collision;
        CVector2 diffusion = DiffusionVector(collision);
        // As time in RANDOM_EXPLORATION increases, the mobile robot will start moving towards a random direction instead of diffusion
        if (StateData.TimeStep > 145 and StateData.TimeStep % 100 == 0 and !collision) {
            CRange<CRadians> RandomRange(CRadians(-1.0472), CRadians(1.0472));
            CRadians NewAngle = m_RNG->Uniform(RandomRange);
            diffusion = 0.8f * diffusion + CVector2(1.0f, NewAngle) * 0.2f;
        }
      
        SetWheelSpeedsFromVector(WheelTurningParams.MaxSpeed * diffusion);
    }
}

void CFootBotMobile::MoveToGoal() {
    // Check if we are close to the goal?
    if (StateData.ReachedGoal) {
        StateData.TimeTakenToReachGoal = StateData.TimeStep;
        StartResting();
        return;
    }
    else {
        bool collision;
        CVector2 diffusion = DiffusionVector(collision);
        if (collision) {
            CRange<Real> range(0.2f, 0.7f);
            double r = m_RNG->Uniform(range);
            CVector2 NewVector = r * diffusion + (1.0 - r) * VectorToGoal();
            SetWheelSpeedsFromVector(WheelTurningParams.MaxSpeed * NewVector);
        }
        else {
            CRange<Real> range(0.0f, 0.2f);
            double r = m_RNG->Uniform(range);
            CVector2 NewVector = r * diffusion + (1.0 - r) * VectorToGoal();
            SetWheelSpeedsFromVector(WheelTurningParams.MaxSpeed * NewVector);
        }
    }
}

void CFootBotMobile::Rest() {
    if (StateData.TargetRobot) {
        NavData.GoalId = StateData.GoalIdRepresentative;
        m_pcWheels->SetLinearVelocity(0,0);
        return;
    }
    if (!StateData.ReachedGoal)
        StartRandomExploration();
    else
        m_pcWheels->SetLinearVelocity(0,0);
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

REGISTER_CONTROLLER(CFootBotMobile, "footbot_mobile_controller")