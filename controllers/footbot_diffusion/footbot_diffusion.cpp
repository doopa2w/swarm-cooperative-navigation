/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/angles.h>
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
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void CFootBotDiffusion::SNavigationData::Init(TConfigurationNode& t_node) {
   GetNodeAttribute(t_node, "sender_distance", SDistance);
   GetNodeAttribute(t_node, "sender_angle", SAngle);
   GetNodeAttribute(t_node, "goal_distance", GDistance);
   GetNodeAttribute(t_node, "goal_angle", GAngle);
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

CFootBotDiffusion::CFootBotDiffusion() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL),
    m_pcEncoder(NULL),
    m_pcPosition(NULL),
    m_RNG(NULL) {}



/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {

    // Init actuators/ sensors handlers with the string as label to be used inside the XML file
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
    m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
    m_pcEncoder = GetSensor<CCI_DifferentialSteeringSensor>("differential_steering");
    m_pcPosition = GetSensor<CCI_PositioningSensor>("positioning");
    m_RNG = CRandom::CreateRNG("argos"); 
    /*
    * Parses the configuration file
    */
    DiffusionParams.Init(GetNode(t_node, "diffusion"));
    WheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
    NavigationParams.Init(GetNode(t_node, "navigation" ));

    NavigationParams.CurrentRotation = CDegrees(0);
    NavigationParams.PreviousRotation = CDegrees(0);


}
/****************************************/
/****************************************/

void CFootBotDiffusion::ControlStep() {

    // LOg to know distance travlled with respect to static robot
    const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
    for (const CCI_RangeAndBearingSensor::SPacket sPackets : tPackets) {
        LOG << "Relative distance " << sPackets.Range << " and angle " << sPackets.HorizontalBearing << std::endl;
    }

    CByteArray cBuf;
    while (cBuf.Size() < 10) cBuf << '\0';
    m_pcRABA->SetData(cBuf);

    const CCI_DifferentialSteeringSensor::SReading& sEncodes = m_pcEncoder->GetReading();
    const CCI_PositioningSensor::SReading& sPosition = m_pcPosition->GetReading();
    
    LOG << "\nMobile Robot's Covered Left Distace: " << sEncodes.CoveredDistanceLeftWheel << " and Covered Right Distance "
        << sEncodes.CoveredDistanceRightWheel << " with Wheel axis lenght of " << sEncodes.WheelAxisLength << std::endl;

    LOG << "Mobile Robot has travalled " << (sEncodes.CoveredDistanceLeftWheel + sEncodes.CoveredDistanceRightWheel)/ sEncodes.WheelAxisLength << std::endl;
    
    LOG << "Mobile Robot's Position is " << sPosition.Position << " with Orientation of " << sPosition.Orientation << std::endl;


//     CRadians cZAngle, cYAngle, cXAngle;
// 362  c_quaternion.ToEulerAngles(cZAngle, cYAngle, cXAngle);
// 363  c_os << ToDegrees(cZAngle).GetValue() << ","
// 364  << ToDegrees(cYAngle).GetValue() << ","
// 365  << ToDegrees(cXAngle).GetValue();

    // Get Previous Orientation and Current Orientation;
    LOG << "Current W is " << sPosition.Orientation.GetW() << std::endl;
    
    CRadians cZAngle, cYAngle, cXAngle; // we will only be using Z angle
    sPosition.Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
    NavigationParams.CurrentRotation = CDegrees(ToDegrees(cZAngle).GetValue()); // converting to degrees
    NavigationParams.CurrentRotation.UnsignedNormalize(); // convert to 0:360 
    NavigationParams.TotalRotation +=(NavigationParams.CurrentRotation - NavigationParams.PreviousRotation);
    LOG << "Mobile robot's previous rotation is " << NavigationParams.PreviousRotation << " and Current Rotation is "
        << NavigationParams.CurrentRotation << std::endl;
    LOG << "Mobile Robot's total rotation from origin is " << NavigationParams.TotalRotation << std::endl;

    CRange<CDegrees> cDesiredRange(CDegrees(-10.0f), CDegrees(100.0f));

    // For first time step
    m_pcWheels->SetLinearVelocity(4.0f, 10.0f); 

    if (NavigationParams.TotalRotation>=(CDegrees(80)) && NavigationParams.TotalRotation<=(CDegrees(100))) {
        m_pcWheels->SetLinearVelocity(10.0f, 10.0f);
    }


    // Testing normalized difference
    CDegrees GoLeftRotation = CDegrees(60);
    CDegrees GoRightRotation = CDegrees(-60);
    CDegrees InitialLeftTarget = CDegrees(150);
    CDegrees InitialRightTarget = CDegrees(-150);

    LOG << "From Left Target: " << InitialLeftTarget << " then rotate " << GoLeftRotation << " which results in "
        << NormalizedDifference(InitialLeftTarget, GoLeftRotation);
    LOG << "From Left Target: " << InitialLeftTarget << " then rotate " << GoRightRotation << " which results in "
        << NormalizedDifference(InitialLeftTarget, GoRightRotation);
    LOG << "From Right Target: " << InitialRightTarget << " then rotate " << GoLeftRotation << " which results in "
        << NormalizedDifference(InitialRightTarget, GoLeftRotation);
    LOG << "From Right Target: " << InitialRightTarget << " then rotate " << GoRightRotation << " which results in "
        << NormalizedDifference(InitialRightTarget, GoRightRotation);



    
    // if (cDesiredRange.WithinMinBoundIncludedMaxBoundIncluded(NavigationParams.TotalRotation)) {
    //     LOG << "It is within the range of +-10! Hence, move forward!" << std::endl;
    //     // Stop rotating and move forward
    //     // Make sure to reset the total rotation
    //     m_pcWheels->SetLinearVelocity(10.0f, 10.0f);
    //     NavigationParams.TotalRotation = CDegrees(0);
    // }
    // else {
    //     LOG << "Continue rotating to the desired heading angle!" << std::endl;
    //     m_pcWheels->SetLinearVelocity(4.0f, 10.0f); 
    // }

    NavigationParams.PreviousRotation = NavigationParams.CurrentRotation;



    


    

//     LOG << "Mobile Robot Previous Vector was " << NavigationParams.PreviousVector << std::endl;

//     // Get Previous turning state
//     SWheelTurningParams::ETurningMechanism PreviousTurningMechanism = WheelTurningParams.TurningMechanism;

//    // Update odometry
//    if (State == SENDER) {
//     //   CVector2 VectorToSender = CVector2(NavigationParams.SDistance, CRadians(NavigationParams.SAngle));

//       if (PreviousTurningMechanism == WheelTurningParams.HARD_TURN || PreviousTurningMechanism == WheelTurningParams.SOFT_TURN) {
//           // Subtract angle to compensate rotation
//           NavigationParams.SAngle-=(sPosition.Orientation.GetW());
//       }
//       else {
//           // Subtract distance instead
//           NavigationParams.SDistance-=(NavigationParams.PreviousVector.Length());
//       }
      
//       LOG << "Mobile robot updated the sender info to Distance: " << NavigationParams.SDistance << " and Angle: "
//         << NavigationParams.SAngle << std::endl;
//    }
//    else if (State == GOAL) {
//     //   CVector2 VectorToGoal = CVector2(NavigationParams.GDistance, CRadians(NavigationParams.GAngle));
//     //   CVector2 NewVectorToGoal = VectorToGoal - NavigationParams.PreviousVector;
//     //   NavigationParams.GDistance  = NewVectorToGoal.Length();
//     //   NavigationParams.GAngle = NewVectorToGoal.Angle().GetValue();
//     //   LOG << "Mobile robot updated the goal info to " << NewVectorToGoal << std::endl;
//     if (PreviousTurningMechanism == WheelTurningParams.HARD_TURN || PreviousTurningMechanism == WheelTurningParams.SOFT_TURN) {
//         NavigationParams.GAngle-=(sPosition.Orientation.GetW());
//     }
//     else {
//         NavigationParams.GDistance-=(NavigationParams.PreviousVector.Length());
//     }
//     LOG << "Mobile robot updated the goal info to Distance: " << NavigationParams.GDistance << " and Angle: "
//         << NavigationParams.GAngle << std::endl; 
//    }

   
//    bool collision;
//    CVector2 diffusion = DiffusionVector(collision);
//    CVector2 NewVector;
//    // Move towards sender first
//    if (NavigationParams.SDistance > 10.0f) {
//       State = SENDER;
//       if (collision) {
//          CRange<Real> range(0.2f, 0.7f);
//          double r = m_RNG->Uniform(range);
//          NewVector = r * diffusion + (1.0 - r) * VectorToGoal(false) * WheelTurningParams.MaxSpeed;
//          SetWheelSpeedsFromVector(NewVector);
//          NavigationParams.PreviousVector = NewVector;
//       }
//       else {
//          CRange<Real> range(0.0f, 0.2f);
//          double r = m_RNG->Uniform(range);
//          NewVector = r * diffusion + (1.0 - r) * VectorToGoal(false) * WheelTurningParams.MaxSpeed;
//          SetWheelSpeedsFromVector(NewVector);
//          NavigationParams.PreviousVector = NewVector;
//       }
//       // Broadcast the calculate vector
//       LOG << "Calculated the new vector " << NewVector << std::endl;
//    }
//    // Move towards goal
//    else {
//       if (NavigationParams.GDistance > 10.0f) {
//          State = GOAL;
//          if (collision) {
//             CRange<Real> range(0.2f, 0.7f);
//             double r = m_RNG->Uniform(range);
//             NewVector = r * diffusion + (1.0 - r) * VectorToGoal(true) * WheelTurningParams.MaxSpeed;
//             SetWheelSpeedsFromVector(NewVector);
//             NavigationParams.PreviousVector = NewVector;
//          }
//          else {
//             CRange<Real> range(0.0f, 0.2f);
//             double r = m_RNG->Uniform(range);
//             NewVector = r * diffusion + (1.0 - r) * VectorToGoal(true) * WheelTurningParams.MaxSpeed;
//             SetWheelSpeedsFromVector(NewVector);
//             NavigationParams.PreviousVector = NewVector;
//          }

//          // Broadcast the calculate vector
//          LOG << "Calculated the new vector " << NewVector << std::endl;
//       }
//       else {
//          State = REST;
//          m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
//          LOG << "Mobile Robot done task!";
//       }
      
//    }

   

}




/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

CVector2 CFootBotDiffusion::VectorToGoal(bool goalorsender) {
    CVector2 cAccumulator;
    
    if (goalorsender)
        cAccumulator = CVector2(NavigationParams.GDistance, CRadians(NavigationParams.GAngle));
    else
        cAccumulator = CVector2(NavigationParams.SDistance, CRadians(NavigationParams.SAngle));
    
    // Debug
    LOG << "Mobile Robot Calculated the vector in VectorToGoal: " << cAccumulator.Length() << ", "
        << cAccumulator.Angle() << std::endl;

    // if the range is > 0.0f then return the vector
    if (DiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAccumulator.Angle())) {
        cAccumulator.Normalize();
        return cAccumulator;
    }
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
            LOG << "Mobile Robot employing NO_TURN! "<< std::endl;

            break;
        }
        case SWheelTurningParams::SOFT_TURN: {
            // Both wheels still go straight but one of the wheels is faster than the other
            Real fSpeedFactor = (WheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) /
                WheelTurningParams.HardTurnOnAngleThreshold;
            fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);   // Slower
            fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);   // Faster

            // Debug = pass
            LOG << "Mobile Robot employing SOFT_TURN! "<< std::endl;

            break;
        }
        case SWheelTurningParams::HARD_TURN: {
            // Both wheels have opposite speeds
            fSpeed1 = -WheelTurningParams.MaxSpeed;
            fSpeed2 = WheelTurningParams.MaxSpeed;

            // Debug = pass
            LOG << "Mobile Robot employing HARD_TURN! "<< std::endl;

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


/****************************************/
/****************************************/

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
