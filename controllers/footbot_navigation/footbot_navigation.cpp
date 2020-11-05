#include "footbot_navigation.h"
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/logging/argos_log.h>


CFootBotNavigation::SDiffusionParams::SDiffusionParams() :
    GoStraightAngleRange(CRadians(-1.0f), CRadians(1.0f)) {}

void CFootBotNavigation::SDiffusionParams::Init(TConfigurationNode& t_node) {
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

void CFootBotNavigation::SWheelTurningParams::Init(TConfigurationNode& t_node) {
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


void CFootBotNavigation::SNavigationData::Reset() {
    GoalNavigationalInfo = {
        {"Angle", 0}, {"EstimateDistance", 289.23}, {"SequenceNumber", 0} 
    };
    FoundDesignatedGoal = false;
    ReachedDesignatedGoal = false;
}

CFootBotNavigation::CFootBotNavigation() :
    m_pcWheels(NULL),
    m_pcProximity(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL),
    m_pcLEDs(NULL),
    m_RNG(NULL) {}  


void CFootBotNavigation::Init(TConfigurationNode& t_node) {
    // Init actuators/ sensors handlers with the string as label to be used inside the XML file
    m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
    m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
    m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
    m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
    m_RNG = CRandom::CreateRNG("argos");


    DiffusionParams.Init(GetNode(t_node, "diffusion"));
    WheelTurningParams.Init(GetNode(t_node, "wheel_turning"));

    Reset();
}


void CFootBotNavigation::ControlStep() {
    // if received message, taket the sender's relative position 
    // and attempt to move towards it while also updating the odometry info
    LOG << "At time step, Navigational Info " << NavData.GoalNavigationalInfo["Angle"] <<
        " and " << NavData.GoalNavigationalInfo["EstimateDistance"] << "cm!" << std::endl;
    if (NavData.GoalNavigationalInfo["EstimateDistance"] < 30.0f) {
            NavData.ReachedDesignatedGoal = true;
        }
    if (NavData.ReachedDesignatedGoal == true) {
        m_pcWheels->SetLinearVelocity(0.0f, 0.0f);
        LOG << "Mobile Robot done with tasks! Final info: " << NavData.GoalNavigationalInfo["Angle"] << ", "
            << NavData.GoalNavigationalInfo["EstimateDistance"] << "cm!" << std::endl;
    }
    else {
        bool collision;
        CVector2 diffusion = DiffusionVector(collision);
        if (collision) {
            CRange<Real> range(0.2f, 0.7f);
            double r = m_RNG->Uniform(range);
            LOG << "The r is " << r << std::endl;
            CVector2 NewVector = r * diffusion + (1.0 - r) * CalculateVectorToGoal();
            SetWheelSpeedsFromVector(WheelTurningParams.MaxSpeed * NewVector);

            LOG << "New vector after calculation " << NewVector << std::endl;
            // Upadte odometry info inside the table
            UpdateOdometry(NewVector);
        }
        else {
            CRange<Real> range(0.0f, 0.2f);
            double r = m_RNG->Uniform(range);
            LOG << "The r is " << r << std::endl;
            CVector2 NewVector = r * diffusion + (1.0 - r) * CalculateVectorToGoal();
            SetWheelSpeedsFromVector(WheelTurningParams.MaxSpeed * NewVector);

            LOG << "New vector after calculation " << NewVector << std::endl;
            // Upadte odometry info inside the table
            UpdateOdometry(NewVector);
        }
        LOG << "Mobile Robot New Navigational Info " << NavData.GoalNavigationalInfo["Angle"] << ", " <<
            NavData.GoalNavigationalInfo["EstimateDistance"] << "cm! " << std::endl;
    }

}

void CFootBotNavigation::Reset() {
    NavData.Reset();
    m_pcRABA->ClearData();
    m_pcLEDs->SetAllColors(CColor::BLACK);

}

void CFootBotNavigation::UpdateOdometry(const CVector2& heading) {

    LOG << "The heading is " << heading.Angle().GetValue() << " length is " <<
        heading.Length() << "cm! " << std::endl;

    if (NavData.GoalNavigationalInfo["Angle"] == 0 && NavData.GoalNavigationalInfo["EstimateDistance"] == 0) {
        return;
    }
    else {
        NavData.GoalNavigationalInfo["Angle"] = NavData.GoalNavigationalInfo["Angle"] - heading.Angle().GetValue();
        NavData.GoalNavigationalInfo["EstimateDistance"] = NavData.GoalNavigationalInfo["EstimateDistance"] - heading.Length();
    }
    

}

CVector2 CFootBotNavigation::CalculateVectorToGoal() {
    // will only be used if in MOVE_TO_GOAL state
    // get the goal info
    // TODO: #22 Experimental feature
    // variable name might not reflect the value or how it was obtained
    // 
    CVector2 cAccumulator = CVector2(NavData.GoalNavigationalInfo["EstimateDistance"], 
                                CRadians(NavData.GoalNavigationalInfo["Angle"]));



    LOG << "Inside CalculateVectorToGoal, Mobile Robot " << " got the vector: " << cAccumulator.Length() << ", "
        << cAccumulator.Angle() << std::endl;
    // if the range is > 0.0f then return the vector
    if (cAccumulator.Length() > 0.0f) {
        cAccumulator.Normalize();
        LOG << "Inside CalculateVectorToGoal, New vector after normalization " << cAccumulator.Length() << "cm; angle of " <<
            cAccumulator.Angle() << std::endl;
        return cAccumulator;

    }
    // Otherwise, returns zero
    // TODO: Might be redundant since less than 20 we would have ReachedGoal = true and State = RESTING
    else {
        LOGERR << "Checkers for less than 20cm did not work as intended!" << std::endl;
        return CVector2();
    }
}

// TODO: #23 Need to understand this clearly too
CVector2 CFootBotNavigation::DiffusionVector(bool& collision) {
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
    LOG << "Within diffusion vector with minus, Collision is " << collision << " and vector: " << -diffusionVector << std::endl;
    LOG << "Within diffusion vector without minus, Collision is " << collision << " and vector: " << diffusionVector << std::endl;
    return -diffusionVector;
}

void CFootBotNavigation::SetWheelSpeedsFromVector(const CVector2& heading) {
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
        // CollisionMessages.push_back(new CCollisionTrace(Id));
        LOG << "Mobile Robot collision!" << std::endl;
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
            LOG << "Mobile Robot "  << " employing NO_TURN! "<< std::endl;

            break;
        }
        case SWheelTurningParams::SOFT_TURN: {
            // Both wheels still go straight but one of the wheels is faster than the other
            Real fSpeedFactor = (WheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) /
                WheelTurningParams.HardTurnOnAngleThreshold;
            fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);   // Slower
            fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);   // Faster

            // Debug = pass
            LOG << "Mobile Robot " << " employing SOFT_TURN! "<< std::endl;

            break;
        }
        case SWheelTurningParams::HARD_TURN: {
            // Both wheels have opposite speeds
            fSpeed1 = -WheelTurningParams.MaxSpeed;
            fSpeed2 = WheelTurningParams.MaxSpeed;

            // Debug = pass
            LOG << "Mobile Robot "  << " employing HARD_TURN! "<< std::endl;

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
    // Debug
    // LOG << "Mobile Robot " << Id << " applied wheel speed with L, R " << fLeftWheelSpeed << " " << fRightWheelSpeed << std::endl;
    m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

REGISTER_CONTROLLER(CFootBotNavigation, "footbot_navigation_controller")