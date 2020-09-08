/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL),
   m_light(NULL),
   m_RNG(NULL) {}


UInt32 CFootBotDiffusion::s_unIdCounter = 0;

/**************************************************************************/
/**************************************************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */

   // Set Id
   Id = s_unIdCounter++;

   // Initializing actuator/ sensor handlers
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
   m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
   m_light       = GetSensor  <CCI_FootBotLightSensor          >("footbot_light"        );

   // Initialize other stuffs as well
   m_RNG = CRandom::CreateRNG("argos");
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */

   DiffusionParams.Init(GetNode(t_node, "diffusion"));
   WheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
   StateData.Init(GetNode(t_node, "state"));

   Reset();
}

/**************************************************************************/
/**************************************************************************/

void CFootBotDiffusion::ControlStep() {
   UpdateState();
   switch(StateData.State) {
      case RANDOM_EXPLORATION: {
         RandomExplore();
         break;
      }
      case AGGRESSIVE_EXPLORATION: {
         StartAggressiveExploring();
         break;
      }
      case FOUND_GOAL: {
         StartFoundGoal();
         break;
      }
      case MOVE_TO_GOAL: {
         MoveToGoal();
         break;
      }
      case RESTING: {
         Rest();
         break;
      }
      default: {
         LOG << "BEEP BEEP! There's a bug!" << std::endl;
      }
   }
}

/**************************************************************************/
/**************************************************************************/

void CFootBotDiffusion::Reset() {
   // Reset robot state
   StateData.Reset();
}

/**************************************************************************/
/**************************************************************************/

void CFootBotDiffusion::UpdateState() {
   // TODO: #6 Evaluate Navigational Table Info

}

/**************************************************************************/
/**************************************************************************/

CVector2 CFootBotDiffusion::CalculateVectorToLight() {
   // Get readings from light sensor
   const CCI_FootBotLightSensor::TReadings& tLightReads = m_light->GetReadings();
   // Sum them together
   CVector2 cAccumulator;
   
   for (size_t i = 0; i < tLightReads.size(); ++i) {
      // Relative Distance between Light source and robot; Relative angle based on frame of reference
      cAccumulator += CVector2(tLightReads[i].Value, tLightReads[i].Angle);
   }
   // if the light was perceived, return the vector
   if (cAccumulator.Length() > 0.0f) { 
      return CVector2(1.0f, cAccumulator.Angle()); 
   }
   // else, return zero
   else {
      LOG << "Cannot detect any light within viciniy" << std::endl;
      return CVector2();
   }
}

/**************************************************************************/
/**************************************************************************/

CVector2 CFootBotDiffusion::DiffusionVector(bool& collision) {
   // Get readings from the proximity sensor
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   // Sum them together
   CVector2 diffusionVector;

   for (size_t i = 0; i < tProxReads.size(); ++i) {
      diffusionVector += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   /*
    * If the angle of the vector is small enough and the closest obstacle is far enough,
    * ignore the vector and proceed straight.
    * Otherwise, return it to take into consideration
    * 
    */
   if (DiffusionParams.GoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(diffusionVector.Angle()) &&
      diffusionVector.Length() < DiffusionParams.Delta) {
      collision = false;
   }
   else {
      collision = true;
   }
   diffusionVector.Normalize();
   // return opposite heading
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
   // Get previous Turning Mechanism
   SWheelTurningParams::ETurningMechanism oldTurningMechanism = WheelTurningParams.TurningMechanism;
   
   // Turning State Switching Conditions
   if (Abs(cHeadingAngle) <= WheelTurningParams.NoTurnAngleThreshold) {
      // No Turn + very small heading angle
      WheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
   }
   else if (Abs(cHeadingAngle) > WheelTurningParams.HardTurnOnAngleThreshold) {
      // Hard Turn + very large heading angle
      WheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
   }
   else if (Abs(cHeadingAngle) > WheelTurningParams.SoftTurnOnAngleThreshold && 
      WheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      // Soft Turn + heading angle in between NoTurn and HardTurn
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
         // Just go straight with no speed difference between the left and right wheels
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         // Both wheels still go straight, but one wheel is faster than the other
         Real fSpeedFactor = (WheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) /
            WheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 -  fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         // Now both wheels have opposite speeds
         fSpeed1 = -WheelTurningParams.MaxSpeed;
         fSpeed2 = WheelTurningParams.MaxSpeed;
         break;
      }
   }

   // Apply the new fSpeed1, fSpeed2 to the wheels
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if (cHeadingAngle > CRadians::ZERO) {
      // Turn left
      fLeftWheelSpeed = fSpeed1; // less speed
      fRightWheelSpeed = fSpeed2; // more speed
   }
   else {
      // Turn right
      fLeftWheelSpeed = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed); // set the wheel speeds
}

/**************************************************************************/
/**************************************************************************/




   // // broadcast and receive message
   // m_pcRABA->SetData(0,69);
   // const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
   // for (size_t i = 0; i < tPackets.size(); i++)
   // {
   //    LOG << "Robot ID: " << GetId() << " Received Message: " << tPackets[i].Data << std::endl << "Received at: " 
   //       << tPackets[i].VerticalBearing << ", " << tPackets[i].HorizontalBearing << "\tApart by: " << tPackets[i].Range
   //       << "cm" << std::endl;
   // }



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
