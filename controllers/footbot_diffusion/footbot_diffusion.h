#ifndef FOOTBOT_DIFFUSION_H
#define FOOTBOT_DIFFUSION_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the foot-bot wheel encoders */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definitions for random number generator in ARGoS */
#include <argos3/core/utility/math/rng.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotDiffusion : public CCI_Controller {

   public:
      struct SDiffusionParams {
         Real Delta;
         // Angle tolerance range to go straight
         CRange<CRadians> GoStraightAngleRange;
         // constrcutor
         SDiffusionParams();
         // parses the params to be configured inside the XML/ ARGOS file instead for 
         // easy configuration
         void Init(TConfigurationNode& t_tree);
      };

      struct SWheelTurningParams {
         /*
            * The Wheel turning mechanism ->
            *  NO_TURN = Maintain same velocity for both wheels
            * SOFT_TURN = Different velocity for both wheels but with minor difference between the two
            *              velocity values
            * HARD_TURN = Both wheel's velocity were set with opposing values whereby there is 
            *              major differences between the two
            * 
            */
         enum ETurningMechanism {
               NO_TURN = 0,
               SOFT_TURN = 1,
               HARD_TURN = 2
         } TurningMechanism;

         // TODO: later shorten into one line if no issue CRadians val1, val2, val3
         // The angular threshold values for the respective turning mechanisms
         CRadians NoTurnAngleThreshold;
         CRadians SoftTurnOnAngleThreshold;
         CRadians HardTurnOnAngleThreshold;

         // the maximum wheel velocity that is configured inside the XML file
         Real MaxSpeed;  // 10.0
         // Parses the params into the XML
         void Init(TConfigurationNode& t_tree);
      };

      struct SNavigationData {
         // Variables for testing purpose
         Real SDistance, SAngle;
         Real GDistance, GAngle;

         CVector2 PreviousVector;

         CDegrees PreviousRotation;
         CDegrees CurrentRotation;
         CDegrees TotalRotation;



         void Init(TConfigurationNode& t_tree);
      };


   /* Class constructor. */
   CFootBotDiffusion();

   /* Class destructor. */
   virtual ~CFootBotDiffusion() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset() {}

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

private:
   CVector2 VectorToGoal(bool goalorsender);

   CVector2 DiffusionVector(bool& b_collision);

   void SetWheelSpeedsFromVector(const CVector2& c_heading);



   // State 
   enum EState {
      SENDER = 0,
      GOAL = 1,
      REST = 2
   } State;


   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the foot-bot proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;
   /* Pointer to the range and bearing actuator */
   CCI_RangeAndBearingActuator*  m_pcRABA;
   /* Pointer to the range and bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABS;
   /* Pointer to wheele encoders */
   CCI_DifferentialSteeringSensor* m_pcEncoder;

   CCI_PositioningSensor* m_pcPosition;
   // The Random Number Generator for ARGos
   CRandom::CRNG* m_RNG;/* Pointer to the differential steering actuator */

   SWheelTurningParams WheelTurningParams;
   // The diffusion params
   SDiffusionParams DiffusionParams;

   SNavigationData NavigationParams;


};

#endif
