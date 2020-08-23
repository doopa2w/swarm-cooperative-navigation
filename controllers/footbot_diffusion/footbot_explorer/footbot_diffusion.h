/*
 * AUTHOR: Darryl Tan <dtzl.added@gmail.com>
 *
 * TODO: Add more description for this controller; For now we are using light 
 * 
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */

#ifndef FOOTBOT_DIFFUSION_H
#define FOOTBOT_DIFFUSION_H

/*
 * Include some necessary headers.
 */
// Definition for the robot's state enumeration
#include "state.h"
// Definition for generating runtime traces
#include "trace_message.h"
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#include <argos3/core/utility/math/rng.h>


using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotDiffusion : public CCI_Controller {

   public:


      /* //////////////////////////////////////////////////////////////////////////////////////////
       * This structure contains variables that will be used as params for the diffusion algorithm.
       * Can also configure the value via XML file
       * 
       */
      struct SDiffusionParams {
         /*
          * Maximum tolerance for the proximity reading between the robot and the closest
          * obstacle.
          * The proximity reading is 0 when nothing is detected within its raange.
          * The reading increases exponentially to 1 when the obstacle gets nearer to the robot
          * until it touches the robot itself.
          * 
          */
         Real Delta;
         // Angle tolearnce range to go straight
         CRange<CRadians> GoStraightAngleRange;
         // Constructor
         SDiffusionParams();
         // Parses the XML section for the diffusion algorithm
         void Init(TConfigurationNode& t_tree);

      };
      /////////////////////////////////////////////////////////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////////////////

      /*
       * The following structure contains variables used for tuning the wheel during navigation.
       * Can also configure the value via XML file
       * 
       */
      struct SWheelTurningParams {
         /*
          * The Turning Mecahanism.
          * Contains all the different turning states for a mobile robot.
          * 
          * NO_TURN -> Maintain same speed for both wheels.
          * SOFT_TURN -> Different speed for the wheels with minimal differences in speed.
          * HARD_TURN -> Different speed for the wheels with major differences in speed.
          *                Both wheels turning at opposite speed.
          * 
          */
         enum ETurningMechanism {
            NO_TURN = 0,
            SOFT_TURN,
            HARD_TURN
         } TurningMechanism;

         // The angular threshold values for the respective turning mecahnism
         CRadians NoTurnAngleThreshold;
         CRadians SoftTurnOnAngleThreshold;
         CRadians HardTurnOnAngleThreshold;

         // Maximum wheel speed
         Real MaxSpeed;
         // Parses the XML section for the diffusion algorithm
         void Init(TConfigurationNode& t_tree);
      };

      /////////////////////////////////////////////////////////////////////////////////////////////
      ////////////////////////////////////////////////////////////////////////////////////////////

      /*
       * The following structure contains variables defining the state information about
       * the controller.
       * Also contains mechanism on when to change from one state to another.
       * 
       */
      struct SStateData {
         // The possible states in which a controller could be
         EState State; // current state
         EState PreviousState;

         // TODO: Include params for switching from random exploration to aggressive exploration

         SStateData();
         void Init(TConfigurationNode& t_node);
         void Reset();
         void SaveState();
         bool ReachedGoal();
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
      virtual void Reset();

      /*
      * Called to cleanup what done by Init() when the experiment finishes.
      * In this example controller there is no need for clean anything up,
      * so the function could have been omitted. It's here just for
      * completeness.
      */
      virtual void Destroy() {}

      /*
       * The following are functions to check the current robot states and information
       * 
       */

      inline bool IsRandomExploring() const { return StateData.State == RANDOM_EXPLORATION; }
      
      inline bool isAggressiveExploring() const { return StateData.State == AGGRESSIVE_EXPLORATION; }

      inline bool isResting() const { return StateData.State == RESTING; }
      
      inline bool isMovingToGoal() const { return StateData.State == MOVE_TO_GOAL; }

      inline EState GetState() { return StateData.State; }

      // Returns the tracing messages for this robot
      inline std::vector<CTraceMessage*> *GetTraceMessages() { return &TraceMessages; }

      inline std::vector<CTraceMessage*> *GetCollisionMessages() { return &CollisionMessages; }

      inline UInt32 GetId() { return Id; }


   protected:
      /*
       * Updates the robot state information.
       */
      void UpdateState();

      /* 
       * Calculate the vector to the light. Used to perform phototaxis and antiphototaxis
       * TODO: WILL REMOVE THIS AND REPLACED IT WITH NAVIGAIONAL TABLE
       * 
       */
      CVector2 CalculateVectorToLight();

      /*
       * Calculates the diffusion vector.
       * If there is a close obstacle, it points away from it.
       * If there is no obstalce, it points forward.
       * The following parameter is used to determine whether collision avoidance occured
       * or not.
       * It is necessary for collision rule.
       * 
       */
      CVector2 DiffusionVector(bool& b_collision);

      // Gets a direction vector as input and trasform it into wheel actuation.
      void SetWheelSpeedsFromVector(const CVector2& c_heading);

      // Starts the following states
      void StartRandomExploring();

      void StartAggressiveExploring();

      void StartResting();

      void StartMovingToGoal();

      // Executes the following states
      virtual void RandomExplore();

      virtual void AggressiveExplore();

      virtual void MoveToGoal();

      virtual void Rest();

      /* Pointer to the differential steering actuator */
      CCI_DifferentialSteeringActuator* m_pcWheels;
      /* Pointer to the foot-bot proximity sensor */
      CCI_FootBotProximitySensor* m_pcProximity;
      /* Pointer to the range and bearing actuator */
      CCI_RangeAndBearingActuator*  m_pcRABA;
      /* Pointer to the range and bearing sensor */
      CCI_RangeAndBearingSensor* m_pcRABS;
      /* Pointer to the foot-bot light sensor */
      CCI_FootBotLightSensor* m_light;
      // The Random Number Generator for ARGos
      CRandom::CRNG* m_RNG;

      static UInt32 s_unIdCounter;
      UInt32 Id;

      // The controller state information
      SStateData StateData;
      // The turning parameters
      SWheelTurningParams WheelTurningParams;
      // The diffusion paramters;
      SDiffusionParams DiffusionParams;
      
      std::vector<CTraceMessage*> TraceMessages;
      std::vector<CTraceMessage*> CollisionMessages;

};

#endif
