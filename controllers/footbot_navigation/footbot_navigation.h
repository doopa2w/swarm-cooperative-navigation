#ifndef FOOTBOT_NAVIGATION_H
#define FOOTBOT_NAVIGATION_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definitions for random number generator in ARGoS */
#include <argos3/core/utility/math/rng.h>
/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>
#include <map>
#include <string>

using namespace argos;

class CFootBotNavigation : public CCI_Controller {

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

            // the maximum wheel velcity that is configured inside the XML file
            Real MaxSpeed;
            // Parses the params into the XML
            void Init(TConfigurationNode& t_tree);

        };

        struct SNavigationData {

            enum ENavigationalInfo {
                SEQ_NUM = 0,
                ESTIMATE_DISTANCE = 1,
                ANGLE = 2
            } Nav;

            std::map<std::string, Real> GoalNavigationalInfo;

            bool FoundDesignatedGoal;
            bool ReachedDesignatedGoal;

            SNavigationData() {};
            // void Init(TConfigurationNode& t_node);
            void Reset();            
        };

    public:
        CFootBotNavigation();
        virtual ~CFootBotNavigation() {}
        virtual void Init(TConfigurationNode& t_node);
        // The actions to be executed by the robot every time step is called 
        // inside this funciton/ essentially the main entry in a sense for the robot
        virtual void ControlStep();
        // Resets the controller according to the definition
        virtual void Reset();
        // Just for the sake of completeness based on ARGoS's conventions
        virtual void Destroy() {}

    protected:
        CVector2 CalculateVectorToGoal();

        CVector2 DiffusionVector(bool& b_collision);
        
        /*
         * Crucial in controlling the robot's movement based on solely the direction vector
         * as it manipulates the wheel speeds for both by transforming into wheel actuation.
         */
        void SetWheelSpeedsFromVector(const CVector2& c_heading);

        // Update odometry information
        void UpdateOdometry(const CVector2& heading);


    protected:
        /* Pointer to the differential steering actuator */
        CCI_DifferentialSteeringActuator* m_pcWheels;
        /* Pointer to the foot-bot proximity sensor */
        CCI_FootBotProximitySensor* m_pcProximity;
        /* Pointer to the range and bearing actuator */
        CCI_RangeAndBearingActuator*  m_pcRABA;
        /* Pointer to the range and bearing sensor */
        CCI_RangeAndBearingSensor* m_pcRABS;
        /* Pointer to the LEDs actuator */
        CCI_LEDsActuator* m_pcLEDs;
        // The Random Number Generator for ARGos
        CRandom::CRNG* m_RNG;

        SNavigationData NavData;
        SWheelTurningParams WheelTurningParams;
        // The diffusion params
        SDiffusionParams DiffusionParams;
};



#endif // !FOOTBOT_NAVIGATION_H