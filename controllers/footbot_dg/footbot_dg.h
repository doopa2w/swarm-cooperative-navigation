/*
 * AUTHOR: Darryl Tan Zhe Liang <dtzl.added@gmail.com>
 * 
 * A diffusion controller for the foot-bot that also allow
 * it to navigate towards the goal/ target robot.
 * 
 * This controller makes the robots behave as gas particles (random movement).
 * The robots go straight until they get close enough to an obstacle, in
 * which case they turn, loosely simulating an elastic collision. The net
 * effect is that over time the robots diffuse in the environment.
 * 
 * The controller uses the proximity snesor to detect obstacles and the wheels
 * to move around. 
 * 
 * TODO: 
 *  - Add Range and Bearing Sensors and Actuators
 *  - Navigational table
 *  - Static Robot as Target Robot representing the GOAL
 *  - Expand for more than single robot
 */

#ifndef FOOTBOT_DIFFUSION_GOAL_H
#define FOOTBOT_DIFFUSION_GOAL_H

/* Include some necessary headers */
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>

using namespace argos;

class CFootBotDiffusionGoal : public CCI_Controller {

    public:

        // Class Constructor
        CFootBotDiffusionGoal();
        // Class Destructor
        virtual ~CFootBotDiffusionGoal();
        /* 
         * This function initializes the controller.
         * The 't_node' variable points to the <parameter> section in the XML file in
         * the <controllers><footbot_diffusion_controller> section.
         */
        virtual void Init(TConfigurationNode& t_node);
        /*
         * This function is called once every time step.
         * The lenght of the time step is set in the XML file.
         */
        virtual void ControlStep();
        // This function resets the controller to its state right after the Init()
        virtual void Reset() {}
        // This function cleanups what done by Init() when the experiment finishes.
        virtual void Destroy() {}

    private:

        // Pointer to the differential steering actuator
        CCI_DifferentialSteeringActuator* m_pcWheels;
        // Pointer to the foot-bot proximity sensor
        CCI_FootBotProximitySensor* m_pcProximity;
        /* 
         * Maximum tolerance for the angle between the robot heading direction
         * and the closest obstacle detected
         */
        CDegrees m_cAlpha;
        /* 
         * Maximum tolerance for the proximity reading between the robot
         * and the closest obstacle. The proximity reading is 0 when nothing is
         * detected. It grows exponentially to 1 when the obstacle gets closer 
         * until it touches the robot.
         */
        Real m_fDelta;
        // Wheel Speed
        Real m_fWheelVelocity;
        // Angle tolerance range to go straight. It is set to [-alpha, alpha]
        CRange<CRadians> m_cGoStraightAngleRange;



};


#endif