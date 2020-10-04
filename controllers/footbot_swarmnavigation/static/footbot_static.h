#ifndef FOOTBOT_STATIC_H
#define FOOTBOT_STATIC_H

// Include some necessary headers
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

using namespace argos;

class CFootBotStatic : public CCI_Controller {
    public:

        struct SNavigationData {
            // Size of message
            UInt16 SizeOfMessage;
            // Assigned GoalId
            UInt8 GoalId;
            // Setting for duplicating goalId
            bool DuplicateGoal;
            // Duplicated goal
            UInt8 DuplicatedGoalId;
            // Parse configuraiton in XML
            void Init(TConfigurationNode& t_node);
            void Reset(UInt8 id);
        };

        // Constructor and Desconstructor
        CFootBotStatic();
        virtual ~CFootBotStatic() {}
        // Returns the Id
        inline UInt8 GetId() { return Id; }
        // Parse configuration in XML
        virtual void Init(TConfigurationNode& t_node);
        virtual void Destroy();
        virtual void Reset();
        virtual void ControlStep();

    protected:

        // Broadacast the message in correct format
        void BroadcastMessage();
        // Pointer to the LEDs actuator
        CCI_LEDsActuator* m_pcLEDS;
        /* Pointer to the range and bearing actuator */
        CCI_RangeAndBearingActuator*  m_pcRABA;
        /* Pointer to the range and bearing sensor */
        CCI_RangeAndBearingSensor* m_pcRABS;
        // static variable for the Id counter
        static UInt8 s_unIdCounter;
        // Each robot should have unique Id
        UInt8 Id;
        SNavigationData NavData;
};

#endif // !FOOTBOT_STATIC_H
