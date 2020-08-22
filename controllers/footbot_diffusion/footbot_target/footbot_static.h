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

class CFootBotTarget : public CCI_Controller {

    public:

        CFootBotTarget();
        virtual ~CFootBotTarget() {}


        // Returns the Id
        inline UInt32 GetId() { return Id; }

        virtual void Init(TConfigurationNode& t_node);
        virtual void ControlStep();
        virtual void Reset() {}
        virtual void Destroy() {}

    private:

        // Pointer to the LEDs actuator
        CCI_LEDsActuator* m_pcLEDS;
        /* Pointer to the range and bearing actuator */
        CCI_RangeAndBearingActuator*  m_pcRABA;
        /* Pointer to the range and bearing sensor */
        CCI_RangeAndBearingSensor* m_pcRABS;

        static UInt32 s_unIdCounter;
        UInt32 Id;

};

#endif
