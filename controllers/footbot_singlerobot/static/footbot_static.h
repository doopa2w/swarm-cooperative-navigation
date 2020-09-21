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
#include <map>
#include <string>

using namespace argos;

class CFootBotTarget : public CCI_Controller {

    public:

        struct SNavigationData {
            // TODO: Low effort for keeping track of the number of goals
            UInt8 NumberOfGoals;
            // Own Navigational Table
            std::map<UInt32, std::map<std::string, Real>> NavigationalTable;
            // Size of message
            UInt32 SizeOfMessage;
            SNavigationData();
            void Init(TConfigurationNode& t_node);
            void Reset();
        };

        CFootBotTarget();
        virtual ~CFootBotTarget() {}

        // Returns the Id
        inline UInt8 GetId() { return Id; }

        virtual void Init(TConfigurationNode& t_node);
        virtual void Destroy() {}
        virtual void Reset();
        virtual void ControlStep();

    protected:
        // Update NavigationalTable
        // void UpdateNavigationalTable(const CCI_RangeAndBearingSensor::TReadings& t_packets);
        // Broadcast the table
        void BroadcastNavigationalTable();

        // Pointer to the LEDs actuator
        CCI_LEDsActuator* m_pcLEDS;
        /* Pointer to the range and bearing actuator */
        CCI_RangeAndBearingActuator*  m_pcRABA;
        /* Pointer to the range and bearing sensor */
        CCI_RangeAndBearingSensor* m_pcRABS;

        static UInt8 s_unIdCounter;
        UInt8 Id;

        SNavigationData NavigationData;
};

#endif
