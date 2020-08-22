// Include the controller definition
#include "footbot_static.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

CFootBotTarget::CFootBotTarget() :
    m_pcLEDS(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL) {}

UInt32 CFootBotTarget::s_unIdCounter = 0;
//CByteArray cBuf; // bufffer for the message to send
const int SIZE = 20; // Set in the XML file for the message payload size

void CFootBotTarget::Init(TConfigurationNode& t_node) {

    // set Id
    Id = s_unIdCounter++;


    // Get actuator handlers
    m_pcLEDS = GetActuator<CCI_LEDsActuator>("leds");
    m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
     m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );

    // LEDs color
    int clrid = 0;
    GetNodeAttributeOrDefault(t_node, "colorid", clrid, clrid);

    switch(clrid) {
        case 1:
            m_pcLEDS->SetAllColors(CColor::BLUE);
            break;
        case 2:
            m_pcLEDS->SetAllColors(CColor::GREEN);
            break;
        case 3:
            m_pcLEDS->SetAllColors(CColor::YELLOW);
            break;
        case 4:
            m_pcLEDS->SetAllColors(CColor::RED);
            break;
        default:
            break;
    }
}

void CFootBotTarget::ControlStep() {
    // broadcase message
    // GoalID
    //cBuf << GetId();
    // Keep adding until the size is filled
    //while (cBuf.Size() < SIZE) cBuf << '\0';
    //LOG << "Target " << Id << ": Am I Alive?" << std::endl;
    m_pcRABA->SetData(0, Id);
    const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
    

}

REGISTER_CONTROLLER(CFootBotTarget, "footbot_static_controller")