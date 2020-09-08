// Include the controller definition
#include "footbot_static.h"

CFootBotTarget::CFootBotTarget() :
    m_pcLEDS(NULL),
    m_pcRABA(NULL) {}

UInt32 CFootBotTarget::s_unIdCounter = 0;

void CFootBotTarget::Init(TConfigurationNode& t_node) {

    // set Id
    Id = s_unIdCounter++;


    // Get actuator handlers
    m_pcLEDS = GetActuator<CCI_LEDsActuator>("leds");
    m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");

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
    m_pcRABA->SetData(0, Id);

}

REGISTER_CONTROLLER(CFootBotTarget, "footbot_static_controller")