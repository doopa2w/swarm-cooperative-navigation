// Include the controller definition
#include "footbot_static.h"
#include <argos3/core/utility/logging/argos_log.h>

CFootBotTarget::CFootBotTarget() :
    m_pcLEDS(NULL),
    m_pcRABA(NULL) {}

UInt32 CFootBotTarget::s_unIdCounter = 0;

void CFootBotTarget::Init(TConfigurationNode& t_node) {

    // set Id
    Id = s_unIdCounter++;
    seqNum = 0;


    // Get actuator handlers
    m_pcLEDS = GetActuator<CCI_LEDsActuator>("leds");
    m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");

    // LEDs color
    int clrid = 0;
    GetNodeAttributeOrDefault(t_node, "colorid", clrid, clrid);
    GetNodeAttribute(t_node, "message_size", messageSize);

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

    // SIZE = 10; // or whatever you set in the XML file
    // CByteArray cBuf; // create a buffer for the message to send
    // float x = ... // whatever you want it to be
    // float y = ... // whatever you want it to be
    // cBuf << x; // this adds a float (4 bytes) to the buffer
    // cBuf << y; // this adds another float (4 bytes) to the buffer
    // // now the buffer is 8 bytes, but it must be SIZE
    // // keep adding a byte until the size is filled (there are faster ways to do this, it's just an example)
    // while (cBuf.size() < SIZE) cBuf << '\0';
    // // now cBuf is ready
    // m_pcRABA->SetData(cBuf);
    seqNum++;
    CByteArray cBuf;
    std::string identifier = "G";
    cBuf << identifier;
    cBuf << '\0';   // terminating character
    cBuf << seqNum;
    // range
    Real distance = 201.59;
    cBuf << distance;
    cBuf << '\0';
    // angle in CRadians
    Real angle = 2.23123;
    cBuf << angle;
    cBuf << '\0';
    // LOG << "Before filling in " << cBuf.Size() << std::endl;

    while (cBuf.Size() < messageSize) cBuf << '\0';
    m_pcRABA->SetData(cBuf);

}

REGISTER_CONTROLLER(CFootBotTarget, "footbot_static_controller")