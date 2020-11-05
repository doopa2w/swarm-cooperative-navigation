// Include the controller definition
#include "footbot_static.h"
// Include logging from ARGoS
#include <argos3/core/utility/logging/argos_log.h>

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void CFootBotStatic::SNavigationData::Init(TConfigurationNode& t_node) {
    GetNodeAttribute(t_node, "size_of_message", SizeOfMessage);
    GetNodeAttribute(t_node, "goal_id", GoalId);


}

void CFootBotStatic::SNavigationData::Reset() {
    LOG << "Target Robot initializing with GoalID = " << GoalId << std::endl;
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

CFootBotStatic::CFootBotStatic() :
    m_pcLEDS(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL) {}

UInt8 CFootBotStatic::s_unTargetCounter = 0;

void CFootBotStatic::Init(TConfigurationNode& t_node) {
    // Set Id based on counter
    Id = s_unTargetCounter++;
    // Initializes sensors/actuators
    m_pcLEDS = GetActuator<CCI_LEDsActuator>("leds");
    m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
    m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
    NavData.Init(GetNode(t_node, "nav_data_static"));

    // Resets the controller
    Reset();       
}

void CFootBotStatic::Reset() {
    // Resets the navigation data's setting
    NavData.Reset();
    // Clear buffer
    m_pcRABA->ClearData();
    // LEDs color
    UInt8 ColorId = 1;
    // TODO: For now we are only working with at most 4 unique goalID
    switch(ColorId) {
        case 1:
            m_pcLEDS->SetAllColors(CColor::RED);
            break;
        case 2:
            m_pcLEDS->SetAllColors(CColor::GREEN);
            break;
        case 3:
            m_pcLEDS->SetAllColors(CColor::BLUE);
            break;
        case 4:
            m_pcLEDS->SetAllColors(CColor::YELLOW);
            break;
        default:
            break;
    }
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void CFootBotStatic::BroadcastMessage() {
    /*
     * The message is simple whereby only the goalId is sent!
     * 
     * The current implementation is bad since we are wasting 
     * so many bytes since only a 10 bytes message packet is
     * sufficient.
     * 
     * Other mobile robots only need to read the first 8 bytes
     * to determine whether to continue reading the message
     * or ignore the rest (sent from target robot)
     * 
     */
    CByteArray cBuf;
    cBuf << NavData.GoalId;
    cBuf << '\0';
    // Everytime target robot broadcast (every time step), increase age
    // TODO: #30 Fix age not read properly from mobile robot after reaching two bytes since it only reads the leftmost byte and ignore the remaining
    NavData.Age++;
    cBuf << NavData.Age;
    while (cBuf.Size() < NavData.SizeOfMessage)
        cBuf << '\0';
    LOG << "Target Robot " << Id << ": " << cBuf << std::endl;
    m_pcRABA->SetData(cBuf);
}

void CFootBotStatic::ControlStep() {
    // Broadcast message
    BroadcastMessage();
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

REGISTER_CONTROLLER(CFootBotStatic, "footbot_static_controller")