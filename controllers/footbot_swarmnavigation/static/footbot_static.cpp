// Include the controller definition
#include "footbot_static.h"
// Include logging from ARGoS
#include <argos3/core/utility/logging/argos_log.h>

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

void CFootBotStatic::SNavigationData::Init(TConfigurationNode& t_node) {
    GetNodeAttribute(t_node, "size_of_message", SizeOfMessage);
    GetNodeAttribute(t_node, "duplicate_goal_or_not", DuplicateGoal);
    GetNodeAttribute(t_node, "duplicated_goal_id", DuplicatedGoalId);
}

void CFootBotStatic::SNavigationData::Reset(UInt8 id) {
    // If allow duplicated goal, assigned goalId based on local Id - 1
    if (DuplicateGoal)
        GoalId = DuplicatedGoalId;
    else
        GoalId = id;
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

CFootBotStatic::CFootBotStatic() :
    m_pcLEDS(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL) {}

UInt8 CFootBotStatic::s_unIdCounter = 0;

void CFootBotStatic::Init(TConfigurationNode& t_node) {
    // Set Id based on counter
    Id = s_unIdCounter++;
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
    NavData.Reset(Id);
    // Clear buffer
    m_pcRABA->ClearData();
    // LEDs color
    UInt8 ColorId = NavData.GoalId;
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
    while (cBuf.Size() < NavData.SizeOfMessage)
        cBuf << '\0';
    m_pcRABA->SetData(cBuf);
}

void CFootBotStatic::ControlStep() {
    // Broadcast message
    BroadcastMessage();

    // Debug
    LOG << "Target Robot " << Id << " is representing Goal " <<
        NavData.GoalId << " and is broadcasting message!" << std::endl;

}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

REGISTER_CONTROLLER(CFootBotStatic, "footbot_static_controller")