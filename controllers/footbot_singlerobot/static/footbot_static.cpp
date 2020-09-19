// Include the controller definition
#include "footbot_static.h"
#include <argos3/core/utility/logging/argos_log.h>


/**************************************************************************/
/**************************************************************************/

CFootBotTarget::SNavigationData::SNavigationData() :
    NumberOfGoals(2) {}

void CFootBotTarget::SNavigationData::Init(TConfigurationNode& t_node) {
    GetNodeAttributeOrDefault(t_node, "number_of_goals", NumberOfGoals, NumberOfGoals);
    GetNodeAttribute(t_node, "size_of_message", SizeOfMessage);
}

void CFootBotTarget::SNavigationData::Reset() {
    for (size_t i = 0; i < NumberOfGoals; ++i) {
        NavigationalTable[i] = {
            {"SequenceNumber", 0}, {"EstimateDistance", 0}, {"Angle", 0}
        };
    }

}


/**************************************************************************/
/**************************************************************************/

CByteArray CFootBotTarget::SNavigationData::RealToByte(std::map<UInt32, std::map<std::string, Real>>& m_info, UInt8 GoalID) {
    CByteArray cBuf;


    cBuf << GoalID;
    cBuf << '\0';

    for (auto & outer_pair : m_info) {
        for (auto & inner_pair : outer_pair.second) {
            /*
             * For every goal's info, do:
             * Append the Seq followed by four 0's
             * Repeat this for distance and angle as well
             */
            cBuf << inner_pair.second;
            cBuf << '\0';
        }
        /*
         * Before going to the next row/goal, do:
         * Append a '\0' to separate the current and the next goal
         */
        // cBuf << '\0';
    }
    return cBuf;
}

/**************************************************************************/
/**************************************************************************/

CFootBotTarget::CFootBotTarget() :
    m_pcLEDS(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL) {}

UInt8 CFootBotTarget::s_unIdCounter = 0;

void CFootBotTarget::Init(TConfigurationNode& t_node) {

    // set Id
    Id = s_unIdCounter++;

    // TODO: Temp fix on issue where Id = 2 is skipped over some reason
    // For now Id-- for Id above 2
    if (Id > 2) 
        Id--;

    // Get actuator handlers
    m_pcLEDS = GetActuator<CCI_LEDsActuator>("leds");
    m_pcRABA = GetActuator<CCI_RangeAndBearingActuator>("range_and_bearing");
    m_pcRABS = GetSensor<CCI_RangeAndBearingSensor>("range_and_bearing");
    NavigationData.Init(GetNode(t_node, "navigation_info"));

    Reset();
}

void CFootBotTarget::Reset() {
    NavigationData.Reset();
    m_pcRABA->ClearData();
    // LEDs color
    int clrid = 4;
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



void CFootBotTarget::BroadcastNavigationalTable() {
    CByteArray cBuf = NavigationData.RealToByte(NavigationData.NavigationalTable, GetId());
    m_pcRABA->SetData(cBuf);

    // Debug 
    LOG << "Target Robot " << GetId() << " sending " << cBuf << std::endl;
}

void CFootBotTarget::ControlStep() {
    const CCI_RangeAndBearingSensor::TReadings& t_packets = m_pcRABS->GetReadings();
    BroadcastNavigationalTable();
    
}

REGISTER_CONTROLLER(CFootBotTarget, "footbot_static_controller")