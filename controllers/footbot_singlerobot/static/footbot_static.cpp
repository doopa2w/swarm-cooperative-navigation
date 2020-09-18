// Include the controller definition
#include "footbot_static.h"
#include <argos3/core/utility/logging/argos_log.h>


/**************************************************************************/
/**************************************************************************/

CFootBotTarget::SNavigationData::SNavigationData() :
    NumberOfGoals(3) {}

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

// std::map<UInt32, std::map<std::string, Real>> CFootBotTarget::SNavigationData::ByteToReal(CByteArray& b_array) {
//     std::map<UInt32, std::map<std::string, Real>> NeighboursTable;
//     CByteArray cBuf = b_array;
//     // Get first byte = Identifier
//     UInt8 identifier;
//     cBuf >> identifier;

//     for (size_t i = 0; i < NumberOfGoals; ++i) {
//         UInt32 SeqNum;
//         Real Range, Angle;
//         UInt16 factor = i * 40;
//         /*
//          * Proceed to extract the info accordinly
//          * 0-3      Identifier (Mobile = 0; Static = GoalID) ~ Extracted out!
//          * 
//          * 0-3      0
//          * 4-7      Seq
//          * 8-11     0
//          * 12-23    EstimateDistance
//          * 24-27    0
//          * 28-39    Angle
//          * 40-43    0
//          * 
//          * 44-47    Seq
//          * 48-51    0
//          * 52-63    EstimateDistance
//          * 64-67    0
//          * 68-79    Angle
//          * 80-83    0
//          * 
//          * 84-87
//          * 
//          * Size for a navigational info = 45 bytes (0 to 44)   
//          */
//         *cBuf(4 + factor, 8 + factor) >> SeqNum;
//         *cBuf(12 + factor, 24 + factor) >> Range;
//         *cBuf(28 + factor, 40 + factor) >> Angle;
//         // finally insert into the goal
//         NeighboursTable[i] = {
//             {
//                 {"SequenceNumber", SeqNum},
//                 {"EstimateDistance", Range},
//                 {"Angle", Angle}        
//             }
//         };

//     }
//     return NeighboursTable;

// }

/**************************************************************************/
/**************************************************************************/

CByteArray CFootBotTarget::SNavigationData::RealToByte(std::map<UInt32, std::map<std::string, Real>>& m_info, UInt8 GoalId) {
    CByteArray cBuf;

    cBuf << GoalId + 1;
    cBuf << '\0';

    // Debug
    LOG << "Got GoalID of " << GoalId + 1 << std::endl; 

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

// std::map<std::string, Real> CFootBotTarget::SNavigationData::CompareGoalInfos(std::map<std::string, Real>& m_info1, std::map<std::string, Real>& m_info2) {
//     /*
//      * Scoring function to calculate quality of info based on
//      * Sequence Number (Relative age of the info) & EstimateDistance (Relative Distance)
//      * The lower the score, the better the info.
//      * 
//      * TODO: #26 Might add Angle as another factor as well?
//      * 
//      * AScore, BScore > AScore 
//      */
//     UInt32 AScore, BScore;
//     AScore = m_info1["SequenceNumber"] * m_info1["EstimateDistance"] + m_info1["EstimateDistance"];
//     BScore = m_info2["SequenceNumber"] * m_info2["EstimateDistance"] + m_info2["EstimateDistance"];

//     if (AScore > BScore) {
//         return m_info2;
//     }
        
//     else if (AScore < BScore){
//         return m_info1;
//     }
//     // if both equal, return either one 
//     else {
//         return m_info1;
//     }
// }

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

// void CFootBotTarget::UpdateNavigationalTable(const CCI_RangeAndBearingSensor::TReadings& t_packets) {
//     /*
//      * This is where own table is updated accordinly based on the local neihgbours tables
//      * 
//      * 
//      * TOOD: Issue: Target Robot appending relative distance/ angle into the table's info
//      *              Might cause an issue since we are moving first based on the NeighboursNavigationalInfo
//      *              before proceeding towards the goal using info from the updated table.
//      * 
//      */
//     // std::map<UInt32, std::map<std::string, Real>> NeighboursTable;

//     for (CCI_RangeAndBearingSensor::SPacket t_packet : t_packets) {
//         // StateData.NeighboursNavigationalInfo["EstimateDistance"] = t_packet.Range;
//         // StateData.NeighboursNavigationalInfo["Angle"] = t_packet.HorizontalBearing;
//         std::map<UInt32, std::map<std::string, Real>> NeighboursTable;
//         UInt8 identifier = t_packet.Data[3];

//         if (identifier == 0) {
//             // This is mobile robot
//             NeighboursTable = NavigationData.ByteToReal(t_packet.Data);
//         }
//         else {
//             // This is target robot
//             NeighboursTable = NavigationData.ByteToReal(t_packet.Data);
//             /*
//              * Update the target robot's goal info with its navigational info
//              */
//             // LOG << t_packet.Range << ", " << t_packet.HorizontalBearing.GetValue();
//             NeighboursTable[identifier-1]["EstimateDistance"] = t_packet.Range;
//             NeighboursTable[identifier-1]["Angle"] = t_packet.HorizontalBearing.GetValue();
//         }

//         // Debug
//         LOG << "From Goal ID " << Id << std::endl;
//         for (auto it1 = NeighboursTable.begin(); it1 != NeighboursTable.end(); ++it1) {
//             LOG << it1->first << "  ";
//             for (auto it2 = it1->second.begin(); it2 != it1->second.end(); ++it2) {
//                 LOG << it2->second <<", ";
//             }
//             LOG << " ";
//         }


//         // Iterate two maps per row since we know the keys are identical and in order with one another
//         for (auto it1 = NavigationData.NavigationalTable.begin(), it2 = NeighboursTable.begin();
//              it1 != NavigationData.NavigationalTable.end();
//              ++it1, ++it2) {
//             // Skip updating own info since it will always be better
//             if (it1->first == Id) {
//                 continue;
//             }
//             else {
//                 // Get the better info and update the info

//                 //Deb
//                 LOG << "\nComparing \n" << it1->second["EstimateDistance"] << " <-> " << it2->second["EstimateDistance"] << std::endl;   
//                 it1->second = NavigationData.CompareGoalInfos(it1->second, it2->second);

//             }
//         }
        
//     }
// }

void CFootBotTarget::BroadcastNavigationalTable() {
    CByteArray cBuf = NavigationData.RealToByte(NavigationData.NavigationalTable, Id);
    m_pcRABA->SetData(cBuf);


    // Debug 
    LOG << "Target Robot " << Id << " sending " << cBuf << std::endl;
}

void CFootBotTarget::ControlStep() {
    const CCI_RangeAndBearingSensor::TReadings& t_packets = m_pcRABS->GetReadings();
    // UpdateNavigationalTable(t_packets);


    // Debug
    // LOG << "From Goal ID " << Id << std::endl;
    // for (auto it1 = NavigationData.NavigationalTable.begin(); it1 != NavigationData.NavigationalTable.end(); ++it1) {
    //     LOG << it1->first << " ";
    //     for (auto it2 = it1->second.begin(); it2 != it1->second.end(); ++it2) {
    //         LOG << it2->second <<", ";
    //     }
    //     LOG << " ";
    // }


    BroadcastNavigationalTable();
    
}

REGISTER_CONTROLLER(CFootBotTarget, "footbot_static_controller")