#ifndef FOOTBOT_DIFFUSION_H
#define FOOTBOT_DIFFUSION_H

// Definition for the robot's state enumeration
#include "state.h"
// Definition for generating runtime traces
#include "trace_message.h"
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
// Definition of the foot-bot LEDs actuator 
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definitions for random number generator in ARGoS */
#include <argos3/core/utility/math/rng.h>
/* Container for navigation info */
#include <map>
#include <string>

using namespace argos;

class CFootBotDiffusion : public CCI_Controller {
    public:
        /* 
         * This structure contains variables that will be used as params for the diffusion algorithm.
         * Can also configure the value via XML file
         * 
         */
        struct SDiffusionParams {
            Real Delta;
            // Angle tolerance range to go straight
            CRange<CRadians> GoStraightAngleRange;
            // constrcutor
            SDiffusionParams();
            // parses the params to be configured inside the XML/ ARGOS file instead for 
            // easy configuration
            void Init(TConfigurationNode& t_tree);
        };
        /*
         * This structure contains params used to change the wheel speed during navigation for 
         * turning purposes
         * Similar to the SDiffusionParams whereby the params are configured inside the XML
         * 
         */
        struct SWheelTurningParams {
            /*
             * The Wheel turning mechanism ->
             *  NO_TURN = Maintain same velocity for both wheels
             * SOFT_TURN = Different velocity for both wheels but with minor difference between the two
             *              velocity values
             * HARD_TURN = Both wheel's velocity were set with opposing values whereby there is 
             *              major differences between the two
             * 
             */
            enum ETurningMechanism {
                NO_TURN = 0,
                SOFT_TURN = 1,
                HARD_TURN = 2
            } TurningMechanism;

            // TODO: later shorten into one line if no issue CRadians val1, val2, val3
            // The angular threshold values for the respective turning mechanisms
            CRadians NoTurnAngleThreshold;
            CRadians SoftTurnOnAngleThreshold;
            CRadians HardTurnOnAngleThreshold;

            // the maximum wheel velocity that is configured inside the XML file
            Real MaxSpeed;  // 10.0
            // the maximum wheel velocity during RANDOM_EXPLORATION
            Real RESpeed;   // 6.0
            // Parses the params into the XML
            void Init(TConfigurationNode& t_tree);
        };
        /*
         * This structure contains params that define the robot's state information
         * Also contains mechanism (params, if else) to switch from one state to another
         * 
         */
        struct SStateData {
            // One of the possible states that a robot/ controller could be 
            EState State;
            // Critical for determining the turning mechanism for the robot in
            // the next time step
            EState PreviousState;
            // Used as a range for uniform number generator
            CRange<Real> ProbRange;
            // Number of time steps that a robot is in the RANDOM_EXPLORATION state
            size_t TimeRandomExplore;
            // Threshold value for switching from RANDOM_EXPLORATION to AGGRESSIVE_EXPLORATION
            size_t MaximumTimeInRandomExploration;
            // Flag for switching from any state to MOVE_TO_GOAL
            bool FoundDesignatedGoal;
            // Flag for switching from any state to ensure robot reached sender's initial position
            bool ReachedSender;
            // Flag for switching from MOVE_TO_GOAL to RESTING
            bool ReachedGoal;

            SStateData();
            void Init(TConfigurationNode& t_node);
            void Reset();
            void SaveState();
        };
        /*
         * This structure contains navigational info used in navigating towards
         * the designated goal
         * 
         */
        struct SNavigationData {
            /* The goal info with container */
            // The current goal info for Designated goal
            std::map<std::string, Real> GoalInfo;
            // The local navigation table containing n rows
            std::map<UInt8, std::map<std::string, Real>> NavigationTable;
            // The sender's relative location
            CVector2 SenderInfo;

            /* The odometry info to update the table */
            CVector2 PreviousMovement;

            /*  Settings to be configured inside XML */
            // Size of message packet
            UInt32 SizeOfMessage;
            // DesignatedGoalID
            UInt8 GoalId;
            // Number of goals
            UInt8 NumberOfGoals;

            /* Formatter */
            CByteArray ConvertToByte(std::map<UInt8, std::map<std::string, Real>>& m_info);

            std::map<UInt8, std::map<std::string, Real>> ConvertFromByte(CByteArray& b_array);

            SNavigationData();
            void Init(TConfigurationNode& t_node);
            void Reset(UInt8 local_id);
        };

        // Constructor and Deconstructor
        CFootBotDiffusion();
        virtual ~CFootBotDiffusion() {}
        // Initializes the controller and parses into the XML/ ARGOS file
        virtual void Init(TConfigurationNode& t_node);
        // The actions to be executed by the robot every time step is called 
        // inside this funciton/ essentially the main entry in a sense for the robot
        virtual void ControlStep();
        // Resets the controller according to the definition
        virtual void Reset();
        // Just for the sake of completeness based on ARGoS's conventions
        virtual void Destroy() {}

        // The following are methods used to check the current robot states and other info
        inline bool IsRandomExploring() const { return StateData.State == RANDOM_EXPLORATION; }

        inline bool isAggressiveExploring() const { return StateData.State == AGGRESSIVE_EXPLORATION; }

        inline bool isResting() const { return StateData.State == RESTING; }
      
        inline bool isMovingToGoal() const { return StateData.State == MOVE_TO_GOAL; }

        inline bool isMoveingToSender() const { return StateData.State == MOVE_TO_SENDER; }

        inline EState GetState() { return StateData.State; }

        // returns the tracing messages for the robot
        inline std::vector<CTraceMessage*> *GetTraceMessages() { return &TraceMessages; }

        inline std::vector<CTraceMessage*> *GetCollisionMessages() { return &CollisionMessages; }

        inline UInt8 GetId() { return Id; }


    protected:
        /*
         * Updates the state information accordinly based on the availability of 
         * designate goal's navigational info inside the robot's navigational table
         *  
         */
        void UpdateState();
        /*
         * Calculates the vector towards the goal based on the navigational info.
         * Should return CVector2(Real Range, CRadians Horizontal bearing)
         * Also calculates the vector towards the sender based on sender info
         * 
         * Calculate to goal = true
         * Calculate to sender = false
         */
        CVector2 CalculateVectorToGoal(bool b_goalorsender);
        /*
         * Calculates the diffusion vector.
         * TODO: Might need to re-edit the following line if the info is incorrect
         * Based on the principle of gas diffusion and it is used in the RANDOM EXPLORATION & AGGRESSIVE ...
         * If there is a close obstalce, it points the robot's local heading away from the obstalce
         * Else if there is no obstalce, it points the robot's local heading forward
         * 
         */
        CVector2 DiffusionVector(bool& b_collision);
        
        /*
         * Crucial in controlling the robot's movement based on solely the direction vector
         * as it manipulates the wheel speeds for both by transforming into wheel actuation.
         * 
         */
        void SetWheelSpeedsFromVector(const CVector2& c_heading);
        /*
         * Update local distance estimates firstmost before evaluating the info from others
         * 
         */
        void UpdateOdometry();
        /*
         * Updates local table every time step based on new navigation info from local neighbours
         * Also contains algorithm to compare info
         *
         */
        void UpdateNavigationTable(const CCI_RangeAndBearingSensor::TReadings& t_packets);
        // /*
        //  * Updates the navigation info for goal and sender info
        //  * Also contains algorithm to compare info and updates relevant flags
        //  * for transitioning between states
        //  * 
        //  */
        // void UpdateNavigationBehaviour(std::map<std::string, Real> local_info, std::map<std::string, Real> sender_info );
        /*
         * Format and broadcast the byte array
         * Runs at the very end of timestep
         * 
         */
        void BroadcastNavigationTable();

        // Starts the following states
        void StartRandomExploration();
        void StartAggressiveExploration();
        void StartMoveToSender();
        void StartMoveToGoal();
        void StartResting();

        // Executes the following states based on its definition
        virtual void RandomExplore();
        virtual void AggressiveExplore();
        virtual void MoveToSender();
        virtual void MoveToGoal();
        virtual void Rest();

        /* Pointer to the differential steering actuator */
        CCI_DifferentialSteeringActuator* m_pcWheels;
        /* Pointer to the foot-bot proximity sensor */
        CCI_FootBotProximitySensor* m_pcProximity;
        /* Pointer to the range and bearing actuator */
        CCI_RangeAndBearingActuator*  m_pcRABA;
        /* Pointer to the range and bearing sensor */
        CCI_RangeAndBearingSensor* m_pcRABS;
        /* Pointer to the LEDs actuator */
        CCI_LEDsActuator* m_pcLEDs;
        // The Random Number Generator for ARGos
        CRandom::CRNG* m_RNG;

        // robotId determined based on order of creation
        static UInt8 s_unMobileCounter;
        UInt8 Id;

        // The controller state information
        SStateData StateData;
        // The navigation info
        SNavigationData NavData;
        // The wheel turning params
        SWheelTurningParams WheelTurningParams;
        // The diffusion params
        SDiffusionParams DiffusionParams;
        std::vector<CTraceMessage*> TraceMessages;
        std::vector<CTraceMessage*> CollisionMessages;      
};

#endif // !FOOTBOT_DIFFUSION_H
