#ifndef FOOTBOT_DIFFUSION_H
#define FOOTBOT_DIFFUSION_H

/*
 * Include some necessary headers.
 */
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
#include <vector>

using namespace argos;

class CFootBotDiffusion : public CCI_Controller {
    public:
        /* 
         * This structure contains variables that will be used as params for the diffusion algorithm.
         * Can also configure the value via XML file
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
         */
        struct SWheelTurningParams {
            /*
             * The Wheel turning mechanism ->
             *  NO_TURN = Maintain same velocity for both wheels
             * SOFT_TURN = Different velocity for both wheels but with minor difference between the two
             *              velocity values
             * HARD_TURN = Both wheel's velocity were set with opposing values whereby there is 
             *              major differences between the two
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

            // the maximum wheel velcity that is configured inside the XML file
            Real MaxSpeed;
            // Parses the params into the XML
            void Init(TConfigurationNode& t_tree);

        };

        /*
         * This structure contains params that define the robot's state information
         * Also contains mechanism (params, if else) to switch from one state to another
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
            // Designated goal for the robot -> represented by static robot's Id
            UInt32 GoalId;
            /*
             * The navigational information regarding the designated goal or other non-designated goal(s)
             *  Format:
             *       0: GoalID (Based on index)
             *       1: Sequence Number (age of info based on time step)
             *          - Reset to 0 as long a new navigational info is append into it
             *          - Increases when there is no new update
             *          - Can be used as a factor to determine if the receiver should 
             *            use/ update its table based on the related info or reject it
             *             - SeqNum > 60 -> Reject else Accept
             *             - Own SeqNum > Sender SeqNum -> Update with Sender info else retain own ifo
             *       2: Range (Relative Distance between the goal and robot based on cm/ Real)
             *          - Constantly updated as long within the range of the goal's broadcast range
             *          - TODO: might include ways to update the range info based on its distance travelled
             *                  from the position where it last received it from the goal itself
             *       3: Horizontal Bearing (Angle between robot's local x axis and the origin of meesage sender/ goal)
             *          - In CRadians
             *          -  Similar to Range
             * 
             */
            std::vector<Real> GoalNavigationalInfo;
            /*
             * The navigational table may contain all the goal(s) info 
             * Format: {NavigationalInfo, NavigationalInfo, ...}
             */
            UInt8 NumberOfGoals;

            std::vector<std::vector<Real>> NavigationalTable;

            // format from real to byte of navigational info
            CByteArray RealToByte(std::vector<Real>& v_info);
            // vice versa
            std::vector<Real> ByteToReal(CByteArray& b_array);
            // bool SortGoalId(const std::vector<Real>& v_a, const std::vector<Real>& v_b);
            
            // compare a goal info with another goal info
            std::vector<Real> CompareGoalInfos(std::vector<Real>& v_info1, std::vector<Real>& v_info2);

            // Use to switch from AGGRESSIVE_EXPLORATION -> MOVE_TO_GOAL
            bool FoundDesignatedGoal;
            /*
             * Use to swtich from MOVE_TO_GOAL -> RESTING
             * For now, flag = true if range < ~20cm
             */
            bool ReachedDesignatedGoal;

            SStateData();
            void Init(TConfigurationNode& t_node);
            void Reset();
            void SaveState();
    };

    public:
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

        inline EState GetState() { return StateData.State; }
        
        // returns the tracing messages for the robot
        inline std::vector<CTraceMessage*> *GetTraceMessages() { return &TraceMessages; }

        inline std::vector<CTraceMessage*> *GetCollisionMessages() { return &CollisionMessages; }

        inline UInt32 GetId() { return Id; }

    protected:
        /*
         * Updates the state information accordinly based on the availability of 
         * designate goal's navigational info inside the robot's navigational table
         * 
         *  
         */
        void UpdateState();
        
        /*
         * Calculates the vector towards the goal based on the navigational info.
         * Should return CVector2(Real Range, CRadians Horizontal bearing)
         */
        CVector2 CalculateVectorToGoal();

        /*
         * Calculates the diffusion vector.
         * TODO: Might need to re-edit the following line if the info is incorrect
         * Based on the principle of gas diffusion and it is used in the RANDOM EXPLORATION & AGGRESSIVE ...
         * If there is a close obstalce, it points the robot's local heading away from the obstalce
         * Else if there is no obstalce, it points the robot's local heading forward
         */
        CVector2 DiffusionVector(bool& b_collision);
        
        /*
         * Crucial in controlling the robot's movement based on solely the direction vector
         * as it manipulates the wheel speeds for both by transforming into wheel actuation.
         */
        void SetWheelSpeedsFromVector(const CVector2& c_heading);

        /*
         * Update the robot's navigational table every time step based on newly received navigational 
         * info sent by local neighbours in communication range
         * Also update the table based on info sent by the goal itself as well
         * returns the updated navigational table and would be used in the UpdateState() to determine
         * which state should it be and etc.
         * if t_packets[i].Data[i] == '?' -> Goal so update based on goal info sent by the goal itself
         * else -> compare own table and received table from sender 
         * everytime an info is update -> reset SeqNum to 0 else ++SeqNum for the last info's iteration
         * 
         * Despite the navigational table init with a fixed size, allow option for robot to append
         * a newly introduced goal into the environment by push_back()
         * 
         * 
         * TODO: #20 UpdateNavigationalTable should be void since we are updating the state's table inside
         * the method anyway
         */
        void UpdateNavigationalTable(const CCI_RangeAndBearingSensor::TReadings& t_packets);
        
        // This method is used for broadcasting navigational table through range and bearing actuator
        void BroadcastNavigationalTable();

        /*
         * Evaluates the newly updated navigational table right after UpdateNavigationalTable
         * This method checks the new table and seek info for designated goal then initialize
         * the info if found with StateData.GoalNavigationalInfo
         * 
         * Also determines which state a robot should be in every time step
         */
        void EvaluateNavigationalTable();

        // Starts the following states as the name implies
        void StartRandomExploration();
        
        void StartAggressiveExploration();

        void StartMoveToGoal();

        void StartResting();

        // Executes the following states based on its definition
        virtual void RandomExplore();

        virtual void AggressiveExplore();

        virtual void MoveToGoal();

        virtual void Rest();

    protected:
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
        static UInt32 s_unIdCounter;
        UInt32 Id;


        // The controller state information
        SStateData StateData;
        // The wheel turning params
        SWheelTurningParams WheelTurningParams;
        // The diffusion params
        SDiffusionParams DiffusionParams;
        std::vector<CTraceMessage*> TraceMessages;
        std::vector<CTraceMessage*> CollisionMessages;
        
};

#endif // !FOOTBOT_DIFFUSION_H
