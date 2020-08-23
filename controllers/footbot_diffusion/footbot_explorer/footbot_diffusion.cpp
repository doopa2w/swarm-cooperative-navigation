/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>

/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL),
   m_light(NULL),
   m_RNG(NULL) {}


UInt32 CFootBotDiffusion::s_unIdCounter = 0;

/****************************************/
/****************************************/

void CFootBotDiffusion::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><footbot_diffusion><actuators> and
    * <controllers><footbot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */

   // Set Id
   Id = s_unIdCounter++;

   // Initializing actuator/ sensor handlers
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
   m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
   m_light       = GetSensor  <CCI_FootBotLightSensor          >("footbot_light"        );

   // Initialize other stuffs as well
   m_RNG = CRandom::CreateRNG("argos");
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */

   DiffusionParams.Init(GetNode(t_node, "diffusion"));
   WheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
   StateData.Init(GetNode(t_node, "state"));

   Reset();
}

/****************************************/
/****************************************/

void CFootBotDiffusion::ControlStep() {

   UpdateState();

   switch(StateData.State) {
      case RANDOM_EXPLORATION:
         RandomExplore();
         break;
      
      case AGGRESSIVE_EXPLORATION:
         StartAggressiveExploring();
   }
   

   // broadcast and receive message
   m_pcRABA->SetData(0,69);
   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
   for (size_t i = 0; i < tPackets.size(); i++)
   {
      LOG << "Robot ID: " << GetId() << " Received Message: " << tPackets[i].Data << std::endl << "Received at: " 
         << tPackets[i].VerticalBearing << ", " << tPackets[i].HorizontalBearing << "\tApart by: " << tPackets[i].Range
         << "cm" << std::endl;
   }

}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotDiffusion, "footbot_diffusion_controller")
