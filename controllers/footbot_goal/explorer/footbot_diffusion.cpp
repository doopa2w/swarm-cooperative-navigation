/* Include the controller definition */
#include "footbot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>
#include <string>

/****************************************/
/****************************************/

CFootBotDiffusion::CFootBotDiffusion() :
   m_pcWheels(NULL),
   m_pcProximity(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL),
   m_cAlpha(10.0f),
   m_fDelta(0.5f),
   m_fWheelVelocity(2.5f),
   m_cGoStraightAngleRange(-ToRadians(m_cAlpha),
                           ToRadians(m_cAlpha)) {}

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
   m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcProximity = GetSensor  <CCI_FootBotProximitySensor      >("footbot_proximity"    );
   m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
   m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "alpha", m_cAlpha, m_cAlpha);
   m_cGoStraightAngleRange.Set(-ToRadians(m_cAlpha), ToRadians(m_cAlpha));
   GetNodeAttributeOrDefault(t_node, "delta", m_fDelta, m_fDelta);
   GetNodeAttributeOrDefault(t_node, "velocity", m_fWheelVelocity, m_fWheelVelocity);
   GetNodeAttribute(t_node, "message_size", MessageSize);
}

/****************************************/
/****************************************/

void CFootBotDiffusion::ControlStep() {

   std::string identifier;
   CByteArray cBuf;



   /* Get readings from proximity sensor */
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   /* Sum them together */
   CVector2 cAccumulator;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      cAccumulator += CVector2(tProxReads[i].Value, tProxReads[i].Angle);
   }
   cAccumulator /= tProxReads.size();
   /* If the angle of the vector is small enough and the closest obstacle
    * is far enough, continue going straight, otherwise curve a little
    */
   CRadians cAngle = cAccumulator.Angle();
   if(m_cGoStraightAngleRange.WithinMinBoundIncludedMaxBoundIncluded(cAngle) &&
      cAccumulator.Length() < m_fDelta ) {
      /* Go straight */
      m_pcWheels->SetLinearVelocity(m_fWheelVelocity, m_fWheelVelocity);
   }
   else {
      /* Turn, depending on the sign of the angle */
      if(cAngle.GetValue() > 0.0f) {
         m_pcWheels->SetLinearVelocity(m_fWheelVelocity, 0.0f);
      }
      else {
         m_pcWheels->SetLinearVelocity(0.0f, m_fWheelVelocity);
      }
   }

   const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();

   for (CCI_RangeAndBearingSensor::SPacket tPacket : tPackets) {
      // Deep copy the const tPacket into a tmp buffer
      CByteArray cBuf = tPacket.Data;
      // LOG << cBuf << std::endl;
      /*
       * Different execution depending on the first byte (Identifier)
       * 1 -> Sent by Target robot(s)
       * 2 -> Sent by Mobile Robot(s)
       */
      // LOG << "Initial size: " << cBuf.Size() << std::endl;
      // LOG << cBuf << std::endl;


      UInt8 Identifier;
      UInt32 SequenceNumber;
      Real Range;
      Real Angle;


      cBuf >> Identifier;
      
      if (Identifier == 1) {
         // Recevied a packet from Target Robot
         //  LOG << "After size: " << cBuf.Size() << std::endl;
         // LOG << cBuf << std::endl;

         if (tPacket.Data == cBuf) {
            LOG << "Still the same brother!" << std::endl;
         }
         else {
            LOG << "Success!" << std::endl;
         }
         // Proceed to extract the info accordingly
         /*
          * 0 -3     t
          * 4 -7     Seq
          * 8 - 11   t
          * 12 - 23  Range(Real)
          * 24 - 27  t
          * 28 - 39  Horizontal(CRadians)
          * 40 - 43  t
          * 45 bytes per row (44 + 1 for the Identifier)
          */
         *cBuf(4,8) >> SequenceNumber;
         *cBuf(12,24) >> Range;
         *cBuf(28, 40) >> Angle;

         CRadians cAngle = CRadians(Angle);

         // LOG << "TESTING : " << SequenceNumber << ", " << Range << ", " << Angle << ", " << std::endl;
         std::vector<Real> info;
         info.push_back(SequenceNumber);
         info.push_back(Range);
         info.push_back(Angle);
         for (size_t i = 0; i < info.size(); i++) {
            LOG << info[i] << ", ";
         }
         

         // TODO: Convert Real to CRadians for the angle part




      }
      else if (Identifier == 2) {
         // Received a packet from Mobile Robot
         LOG << "Received from a mobile robot!" << std::endl;
      }
      else {
         LOG << "Error brother!" << std::endl;
      }

   }
}
   // broadcast and receive message
   /*
    *
    * 
    * 
    * SIZE = 10; // or whatever you set in the XML file
   CByteArray cBuf; // create a buffer for the message to send
   float x = ... // whatever you want it to be
   float y = ... // whatever you want it to be
   cBuf << x; // this adds a float (4 bytes) to the buffer
   cBuf << y; // this adds another float (4 bytes) to the buffer
   // now the buffer is 8 bytes, but it must be SIZE
   // keep adding a byte until the size is filled (there are faster ways to do this, it's just an example)
   while (cBuf.size() < SIZE) cBuf << '\0';
   // now cBuf is ready
    */


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
