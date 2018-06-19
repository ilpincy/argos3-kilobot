#include "kilobot_communication_default_sensor.h"
#include "kilobot_entity.h"
#include "kilobot_communication_entity.h"
#include "kilobot_communication_medium.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CKilobotCommunicationDefaultSensor::CKilobotCommunicationDefaultSensor() :
      m_pcRobot(NULL),
      m_pcCommEntity(NULL),
      m_pcMedium(NULL),
      m_pcControllableEntity(NULL),
      m_fDistanceNoiseStdDev(0.0f),
      m_pcRNG(NULL),
      m_cSpace(CSimulator::GetInstance().GetSpace()),
      m_bShowRays(false) {}

   /****************************************/
   /****************************************/

   void CKilobotCommunicationDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      /* Assign Kilobot entity to this sensor */
      m_pcRobot = dynamic_cast<CKilobotEntity*>(&c_entity);
      /* Assign Kilobot communication entity to this sensor */
      m_pcCommEntity = &c_entity.GetComponent<CKilobotCommunicationEntity>("kilocomm");
      /* Get reference to controllable entity */
      m_pcControllableEntity = &c_entity.GetComponent<CControllableEntity>("controller");
   }

   /****************************************/
   /****************************************/

   void CKilobotCommunicationDefaultSensor::Init(TConfigurationNode& t_tree) {
      try {
         /* Parent class init */
         CCI_KilobotCommunicationSensor::Init(t_tree);
         /* Show rays? */
         GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
         /* Parse noise */
         GetNodeAttributeOrDefault(t_tree, "noise_std_dev", m_fDistanceNoiseStdDev, m_fDistanceNoiseStdDev);
         if(m_fDistanceNoiseStdDev > 0.0f)
            m_pcRNG = CRandom::CreateRNG("argos");
         /* Get Kilobot communication medium from id specified in the XML */
         std::string strMedium;
         GetNodeAttribute(t_tree, "medium", strMedium);
         m_pcMedium = &(CSimulator::GetInstance().GetMedium<CKilobotCommunicationMedium>(strMedium));
         /* Enable the Kilobot communication entity */
         m_pcCommEntity->SetMedium(*m_pcMedium);
         m_pcCommEntity->Enable();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the Kilobot communication medium sensor", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CKilobotCommunicationDefaultSensor::Update() {
      /*
       * Variable definitions
       */
      /* Buffer for the received packet */
      CCI_KilobotCommunicationSensor::SPacket sPacket;
      /*
       * Housekeeping
       */
      /* Buffer for calculating the message--robot distance */
      CVector3 cVectorRobotToMessage;
      /* Update status of last delivery */
      m_bMessageSent = (m_pcCommEntity->GetTxStatus() == CKilobotCommunicationEntity::TX_SUCCESS);
      /* Delete old readings */
      m_tPackets.clear();
      /*
       * OHC message processing
       */
      /* Get OHC message, if any */
      message_t* ptOHCMsg = m_pcMedium->GetOHCMessageFor(*m_pcRobot);
      if(ptOHCMsg != NULL) {
         sPacket.Message = ptOHCMsg;
         sPacket.Distance.low_gain = 0;
         sPacket.Distance.high_gain = 0;
         m_tPackets.push_back(sPacket);
      }
      /*
       * Kilobot messages
       */
      /* Get list of communicating RABs */
      const CSet<CKilobotCommunicationEntity*,SEntityComparator>& setComms = m_pcMedium->GetKilobotsCommunicatingWith(*m_pcCommEntity);
      /* Go through communicating RABs and create packets */
      for(CSet<CKilobotCommunicationEntity*,SEntityComparator>::iterator it = setComms.begin();
          it != setComms.end(); ++it) {
         /* Create a reference to the Kilobot communication entity to process */
         CKilobotCommunicationEntity& cOtherCommEntity = **it;
         /* Add ray if requested */
         if(m_bShowRays) {
            m_pcControllableEntity->AddCheckedRay(false,
                                                  CRay3(cOtherCommEntity.GetPosition(),
                                                        m_pcCommEntity->GetPosition()));
         }
         /* Set message data */
         sPacket.Message = cOtherCommEntity.GetTxMessage();
         /* Set message distance */
         sPacket.Distance.low_gain = 0;
         sPacket.Distance.high_gain = Distance(m_pcCommEntity->GetPosition(),
                                               cOtherCommEntity.GetPosition()) * 1000.0;
         if(m_pcRNG)
            sPacket.Distance.high_gain += m_pcRNG->Gaussian(m_fDistanceNoiseStdDev);
         /* Add message to the list */
         m_tPackets.push_back(sPacket);
      } /* communicating neighbors loop */
   }
   
   /****************************************/
   /****************************************/

   void CKilobotCommunicationDefaultSensor::Reset() {
      m_tPackets.clear();
      m_bMessageSent = false;
   }

   /****************************************/
   /****************************************/

   void CKilobotCommunicationDefaultSensor::Destroy() {
      m_pcCommEntity->Disable();
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CKilobotCommunicationDefaultSensor,
                   "kilobot_communication", "default",
                   "Carlo Pinciroli [ilpincy@gmail.com]",
                   "1.0",
                   "The Kilobot communication sensor.",
                   "This sensor allows Kilobots to perform situated communication, i.e., a form of\n"
                   "wireless communication whereby the receiver also knows the distance of the\n"
                   "sender. This sensor is associated to the Kilobot communication medium. To be\n"
                   "able to use this sensor, you must add a Kilobot communication medium to the\n"
                   "<media> section. To use this sensor, in controllers you must include the\n"
                   "ci_kilobot_communication_sensor.h header. This sensor only allows a Kilobot to\n"
                   "receive messages. To send messages, you need the Kilobot communication actuator.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <kilobot_communication implementation=\"default\"\n"
                   "                               medium=\"kilomedium\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n\n"
                   "The 'medium' attribute must be set to the id of the Kilobot communication\n"
                   "medium declared in the <media> section.\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "It is possible to draw the rays shot by the Kilobot communication sensor in the\n"
                   "OpenGL visualization. This can be useful for sensor debugging but also to\n"
                   "understand what's wrong in your controller. In OpenGL, the rays are drawn in\n"
                   "cyan when two robots are communicating.\n"
                   "To turn this functionality on, add the attribute \"show_rays\" as in this\n"
                   "example:\n\n"
                   "  <controllers>\n"
                   "    ...\n"
                   "    <my_controller ...>\n"
                   "      ...\n"
                   "      <sensors>\n"
                   "        ...\n"
                   "        <kilobot_communication implementation=\"default\"\n"
                   "                               medium=\"kilomedium\"\n"
                   "                               show_rays=\"true\" />\n"
                   "        ...\n"
                   "      </sensors>\n"
                   "      ...\n"
                   "    </my_controller>\n"
                   "    ...\n"
                   "  </controllers>\n",
                   "Usable");
   
}
