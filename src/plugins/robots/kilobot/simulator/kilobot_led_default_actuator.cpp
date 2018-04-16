/**
 * @file <argos3/plugins/robots/kilobot/simulator/kilobot_led_default_actuator.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "kilobot_led_default_actuator.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/logging/argos_log.h>

namespace argos {

   /****************************************/
   /****************************************/

   CKilobotLEDDefaultActuator::CKilobotLEDDefaultActuator() :
      m_pcLEDEquippedEntity(NULL) {}

   /****************************************/
   /****************************************/

   void CKilobotLEDDefaultActuator::SetRobot(CComposableEntity& c_entity) {
      m_pcLEDEquippedEntity = &(c_entity.GetComponent<CLEDEquippedEntity>("leds"));
   }

   /****************************************/
   /****************************************/

   void CKilobotLEDDefaultActuator::Init(TConfigurationNode& t_tree) {
      try {
         CCI_KilobotLEDActuator::Init(t_tree);
         m_pcLEDEquippedEntity->Enable();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the LEDs default actuator", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CKilobotLEDDefaultActuator::Update() {
      m_pcLEDEquippedEntity->SetLEDColor(0, m_cColor);
   }

   /****************************************/
   /****************************************/

   void CKilobotLEDDefaultActuator::Reset() {
      SetColor(CColor::BLACK);
   }

   /****************************************/
   /****************************************/

   void CKilobotLEDDefaultActuator::Destroy() {
      m_pcLEDEquippedEntity->Disable();
   }

   /****************************************/
   /****************************************/

}

REGISTER_ACTUATOR(CKilobotLEDDefaultActuator,
                  "kilobot_led", "default",
                  "Carlo Pinciroli [ilpincy@gmail.com]",
                  "1.0",
                  "The Kilobot LED actuator.",
                  "This actuator controls the LED of the kilobot. For a complete description of its\n"
                  "usage, refer to the ci_kilobot_led_actuator.h file.\n\n"
                  "REQUIRED XML CONFIGURATION\n\n"
                  "  <controllers>\n"
                  "    ...\n"
                  "    <my_controller ...>\n"
                  "      ...\n"
                  "      <actuators>\n"
                  "        ...\n"
                  "        <kilobot-led implementation=\"default\" />\n"
                  "        ...\n"
                  "      </actuators>\n"
                  "      ...\n"
                  "    </my_controller>\n"
                  "    ...\n"
                  "  </controllers>\n\n"
                  "OPTIONAL XML CONFIGURATION\n\n"
                  "None.\n",
                  "Usable"
   );

