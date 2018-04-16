
/**
 * @file <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_led_actuator.cpp>
 *
 * @author Carlo Pinciroli <ilpincy@gmail.com>
 */

#include "ci_kilobot_led_actuator.h"

namespace argos {

   /****************************************/
   /****************************************/

   void CCI_KilobotLEDActuator::SetColor(const CColor& c_color) {
      m_cColor = c_color;
   }
      
   /****************************************/
   /****************************************/

}
