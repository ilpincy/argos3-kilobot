/**
 * @file <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_led_actuator.h>
 *
 * @author Carlo Pinciroli <ilpincy@gmail.com>
 */

#ifndef CCI_KILOBOT_LED_ACTUATOR_H
#define CCI_KILOBOT_LED_ACTUATOR_H

namespace argos {
   class CCI_KilobotLedActuator;
}

#include <argos3/core/control_interface/ci_actuator.h>
#include <argos3/core/utility/datatypes/color.h>

namespace argos {

   class CCI_KilobotLEDActuator : public CCI_Actuator {

   public:

      CCI_KilobotLEDActuator() {}

      virtual ~CCI_KilobotLEDActuator() {}

      /**
       * @brief Sets the color of the LED.
       */
      virtual void SetColor(const CColor& c_color);
      
#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state) {}
#endif

   protected:

      CColor m_cColor;

   };

}

#endif
