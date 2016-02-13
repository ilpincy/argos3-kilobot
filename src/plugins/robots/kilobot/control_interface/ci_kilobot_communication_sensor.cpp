/**
 * @file <argos3/plugins/robots/foot-bot/control_interface/ci_kilobot_communication_sensor.cpp>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#include "ci_kilobot_communication_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

   CCI_KilobotCommunicationSensor::CCI_KilobotCommunicationSensor() {
   }

   /****************************************/
   /****************************************/

   const CCI_KilobotCommunicationSensor::TReadings& CCI_KilobotCommunicationSensor::GetMessages() const {
      return m_vecMessages;
   }

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_KilobotCommunicationSensor::CreateLuaState(lua_State* pt_lua_state) {
      // TODO
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_KilobotCommunicationSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      // TODO
   }
#endif


   /****************************************/
   /****************************************/

}
