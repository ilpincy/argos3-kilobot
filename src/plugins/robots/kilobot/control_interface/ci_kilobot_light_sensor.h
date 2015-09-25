/**
 * @file <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_light_sensor.h>
 *
 * @brief This file provides the definition of the kilobot light sensor.
 *
 * This file provides the definition of the kilobot light sensor.
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Vito Trianni - <vito.trianni@istc.cnr.it>
 */

#ifndef CCI_KILOBOT_LIGHT_SENSOR_H
#define CCI_KILOBOT_LIGHT_SENSOR_H

namespace argos {
   class CCI_KilobotLightSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/angles.h>
#include <vector>

namespace argos {

   class CCI_KilobotLightSensor : public CCI_Sensor {

   public:

      /**
       * The DTO of the light sensor, containing the sensor reading.
       */
      struct SReading {
         SInt16 Value;

         SReading() :
            Value(-1) {}

         SReading(SInt16 n_value) :
            Value(n_value) {}
      };

   public:

      CCI_KilobotLightSensor();
      virtual ~CCI_KilobotLightSensor() {}

      /**
       * Returns the readings of this sensor
       */
      inline const SReading& GetReading() const {
         return m_tReading;
      }

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:

      SReading m_tReading;
   };

   std::ostream& operator<<(std::ostream& c_os, const CCI_KilobotLightSensor::SReading& s_reading);

}

#endif
