
/**
 * @file <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_differential_steering_actuator.h>
 *
 * @author Carlo Pinciroli <ilpincy@gmail.com>
 */
#ifndef CCI_KILOBOT_DIFFERENTIAL_STEERING_ACTUATOR_H
#define CCI_KILOBOT_DIFFERENTIAL_STEERING_ACTUATOR_H

/* To avoid dependency problems when including */
namespace argos {
   class CCI_KilobotDifferentialSteeringActuator;
}

#include <argos3/core/control_interface/ci_actuator.h>

namespace argos {

   class CCI_KilobotDifferentialSteeringActuator : public CCI_Actuator {

   public:

      virtual ~CCI_KilobotDifferentialSteeringActuator() {}

      virtual void SetLinearVelocity(Real f_left_velocity,
                                     Real f_right_velocity) = 0;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);
#endif

   protected:

      Real m_fCurrentVelocity[2];

   };

}

#endif
