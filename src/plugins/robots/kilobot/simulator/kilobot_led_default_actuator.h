/**
 * @file <argos3/plugins/robots/kilobot/simulator/kilobot_led_default_actuator.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef KILOBOT_LED_DEFAULT_ACTUATOR_H
#define KILOBOT_LED_DEFAULT_ACTUATOR_H

#include <string>
#include <map>

namespace argos {
   class CKilobotLEDsDefaultActuator;
}

#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_led_actuator.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/core/simulator/actuator.h>

namespace argos {

   class CKilobotLEDDefaultActuator : public CSimulatedActuator,
                                      public CCI_KilobotLEDActuator {

   public:

      CKilobotLEDDefaultActuator();

      virtual ~CKilobotLEDDefaultActuator() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Update();
      virtual void Reset();
      virtual void Destroy();

   private:

      CLEDEquippedEntity* m_pcLEDEquippedEntity;

   };

}

#endif
