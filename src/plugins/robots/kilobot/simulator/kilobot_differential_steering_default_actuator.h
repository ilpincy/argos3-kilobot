/**
 * @file <argos3/plugins/robots/kilobot/simulator/kilobot_differential_steering_default_actuator.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef KILOBOT_DIFFERENTIAL_STEERING_ACTUATOR_DEFAULT_H
#define KILOBOT_DIFFERENTIAL_STEERING_ACTUATOR_DEFAULT_H

#include <string>
#include <map>

namespace argos {
   class CKilobotDifferentialSteeringDefaultActuator;
}

#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_differential_steering_actuator.h>
#include <argos3/core/simulator/actuator.h>
#include <argos3/plugins/simulator/entities/wheeled_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/utility/math/rng.h>

namespace argos {

   class CKilobotDifferentialSteeringDefaultActuator : public CSimulatedActuator,
                                                       public CCI_KilobotDifferentialSteeringActuator {

   public:

      enum DIFFERENTIAL_STEERING {
         LEFT_WHEEL = 0,
         RIGHT_WHEEL = 1
      };

   public:

      /**
       * @brief Constructor.
       */
      CKilobotDifferentialSteeringDefaultActuator();

      /**
       * @brief Destructor.
       */
      virtual ~CKilobotDifferentialSteeringDefaultActuator() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      /**
       * @brief Sets the linear velocity of the two steering.
       * Velocities are expressed in cm per second.
       *
       * @param f_left_velocity desired left wheel velocity.
       * @param f_right_velocity desired right wheel velocity.
       */
      virtual void SetLinearVelocity(Real f_left_velocity,
                                     Real f_right_velocity);

      virtual void Update();

      virtual void Reset();

   protected:

      CWheeledEntity* m_pcWheeledEntity;
      
      /** Random number generator */
      CRandom::CRNG* m_pcRNG;

      /** Noise bias for each wheel */
      Real m_fNoiseBias[2];
      
      /** Noise average (Gaussian model) for each wheel  */
      Real m_fNoiseAvg[2];

      /** Noise stddev (Gaussian model) for each wheel  */
      Real m_fNoiseStdDev[2];

   };

}

#endif
