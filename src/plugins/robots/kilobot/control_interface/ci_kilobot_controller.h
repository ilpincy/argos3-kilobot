/**
 * @file <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_controller.h>
 *
 * @brief This file provides the definition of the kilobot controller.
 *
 * This file provides the definition of the kilobot controller.
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 */

#ifndef CCI_KILOBOT_CONTROLLER_H
#define CCI_KILOBOT_CONTROLLER_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/kilobot/control_interface/kilolib.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_led_actuator.h>
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_light_sensor.h>
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_communication_actuator.h>
#include <argos3/plugins/robots/kilobot/control_interface/ci_kilobot_communication_sensor.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>

using namespace argos;

class CCI_KilobotController : public CCI_Controller {

public:

   CCI_KilobotController();
   virtual ~CCI_KilobotController() {}

   virtual void Init(TConfigurationNode& t_tree);

   virtual void ControlStep();

   virtual void Reset();

   virtual void Destroy();

   int GetSharedMemFD() const {
      return m_nSharedMemFD;
   }

   pid_t GetBehaviorPID() const {
      return m_tBehaviorPID;
   }

   kilobot_state_t* GetRobotState() {
      return m_ptRobotState;
   }

   template<class S> S* DebugInfoCreate() {
      /* Open shared file */
      m_nDebugInfoFD =
         ::shm_open(
            ("/ARGoS_DEBUG_" + ToString<pid_t>(getpid()) + "_" + GetId()).c_str(),
            O_RDWR | O_CREAT,
            S_IRUSR | S_IWUSR);
      /* Check for errors */
      if(m_nDebugInfoFD < 0) {
         THROW_ARGOSEXCEPTION("Creating debug info shared memory area for " << GetId() << ": " << ::strerror(errno));
      }
      /* Resize shared memory area to contain the robot state */
      ::ftruncate(m_nDebugInfoFD, sizeof(S));
      /* Get pointer to shared memory area */
      S* ptDebugInfo = reinterpret_cast<S*>(
         ::mmap(NULL,
                sizeof(S),
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                m_nDebugInfoFD,
                0));
      if(ptDebugInfo == MAP_FAILED) {
         THROW_ARGOSEXCEPTION("Mmapping the debug info shared memory area of " << GetId() << ": " << ::strerror(errno));
      }
      ::memset(ptDebugInfo, 0, sizeof(S));
      /* Return pointer */
      return ptDebugInfo;
   }

   template<class S> void DebugInfoDestroy(S* pt_debug_info) {
      munmap(pt_debug_info, sizeof(S));
      close(m_nDebugInfoFD);
      ::shm_unlink(
         ("/ARGoS_DEBUG_" + ToString<pid_t>(getpid()) + "_" + GetId()).c_str());
   }

protected:

   virtual void CreateBehavior();

   virtual void DestroyBehavior();

private:

   /** Pointer to the shared memory area */
   kilobot_state_t* m_ptRobotState;

   /** Pointer to the motor actuator */
   CCI_DifferentialSteeringActuator* m_pcMotors;

   /** Pointer to LED actuator */
   CCI_KilobotLEDActuator* m_pcLED;

   /** Pointer to the light sensor */
   CCI_KilobotLightSensor* m_pcLight;

   /** Pointer to the communication actuator */
   CCI_KilobotCommunicationActuator* m_pcCommA;

   /** Pointer to the communication sensor */
   CCI_KilobotCommunicationSensor* m_pcCommS;

   /* The random number generator */
   CRandom::CRNG* m_pcRNG;

   /** File descriptor for shared memory area */
   int m_nSharedMemFD;

   /** File descriptor for debug info */
   int m_nDebugInfoFD;

   /** PID of the process executing the behavior */
   pid_t m_tBehaviorPID;

   /** File name of the behavior to load */
   std::string m_strBehaviorFName;

};

#endif
