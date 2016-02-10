/* Include the controller definition */
#include "kilobot_phototaxis.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <vector>
#include <algorithm>

/****************************************/
/****************************************/

#define PIN_FORWARD 1.0f
#define PIN_TURN    1.57f
#define PIN_STOP    0.0f

CKilobotPhototaxis::CKilobotPhototaxis() :
   m_pcMotors(NULL),
   m_pcLightSensor(NULL),
   m_tCurrentState(KILOBOT_STATE_STOP),
   m_tPreviousState(KILOBOT_STATE_STOP),
   m_unMaxMotionSteps(50),
   m_unCountMotionSteps(0),
   m_unMaxTurningSteps(50), // = pi/(omega delta_t) = pi/(v*delta_t/l) = (pi*l)/(v*delta_t)
   m_unCountTurningSteps(100),
   m_fMotorL(0.0f),
   m_fMotorR(0.0f)
{
   m_pcRNG = CRandom::CreateRNG( "argos" );
}

/****************************************/
/****************************************/

void CKilobotPhototaxis::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the
    * XML tag of the device whose handle we want to have. For a list of
    * allowed values, type at the command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors
    * internally, on the basis of the lists provided the configuration
    * file at the <controllers><kilobot_phototaxis><actuators> and
    * <controllers><kilobot_phototaxis><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   // Get sensor/actuator handles
   m_pcMotors      = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcLightSensor = GetSensor<CCI_KilobotLightSensor>("kilobot_light");
   
   // Parse the configuration file
   GetNodeAttributeOrDefault(t_node, "max_motion_steps", m_unMaxMotionSteps, m_unMaxMotionSteps );
   if( m_unMaxMotionSteps == 0 ) {
      LOGERR << "[FATAL] Invalid value for num_moving_steps (" << m_unMaxMotionSteps << "). Should be a positive integer." << std::endl;
   }

   Reset();
}

/****************************************/
/****************************************/

void CKilobotPhototaxis::Reset() {
   // reset/intialise the robot state
   m_unCountMotionSteps = m_pcRNG->Uniform(CRange<UInt32>(1,m_unMaxMotionSteps+1));
   m_tCurrentState = KILOBOT_STATE_MOVING;
   m_tPreviousState = KILOBOT_STATE_MOVING;
   m_fMotorL = m_fMotorR = PIN_FORWARD;
}

/****************************************/
/****************************************/

void CKilobotPhototaxis::ControlStep() {
   SInt16 nReading = m_pcLightSensor->GetReading();
   if( nReading > 0 ) {
      LOG << nReading << std::endl;
   }
   m_pcMotors->SetLinearVelocity( PIN_TURN, PIN_STOP );
   return;
   
   // compute the robot motion: move forward for a fixed amount of
   // time, and rotate cw/ccw for a random amount of time
   // max rotation: 180 degrees as determined by m_unMaxTurningSteps
   m_tPreviousState = m_tCurrentState;
   switch(m_tCurrentState) {
   case KILOBOT_STATE_TURNING:
      if( --m_unCountTurningSteps == 0 ) {
         m_fMotorL = m_fMotorR = PIN_FORWARD;
         m_unCountMotionSteps = m_unMaxMotionSteps;
         m_tCurrentState = KILOBOT_STATE_MOVING;
      }
      break;

   case KILOBOT_STATE_MOVING:
      if( --m_unCountMotionSteps == 0 ) {
         UInt32 direction = m_pcRNG->Uniform(CRange<UInt32>(0,2));
         if( direction == 0 ) {
            m_fMotorL = PIN_TURN;
            m_fMotorR = PIN_STOP;
         }
         else {
            m_fMotorL = PIN_STOP;
            m_fMotorR = PIN_TURN;
         }
         m_unCountTurningSteps = m_pcRNG->Uniform(CRange<UInt32>(1,m_unMaxTurningSteps));
         m_tCurrentState = KILOBOT_STATE_TURNING;
      }
      break;

   case KILOBOT_STATE_STOP:
   default:
      m_fMotorL = m_fMotorR = PIN_STOP;
      break;
   };

   m_pcMotors->SetLinearVelocity(m_fMotorL, m_fMotorR);
}

/****************************************/
/****************************************/


/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.  The string is then usable in the configuration
 * file to refer to this controller.  When ARGoS reads that string in
 * the configuration file, it knows which controller class to
 * instantiate.  See also the configuration files for an example of
 * how this is used.
 */
REGISTER_CONTROLLER(CKilobotPhototaxis, "kilobot_phototaxis_controller")
