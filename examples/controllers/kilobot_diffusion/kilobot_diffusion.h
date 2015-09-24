/*
 * AUTHOR: Vito Trianni <vito.trianni@istc.cnr.it>
 *
 * An example diffusion controller for the kilobot.
 *
 * This controller makes the robots behave as gas particles. The
 * robots go straight until a time expires, in which case they
 * turn. The net effect is that over time the robots diffuse in the
 * environment.
 *
 * The controller uses the two motors to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/kilobot_diffusion_1.argos
 *    experiments/kilobot_diffusion_10.argos
 */

#ifndef KILOBOT_DIFFUSION_H
#define KILOBOT_DIFFUSION_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

/* Random number generator */
#include <argos3/core/utility/math/rng.h>
/* Logging functions */
#include <argos3/core/utility/logging/argos_log.h>
#include <vector>


/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

enum TStateNames {KILOBOT_STATE_STOP, KILOBOT_STATE_TURNING, KILOBOT_STATE_MOVING};


/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CKilobotDiffusion : public CCI_Controller {

public:

   /* Class constructor. */
   CKilobotDiffusion();

   /* Class destructor. */
   virtual ~CKilobotDiffusion() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><kilobot_diffusion_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the
    * Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up,
    * so the function could have been omitted. It's here just for
    * completeness.
    */
   virtual void Destroy() {}

private:

   /*
    * The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><kilobot_diffusion_controller> section.
    */

   /* counters and helper variables */
   UInt32 m_unInitStraightCount;
   UInt32 m_unInitTurnCount;
   UInt32 m_unStraightCount;
   UInt32 m_unTurnCount;

   Real m_fSpeakingProbability;
   CColor m_cCurrentColor;

   Real m_fMotorL;
   Real m_fMotorR;

   TStateNames m_tCurrentState;

   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;
   /* Pointer to the range and bearing actuator */
   CCI_RangeAndBearingActuator*  m_pcRABA;
   /* Pointer to the range and bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABS;
   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcMotors;


   /* variables for the random number generation */
   CRandom::CRNG*  m_pcRNG;

   /* List for the words generated during MNG */
    std::vector<int> m_lista;
    
    /*Last spoken word*/
    UInt16 m_detta;

};

#endif
