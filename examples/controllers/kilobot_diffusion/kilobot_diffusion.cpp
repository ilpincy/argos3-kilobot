/* Include the controller definition */
#include "kilobot_diffusion.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <vector>
#include <algorithm>

/****************************************/
/****************************************/

#define PIN_FORWARD 1.0f;
#define PIN_STOP    0.0f;

CKilobotDiffusion::CKilobotDiffusion() :
   m_unInitStraightCount(100),
   m_unInitTurnCount(100),
   m_unStraightCount(100),
   m_unTurnCount(100),
   m_fSpeakingProbability(0.1),
   m_cCurrentColor(CColor::BLACK),
   m_fMotorL(0.0f),
   m_fMotorR(0.0f),
   m_tCurrentState(KILOBOT_STATE_STOP),
   m_pcLEDs(NULL),
   m_pcRABA(NULL),
   m_pcRABS(NULL),
   m_pcMotors(NULL),
   m_lista(0),
   m_detta(0)
{
   m_pcRNG = CRandom::CreateRNG( "argos" );
}

/****************************************/
/****************************************/

void CKilobotDiffusion::Init(TConfigurationNode& t_node) {
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
    * file at the <controllers><kilobot_diffusion><actuators> and
    * <controllers><kilobot_diffusion><sensors> sections. If you forgot to
    * list a device in the XML and then you request it here, an error
    * occurs.
    */
   m_pcLEDs = GetActuator<CCI_LEDsActuator>("leds");
   // m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
   // m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
   m_pcMotors    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");

   /*
    * Parse the configuration file
    *
    * The user defines this part. Here, the algorithm accepts three
    * parameters and it's nice to put them in the config file so we don't
    * have to recompile if we want to try other settings.
    */
   GetNodeAttributeOrDefault(t_node, "straight_count", m_unInitStraightCount, m_unInitStraightCount );
   GetNodeAttributeOrDefault(t_node, "turn_count", m_unInitTurnCount, m_unInitTurnCount );
   GetNodeAttributeOrDefault(t_node, "speaking_probability", m_fSpeakingProbability, m_fSpeakingProbability );

   m_unStraightCount = m_unInitStraightCount;
   m_tCurrentState = KILOBOT_STATE_MOVING;
   m_fMotorL = m_fMotorR = PIN_FORWARD;
    
}

/****************************************/
/****************************************/

void CKilobotDiffusion::Reset() {
    
    // m_pcRABA->ClearData();
    m_lista.clear();
    m_detta=0;

}

/****************************************/
/****************************************/

void CKilobotDiffusion::ControlStep() {
   switch(m_tCurrentState) {
   case KILOBOT_STATE_TURNING:
      if( --m_unTurnCount == 0 ) {
         m_fMotorL = m_fMotorR = PIN_FORWARD;
         m_unStraightCount = m_unInitStraightCount;
         m_tCurrentState = KILOBOT_STATE_MOVING;
      }
      break;

   case KILOBOT_STATE_MOVING:
      if( --m_unStraightCount == 0 ) {
         UInt32 direction = m_pcRNG->Uniform(CRange<UInt32>(0,2));
         if( direction == 0 ) {
            m_fMotorL = PIN_FORWARD;
            m_fMotorR = PIN_STOP;
         }
         else {
            m_fMotorL = PIN_STOP;
            m_fMotorR = PIN_FORWARD;
         }
         m_unTurnCount = m_pcRNG->Uniform(CRange<UInt32>(1,m_unInitTurnCount));
         m_tCurrentState = KILOBOT_STATE_TURNING;
      }
      break;

   case KILOBOT_STATE_STOP:
   default:
      m_fMotorL = m_fMotorR = PIN_STOP;
      break;
   };
    
   m_pcMotors->SetLinearVelocity(m_fMotorL, m_fMotorR);

    /*Speaking function for MNG*/
    if( m_pcRNG->Uniform(CRange<Real>(0,1)) < m_fSpeakingProbability ) {
        if(m_lista.size()==0){
            m_detta = m_pcRNG->Uniform(CRange<UInt32>(0,UINT16_MAX));
            m_lista.push_back(m_detta);
            UInt8 first = floor(m_detta/255);
            UInt8 second = m_detta%255;
            // m_pcRABA->SetData(0, first);
            // m_pcRABA->SetData(1, second);
            // m_pcRABA->SetData(2, 255);
        }
        else {
            UInt16 index = m_pcRNG->Uniform(CRange<UInt32>(0,m_lista.size()));
            m_detta= m_lista[index];
            UInt8 first = floor(m_detta/255);
            UInt8 second = m_detta%255;
            // m_pcRABA->SetData(0, first);
            // m_pcRABA->SetData(1, second);
            // m_pcRABA->SetData(2, 255);
        }
    }
    /*Listening function for MNG*/
    // const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
    // std::vector<UInt16> listened;
    // for (UInt32 i=0; i<tPackets.size(); ++i) {
    //     UInt8 firstread = tPackets[i].Data[0];
    //     UInt8 secondread = tPackets[i].Data[1];
    //     UInt8 flag = tPackets[i].Data[2];
    //     if (flag==255){
    //         UInt16 parola = (firstread*255)+secondread;
    //         listened.push_back(parola);
    //     }
    // }
    
    // if(listened.size()>0){
    //     UInt16 indexlistened = m_pcRNG->Uniform(CRange<UInt32>(0,listened.size()));
    //     UInt16 ascoltata = listened[indexlistened];
    //     if(std::find(m_lista.begin(), m_lista.end(), ascoltata) != m_lista.end()) {
    //         m_lista.clear();
    //         m_lista.push_back(ascoltata);
    //     }
    //     else {
    //         m_lista.push_back(ascoltata);
    //     }
    // }
    
    
                                           
    /*Convert the number in base 4 to turn on the LED*/
    UInt8 colore = m_detta%64;
    UInt8 r=floor(colore/16)*64;
    UInt8 rem1=colore%16;
    UInt8 g=floor(rem1/4)*64;
    UInt8 b=(colore%4)*64;
    m_cCurrentColor.Set(r,g,b);
    m_pcLEDs->SetAllColors(m_cCurrentColor);
}

/****************************************/
/****************************************/


/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the configuration file to refer to this
 * controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CKilobotDiffusion, "kilobot_diffusion_controller")
