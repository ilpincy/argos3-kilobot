/**
 * @file <argos3/plugins/robots/kilobot/simulator/kilobot_measures.h>
 *
 * @author Carlo Pinciroli - <ilpincy@gmail.com>
 * @author Vito Trianni - <vito.trianni@istc.cnr.it>
 */

#ifndef KILOBOT_MEASURES_H
#define KILOBOT_MEASURES_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/angles.h>


namespace argos {

   /* Kilobot measures */
   static const Real BODY_HEIGHT            = 0.0127f;
   static const Real PIN_HEIGHT             = 0.0217f;
   static const Real PIN_RADIUS             = 0.0007f;
   static const Real INTERPIN_DISTANCE      = 0.025f;
   static const Real HALF_INTERPIN_DISTANCE = INTERPIN_DISTANCE * 0.5f;
   static const Real FRONT_PIN_DISTANCE     = 0.025;
   static const Real KILOBOT_RADIUS         = 0.0165f;
   static const Real KILOBOT_ECCENTRICITY   = 0.0092;
   static const Real KILOBOT_HEIGHT         = PIN_HEIGHT + BODY_HEIGHT;
   static const Real KILOBOT_MASS           = 0.01f;

   /* RGB LED */
   static const Real PIN_WHEEL_RADIUS       = 0.001;
   static const CRadians LED_ANGLE          = CRadians(ARGOS_PI / 6.0);
   static const Real LED_ELEVATION          = KILOBOT_HEIGHT;
   static const Real LED_RADIUS             = KILOBOT_RADIUS * 0.25;
   static const Real LED_HEIGHT             = KILOBOT_RADIUS * 0.05;

   /* Light Sensor */
   static const CRadians LIGHT_SENSOR_ANGLE = ToRadians(CDegrees(100));
   static const Real LIGHT_SENSOR_ELEVATION = KILOBOT_HEIGHT;
   static const Real LIGHT_SENSOR_RADIUS    = KILOBOT_RADIUS - 0.002;
   static const Real LIGHT_SENSOR_RANGE     = 0.1f;

   /* Communication RAB */
   static const Real RAB_ELEVATION          = KILOBOT_HEIGHT + 0.001f;
   


}


#endif
