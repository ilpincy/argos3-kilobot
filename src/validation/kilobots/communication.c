/*
  This file contains the implementation code of the validation
  behaviour for communication.

  We put 25 kilbots on a regular grid of 20x20cm (5cm distance between
  the centre of two adjacent kilobots). Each kilobot must be assigned
  an identification (ID) number from 0 to 24.  Kilobots attempt to
  transmit a message containing their own ID to their neighbours every
  TAU seconds, and record the number of messages successfully
  transmitted. The number of received messages is also stored, with a
  different counter for each different ID. In this way, we obtain data on:
  1. average transmission rate, and variability with the position (center vs periphery)
  2. average reception rate, and variability with the position of the receiver
  3. percentage of received messages

*/

#include "kilolib.h"
#define DEBUG
#include "debug.h"

// #define USE_LEDS

/* Enum for boolean flags */
typedef enum {
   false = 0,
   true = 1,
} bool;

/* A pre-defined message with the kilobot ID ready for transmission */ 
message_t message_id;

/* Flag for decision to broadcast a message */ 
bool broadcast = false;

/* time limit */
const uint32_t max_ticks = 150*32;  /* max allotted time: T seconds (i.e., T*32 kilotiks) */
uint32_t init_ticks = 0;  /* value of the kilo_ticks at experiment start (after initialisation) */

/* counters for broadcasting */
const uint32_t max_broadcast_ticks = 0;  /* set the broadcast period to TAU seconds (i.e., TAU*32 kiloticks) */
uint32_t last_broadcast_ticks = 0;

/* counter for printing */
const uint32_t max_print_ticks = 3*32;  /* set the results printing period to Y seconds (i.e., Y*32 kiloticks) */
uint32_t last_print_ticks = 0;


/* structures to store results */
#define num_robots 25
uint16_t count_received_messages[num_robots];
uint16_t count_sent_messages;


bool red_not_blue = true;

/*-------------------------------------------------------------------*/
/* Callback function for message transmission                        */
/*-------------------------------------------------------------------*/
message_t *message_tx() {
   if( broadcast ) {
      return &message_id;
   }
   return 0;
}


/*-------------------------------------------------------------------*/
/* Callback function for successful transmission                     */
/*-------------------------------------------------------------------*/
void tx_message_success() {
#ifdef USE_LEDS
   set_color(RGB(0,0,0));
#endif
   count_sent_messages += 1;
   broadcast = false;
}


/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void message_rx( message_t *msg, distance_measurement_t *d ) {
   uint8_t received_id = msg->data[0];
   /* uint8_t cur_distance  = estimate_distance(d); */
   count_received_messages[received_id] += 1;
}



/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup() {
   /* Initialise LEDs */
   set_color(RGB(0,0,0));

   /* Initialise random seed and wait for a random time in [0:255] milliseconds */
   uint8_t seed = rand_hard();
   rand_seed(seed);
   delay(rand_soft());
   
   /* Initialise the message to be sent */
   message_id.data[0] = (uint8_t) kilo_uid;
   message_id.type    = NORMAL;
   message_id.crc     = message_crc(&message_id);

   /* Initialise the array of counters for received messages */
   uint8_t i = 0;
   for( i = 0; i < num_robots; ++i ) {
      count_received_messages[i] = 0;
   }

   /* Initialise the variable for sent messages */
   count_sent_messages = 0;

   /* Initialise the counters  */
   if( max_broadcast_ticks != 0 ) {
      last_broadcast_ticks = rand_soft() % max_broadcast_ticks + kilo_ticks;
   }
   init_ticks = kilo_ticks;
}


/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
   /* if T seconds have elapsed, print the results every Y seconds */
   if( kilo_ticks > init_ticks + max_ticks ) {
      broadcast = false;
      if( kilo_ticks > last_print_ticks + max_print_ticks ) {
         set_color(RGB(0,3,0));
         delay(250);
         set_color(RGB(0,0,0));
         last_print_ticks = kilo_ticks;
         printf( "% 8d", kilo_uid );
         uint8_t i;
         for( i = 0; i < num_robots; ++i ) {
            if( i == kilo_uid ) {
               printf( "% 8d", count_sent_messages );
            }
            else {
               printf( "% 8d", count_received_messages[i] );
            }
         }
         printf("\n");
      }
      return;
   }

   /* Attempt to broadcast a message every TAU seconds, if previous message was sent */
   if( !broadcast && kilo_ticks > last_broadcast_ticks + max_broadcast_ticks ) {
      last_broadcast_ticks = kilo_ticks;
      broadcast = true;
#ifdef USE_LEDS
      if( red_not_blue )
         set_color(RGB(3,0,0));
      else
         set_color(RGB(0,0,3));
      red_not_blue = !red_not_blue;
#endif
   }
}


/*-------------------------------------------------------------------*/
/* Main function                                                     */
/*-------------------------------------------------------------------*/
int main() {
   kilo_init();
   debug_init();
   
   /* Communication callbacks */
   kilo_message_tx = message_tx;
   kilo_message_tx_success = tx_message_success;
   kilo_message_rx = message_rx;
   
   /* start main loop */
   kilo_start(setup, loop);

   return 0;
}
