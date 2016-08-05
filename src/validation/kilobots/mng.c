/*
  This file contains the implementation code of the Naming Game for
  the kilobot robotics platform. It is implemented using the kilobot
  API from Kilobotics (https://www.kilobotics.com), and requires the
  libraries that can be downloaded from the same website.
  
  This software must be compiled with AVR-GCC, which can be installed
  with AVRDUDE, an open-source command line utility program to
  manipulate the ROM and EEPROM memory of AVR controllers. Additional
  information can be found here:
  https://www.kilobotics.com/documentation
*/

#include "kilolib.h"

/* Enum for different motion types */
typedef enum {
   STOP = 0,
   FORWARD,
   TURN_LEFT,
   TURN_RIGHT,
} motion_t;

/* Enum for boolean flags */
typedef enum {
   false = 0,
   true = 1,
} bool;

/* A list of pre-defined words ready for message sending */ 
const uint8_t num_words = 128;
message_t words[128];

/* Flag for successful message sent */
bool message_sent = false;

/* Global variable for the word chosen for transmission */
uint8_t w_index = 0;

/* Flag for decision to broadcast a word */ 
bool broadcast_word = false;

/* Inventory of known words */
uint8_t inventory[32];         /* a maximum inventory size must be given here. 32 is normally sufficient, but larger values can be provided as well */
uint8_t inventory_size = 0;

/* current motion type */
motion_t current_motion_type = STOP;

/* counters for motion, turning and broadcasting */
unsigned int turning_ticks = 0;
const uint8_t max_turning_ticks = 160; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
const uint8_t max_straight_ticks = 80; /* set the \tau_m period to 2.5 s: n_m = \tau_m/\delta_t = 2.5/(1/32) */
const uint8_t max_broadcast_ticks = 80;  /* set the \tau_s period to 2.5 s: n_s = \tau_s/\delta_t = 2.5/(1/32) */
uint32_t last_motion_ticks = 0;
uint32_t last_broadcast_ticks = 0;


/*-------------------------------------------------------------------*/
/* Callback function for message transmission                        */
/*-------------------------------------------------------------------*/
message_t *message_tx() {
   if( broadcast_word ) {
      return &words[w_index];
   }
   return 0;
}


/*-------------------------------------------------------------------*/
/* Callback function for successful transmission                     */
/*-------------------------------------------------------------------*/
void tx_message_success() {
   set_color(RGB(0,0,0));
   broadcast_word = false;
}


/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void message_rx( message_t *msg, distance_measurement_t *d ) {
   uint8_t received_word = msg->data[0];
   uint8_t cur_distance  = estimate_distance(d);
   if( cur_distance > 100 ) {
      return;
   }
   
   /* Check if the received word is within the inventory */
   int i = 0;
   bool word_known = false;
   for( i = 0; i < inventory_size; ++i ) {
      if( received_word == inventory[i] ) {
         word_known = true;
         break;
      }
   }

   /* if the word is known, remove all other words from the
      inventory, otherwise insert it */
   if( word_known ) {
      inventory[0] = received_word;
      inventory_size = 1;
   }
   else {
      inventory[inventory_size] = received_word;
      inventory_size += 1;
   }
}


/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion( motion_t new_motion_type ) {
   if( current_motion_type != new_motion_type ){
      switch( new_motion_type ) {
      case FORWARD:
         spinup_motors();	
         set_motors(kilo_straight_left,kilo_straight_right);
         break;
      case TURN_LEFT:
         spinup_motors();
         set_motors(kilo_turn_left,0);
         break;
      case TURN_RIGHT:
         spinup_motors();
         set_motors(0,kilo_turn_right);
         break;
      case STOP:
      default:
         set_motors(0,0);         
      }
      current_motion_type = new_motion_type;
   }
}


/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup() {
   /* Initialise LED and motors */
   set_color(RGB(0,0,0));
   set_motors(0,0);

   /* Initialise random seed */
   uint8_t seed = rand_hard();
   rand_seed(seed);

   /* Initialise the list of words ready for message sending. The word
      index is contained in the first payload byte, while the
      follwoing three bytes contain the randomly-chosen word */
   int i;
   for( i = 0; i < num_words; i++ ) {
      words[i].data[0] = i;
      words[i].type    = NORMAL;
      words[i].crc     = message_crc(&words[i]);
   }

   /* Initialise motion variables */
   set_motion( FORWARD );
   last_motion_ticks = rand_soft() % max_straight_ticks + 1;

   /* Initialise mng variables */
   last_broadcast_ticks = rand_soft() % max_broadcast_ticks + 1;
}


/*-------------------------------------------------------------------*/
/* Function implementing the uncorrelated random walk                */
/*-------------------------------------------------------------------*/
void random_walk(){
   switch( current_motion_type ) {
   case TURN_LEFT:
   case TURN_RIGHT:
      if( kilo_ticks > last_motion_ticks + turning_ticks ) {
         /* start moving forward */
         last_motion_ticks = kilo_ticks;
         set_motion(FORWARD);
      }
      break;
   case FORWARD:
      if( kilo_ticks > last_motion_ticks + max_straight_ticks ) {
         /* perform a radnom turn */
         last_motion_ticks = kilo_ticks;
         if( rand_soft()%2 ) {
            set_motion(TURN_LEFT);
         }
         else {
            set_motion(TURN_RIGHT);
         }
         turning_ticks = rand_soft()%max_turning_ticks + 1;
      }
      break;
   case STOP:
   default:
      set_motion(STOP);
   }
}


/*-------------------------------------------------------------------*/
/* Function to select a random word and to broadcast it              */
/*-------------------------------------------------------------------*/
void broadcast() {
   if( kilo_ticks > last_broadcast_ticks + max_broadcast_ticks ) {
      last_broadcast_ticks = kilo_ticks;
      
      /* Select a random word from the inventory, or invent a new one
         if the inventory is empty */
      if( inventory_size == 0 ) {
         w_index = rand_soft()%num_words;
         inventory[0] = words[w_index].data[0];
         inventory_size = 1;
      }
      else {
         w_index = inventory[rand_soft() % inventory_size];
      }
      
      /* set broadcast flag for transmission */
      broadcast_word = true;
   }
}


/*-------------------------------------------------------------------*/
/* Function to color-code the inventory size and the selected word   */
/*-------------------------------------------------------------------*/
void color_code() {
   if( inventory_size != 1 ) {
      set_color(RGB(0,0,0));
      return;
   }

   /* Convert the number in base 4 to turn on the LED */
   uint8_t color = inventory[0]%63 + 1;
   uint8_t r     = color/16;
   uint8_t rem1  = color%16;
   uint8_t g     = rem1/4;
   uint8_t b     = rem1%4;
   set_color(RGB(r,g,b));
}


/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop() {
   random_walk();
   broadcast();
   color_code();
}


/*-------------------------------------------------------------------*/
/* Main function                                                     */
/*-------------------------------------------------------------------*/
int main() {
   kilo_init();
   
   /* Communication callbacks */
   kilo_message_tx=message_tx;
   kilo_message_tx_success = tx_message_success;
   kilo_message_rx=message_rx;
   
   /* start main loop */
   kilo_start(setup, loop);

   return 0;
}
