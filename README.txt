# Compiling the code

Make sure you have ARGoS >= 3.0.0-beta40 installed!

Commands:

 $ mkdir build
 $ cd build
 $ cmake -DCMAKE_BUILD_TYPE=Release
 $ make

# Running the examples

The examples work if you run them from the base directory
(argos3-kilobot).

To run the examples, set the ARGOS_PLUGIN_PATH variable as follows:

 $ export ARGOS_PLUGIN_PATH=/PATH/TO/argos3-kilobot/build/plugins/robots/kilobot

Change /PATH/TO with the full path to the argos3-kilobot directory.

 # Lab 0
 $ argos3 -c src/examples/experiments/kilobot_blinky.argos

 # Lab 1.2
 $ argos3 -c src/examples/experiments/kilobot_simple_movement.argos

 # Lab 1.3
 $ argos3 -c src/examples/experiments/kilobot_nonblocked_movement.argos

 # Lab 2.1-2.2
 $ argos3 -c src/examples/experiments/kilobot_speaker_listener.argos
 
 # Lab 2.3-2.4
 $ argos3 -c src/examples/experiments/kilobot_speaker_listener_mod.argos

 # Lab 3
 $ argos3 -c src/examples/experiments/kilobot_disperse.argos

 # Lab 4
 $ argos3 -c src/examples/experiments/kilobot_orbit.argos

 # Lab 5
 $ argos3 -c src/examples/experiments/kilobot_move_to_light.argos

 # Lab 6
 $ argos3 -c src/examples/experiments/kilobot_gradient_simple.argos

 # Lab 7
 $ argos3 -c src/examples/experiments/kilobot_sync.argos

# Differences between Kilombo and ARGoS

* Kilombo
  * Architecture
    * Single-thread, single process wrapper around kilolib.h
      * Robots must run the same behavior
      * Global variables cannot be used to contain state
  * Models
    * Only model offered is the Kilobot
    * Motion is kinematics with simple overlap resolution
      * Robots cannot push other objects
    * Communication neglects obstructions
    * Message drop has uniform probability

* ARGoS
  * Architecture
    * Multi-thread, multi-process architecture
    * Robots can run different behaviors
    * Global variables can be used to contain state
  * Models
    * Models of Kilobot, other robots, boxes, cylinders
      * Motion is full 2D dynamics
      * Robots can push other objects
    * Communication considers obstruction
    * Message drop considers local density
