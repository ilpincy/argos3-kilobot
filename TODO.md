* In kilobot_communication_default_actuator: When the message is set
  to 0 (NULL pointer) the kilobot should not send any message. This is
  the way in which you can stop sending messages on the real kilobot, by
  clearing its memory.

* Make it possible to cross-compile between ARGoS and the real robot
  by integrating the real kilobot compulation within the CMake
  structure.

* Add support for LoopFunctions in ARGoS. This is important when
  running experiments in which some data stored internally in the
  controller of the kilobots should be made available. We discussed
  about the possibility to define special variables that are stored in
  shared memory.

* Add support for debug to use the same code between kilobots and real
  robots. Normally, with the debug option, kilobots can print out
  variables at every control cycle, more or less (but check the
  API). Basically, debug gives access to <stdio.h>, and is commonly
  used by using printf to print strings on the serial port, which is
  visualised on a window of the kilogui. It would be nice if we could
  send the debug messages to the ARGoS GUI, in a way or the other. As
  a first step, debug messages could just be discarded when compiling
  against ARGoS, so that the code is still compatible.

* Make it possible to have an immobile kilobot (i.e., a kilobot that
  cannot be pushed away, but serves as beacon). This is very useful in
  many experiments with real robots, which can be made immobile by
  putting them above a round paper to avoid other robots get in
  contact with them.

