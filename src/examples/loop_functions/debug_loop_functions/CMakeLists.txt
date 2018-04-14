add_library(debug_loop_functions MODULE 
  debug_loop_functions.h
  debug_loop_functions.cpp)

target_link_libraries(debug_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_kilobot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
