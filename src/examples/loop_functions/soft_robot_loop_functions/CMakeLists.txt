set(soft_robot_loop_functions_SOURCES
  soft_robot_loop_functions.h
  soft_robot_loop_functions.cpp
  soft_robot_qtuser_functions.h
  soft_robot_qtuser_functions.cpp)

add_library(soft_robot_loop_functions MODULE
  ${soft_robot_loop_functions_SOURCES})
target_link_libraries(soft_robot_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_kilobot
  argos3plugin_simulator_qtopengl
  ${ARGOS_QTOPENGL_LIBRARIES})
