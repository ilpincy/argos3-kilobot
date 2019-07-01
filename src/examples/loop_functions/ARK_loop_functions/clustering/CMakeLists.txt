include_directories(argos3/plugins/robot/kilobot/control_interface argos3/plugins/robot/kilobot/simulator)

add_library(ALF_clustering_loop_function MODULE Clustring_ALF.h Clustring_ALF.cpp)

target_link_libraries(ALF_clustering_loop_function
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_media
  argos3plugin_simulator_kilobot
  argos3plugin_simulator_kilolib
)
