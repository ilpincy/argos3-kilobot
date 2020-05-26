if(ARGOS_BUILD_FOR_SIMULATOR)
  include_directories(${CMAKE_SOURCE_DIR}/plugins/robots/kilobot/control_interface)
  include_directories(${CMAKE_SOURCE_DIR}/../extern/SwarmTalk/stable_headers/Kilobot)

  add_executable(firefly ${CMAKE_SOURCE_DIR}/../extern/SwarmTalk/benchmark/firefly/firefly.cpp)
  target_link_libraries(firefly argos3plugin_simulator_kilolib)

endif(ARGOS_BUILD_FOR_SIMULATOR)
