if(ARGOS_BUILD_FOR_SIMULATOR)
  include_directories(${CMAKE_SOURCE_DIR}/plugins/robots/kilobot/control_interface)
  include_directories(${CMAKE_SOURCE_DIR}/../extern/SwarmTalk/stable_headers/Kilobot)

  add_executable(hop_count_seed_1 ${CMAKE_SOURCE_DIR}/../extern/SwarmTalk/benchmark/hop_count/seed_1.cpp)
  target_link_libraries(hop_count_seed_1 argos3plugin_simulator_kilolib)
  
  add_executable(hop_count_seed_2 ${CMAKE_SOURCE_DIR}/../extern/SwarmTalk/benchmark/hop_count/seed_2.cpp)
  target_link_libraries(hop_count_seed_2 argos3plugin_simulator_kilolib)

  add_executable(hop_count_follower ${CMAKE_SOURCE_DIR}/../extern/SwarmTalk/benchmark/hop_count/follower.cpp)
  target_link_libraries(hop_count_follower argos3plugin_simulator_kilolib)
  
endif(ARGOS_BUILD_FOR_SIMULATOR)
