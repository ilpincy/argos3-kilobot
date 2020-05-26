if(ARGOS_BUILD_FOR_SIMULATOR)
  include_directories(${CMAKE_SOURCE_DIR}/plugins/robots/kilobot/control_interface)
  include_directories(${CMAKE_SOURCE_DIR}/../extern/SwarmTalk/stable_headers/Kilobot)

  add_executable(edge_following_seed ${CMAKE_SOURCE_DIR}/../extern/SwarmTalk/benchmark/edge_following/seed.cpp)
  target_link_libraries(edge_following_seed argos3plugin_simulator_kilolib)
  
  add_executable(edge_following_follower ${CMAKE_SOURCE_DIR}/../extern/SwarmTalk/benchmark/edge_following/follower.cpp)
  target_link_libraries(edge_following_follower argos3plugin_simulator_kilolib)
  
endif(ARGOS_BUILD_FOR_SIMULATOR)
