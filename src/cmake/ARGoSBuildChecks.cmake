#
# Check ARGoS
#
find_package(ARGoS REQUIRED)
include_directories(${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})

#
# Look for librt, necessary on some platforms
#
if(NOT APPLE)
  find_package(RT)
endif(NOT APPLE)
