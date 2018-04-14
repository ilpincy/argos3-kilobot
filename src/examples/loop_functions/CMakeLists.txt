# If Qt+OpenGL dependencies were found, descend into these directories
if(ARGOS_COMPILE_QTOPENGL)
  add_subdirectory(trajectory_loop_functions)
endif(ARGOS_COMPILE_QTOPENGL)
