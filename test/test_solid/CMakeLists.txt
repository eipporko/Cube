cmake_minimum_required(VERSION 2.8)
# Project Name
PROJECT(CUBE)

#########################################################
# Include Files
#########################################################
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

add_executable(test_solid test_solid.cpp ../../include/glm/glm.hpp)

########################################################
# Linking & stuff
#########################################################

# create the program “test_solid”
target_link_libraries(test_solid ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} ${GLEW_LIBRARY})

if(CMAKE_GENERATOR STREQUAL Xcode)
  set_target_properties( test_solid PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
  RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)
endif()
