cmake_minimum_required(VERSION 2.8)
# Project Name
PROJECT(CUBE)

#########################################################
# Include Files
#########################################################
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

add_executable(cube main.cpp globals.h globals.cpp file.h file.cpp vao.h vao.cpp shader.h shader.cpp orbitallight.h orbitallight.cpp camera.h camera.cpp)

########################################################
# Linking & stuff
#########################################################

# create the program “cube”
target_link_libraries(cube ${OPENGL_LIBRARIES} ${GLEW_LIBRARY} ${GLFW_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES})

if(CMAKE_GENERATOR STREQUAL Xcode)
  set_target_properties( cube PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
  RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)
endif()
