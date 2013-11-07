cmake_minimum_required(VERSION 2.8)
# Project Name
PROJECT(CUBE)

#########################################################
# FIND GLM
#########################################################
find_path(GLM_INCLUDE_PATH glm/glm.hpp
    $ENV{GLM_HOME}
    $ENV{GLMDIR}
    /usr/include
    /usr/local/include
    /sw/include
    /opt/local/include
    DOC "The directory where glm/glm.hpp resides.")
    
if(GLM_INCLUDE_PATH)
  set(GLM_FOUND 1 CACHE STRING "Set to 1 if GLM is found, 0 otherwise")
else()
  set(GLM_FOUND 0 CACHE STRING "Set to 1 if GLM is found, 0 otherwise")
  message(WARNING "Note: an envvar GLM_HOME assists this script to locate glm.")
endif()

mark_as_advanced(GLM_FOUND)

if(NOT GLM_FOUND)
    message(ERROR " GLM not found!")
endif(NOT GLM_FOUND)
#########################################################
# FIND GLEW
#########################################################
find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})
link_directories(${GLEW_LIBRARY_DIRS})
add_definitions(${GLEW_DEFINITIONS})
if(NOT GLEW_FOUND)
    message(ERROR " GLEW not found!")
endif(NOT GLEW_FOUND)
#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)
#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-Wall -std=gnu++0x")
endif()

add_subdirectory(src)
add_subdirectory(test/test_solid)

