# GLM_FOUND
# GLM_INCLUDE_PATH

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