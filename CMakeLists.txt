cmake_minimum_required(VERSION 2.6)
message(STATUS "[  ----     Software Development Kit for Sputnik robot     ----  ]")
project(SputnikSDK)
#
option(EXAMPLES_ENABLE "Enable examples" OFF)
if(EXAMPLES_ENABLE STREQUAL OFF)
  message(STATUS "[ to enable examples run with parameter: -D EXAMPLES_ENABLE=ON   ]")
else(EXAMPLES_ENABLE STREQUAL OFF)
  message(STATUS "[ to disable examples run with parameter: -D EXAMPLES_ENABLE=OFF ]")
endif(EXAMPLES_ENABLE STREQUAL OFF)
#
option(DOC_ENABLE "Enable documentation" OFF)
if(DOC_ENABLE STREQUAL OFF)
  message(STATUS "[ to enable documentation run with parameter: -D DOC_ENABLE=ON   ]")
else(DOC_ENABLE STREQUAL OFF)
  message(STATUS "[ to disable examples run with parameter: -D DOC_ENABLE=OFF      ]")
endif(DOC_ENABLE STREQUAL OFF)

option(PLAYER_ENABLE "Enable Player" OFF)
if(PLAYER_ENABLE STREQUAL OFF)
  message(STATUS "[ to enable Player module and client: -D PLAYER_ENABLE=ON   ]")
else(PLAYER_ENABLE STREQUAL OFF)
  message(STATUS "[ to disable Player module and client: -D PLAYER_ENABLE=OFF ]")
endif(PLAYER_ENABLE STREQUAL OFF)



# where the SputnikSDK library is, value set when checking this library
set(LIBSPUTNIKSDK_DIR ${PROJECT_SOURCE_DIR}/src/libSputnikSDK)
# prepare all libraries and modules
add_subdirectory(src)
#
# prepare examples
if(EXAMPLES_ENABLE STREQUAL ON)
  add_subdirectory(examples)
endif(EXAMPLES_ENABLE STREQUAL ON)

if(PLAYER_ENABLE STREQUAL ON)
  add_subdirectory(player)
  execute_process(COMMAND cp -r ${CMAKE_SOURCE_DIR}/player/driver/sputnik.cfg  ${CMAKE_BINARY_DIR}/player/driver/)
endif(PLAYER_ENABLE STREQUAL ON)

set(DOC_CONFIG_FILE ${PROJECT_SOURCE_DIR}/sputnik_dox)
if(DOC_ENABLE STREQUAL ON)
  include(FindDoxygen)
  if(DOXYGEN_FOUND)
    execute_process(COMMAND ${DOXYGEN_EXECUTABLE} ${DOC_CONFIG_FILE} RESULT_VARIABLE DOC_RESULT
      OUTPUT_VARIABLE DOC_OUTPUT
      ERROR_QUIET 
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_STRIP_TRAILING_WHITESPACE
      )
    if(DOC_RESULT)
      message("DOC_OUTPUT=${DOC_OUTPUT}")
    endif(DOC_RESULT)
  else(DOXYGEN_FOUND)
    message(FATAL_ERROR "Can't find Doxygen!")
  endif(DOXYGEN_FOUND)
endif(DOC_ENABLE STREQUAL ON)
