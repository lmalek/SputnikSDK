include(FindPkgConfig)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(PLAYERCORE playercore)
else(PKG_CONFIG_FOUND)
  message("FindPkgConfig not found!")
endif(PKG_CONFIG_FOUND)

file(GLOB SputnikSDK_SRCS  ${LIBSPUTNIKSDK_DIR}/src/*.cpp)
include_directories(${PLAYERCORE_INCLUDE_DIRS} ${LIBSPUTNIKSDK_DIR}/include  )

add_library(drrobot SHARED ${SputnikSDK_SRCS} drrobot.cpp)


