include(FindPkgConfig)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(PLAYERC++ playerc++)
else(PKG_CONFIG_FOUND)
  message("FindPkgConfig not found!")
endif(PKG_CONFIG_FOUND)

add_executable(playerjoy playerjoy.cpp)
include_directories(${PLAYERC++_INCLUDE_DIRS} ${LIBSPUTNIKSDK_DIR}/include  )
target_link_libraries(playerjoy SputnikSDK ${PLAYERC++_LIBRARIES})


