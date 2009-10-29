include_directories(include)
file(GLOB SputnikSDK_SRCS  ${LIBSPUTNIKSDK_DIR}/src/*.cpp)
add_library (SputnikSDK
  ${SputnikSDK_SRCS}
)