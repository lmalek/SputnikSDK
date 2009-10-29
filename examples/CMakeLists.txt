include(FindPkgConfig)
pkg_check_modules(OPENCV REQUIRED opencv)
if(NOT OPENCV_FOUND)
  message(FATAL_ERROR "Can't find OpenCV!")
endif(NOT OPENCV_FOUND)

# search for allegro library - needed for testing
execute_process(COMMAND allegro-config --libs RESULT_VARIABLE ALLEGRO_NOTFOUND 
  OUTPUT_VARIABLE ALLEGRO_LIBRARIES
  ERROR_QUIET 
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_STRIP_TRAILING_WHITESPACE
)
if (ALLEGRO_NOTFOUND)
  message(FATAL_ERROR "Can't find Allegro!")
endif(ALLEGRO_NOTFOUND)

add_executable(Sputnik Sputnik.cpp)
# include directories for sputnikSDK, uyvy2rgb, OpenCV
include_directories(${LIBSPUTNIKSDK_DIR}/include ${OPENCV_INCLUDE_DIRS} )
target_link_libraries(Sputnik SputnikSDK ${OPENCV_LIBRARIES} ${ALLEGRO_LIBRARIES})


