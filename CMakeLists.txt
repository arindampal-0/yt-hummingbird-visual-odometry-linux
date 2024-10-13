cmake_minimum_required( VERSION 3.12 )
project( Mono_VO )

find_package( OpenCV 4.0 REQUIRED )

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

include_directories( include ${OpenCV_INCLUDE_DIRS} )

add_executable( vo src/main.cpp )
target_link_libraries( vo ${OpenCV_LIBS} )