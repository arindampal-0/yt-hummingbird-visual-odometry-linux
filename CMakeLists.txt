cmake_minimum_required( VERSION 3.12 )
project( Mono_VO )

find_package( OpenCV 4.0 REQUIRED )

set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
set( CMAKE_BUILD_TYPE DEBUG )

include_directories( include ${OpenCV_INCLUDE_DIRS} )

add_library( visual_odometry SHARED src/vo_utils.cpp src/vo.cpp )

add_executable( vo src/main.cpp )
target_link_libraries( vo visual_odometry ${OpenCV_LIBS} )