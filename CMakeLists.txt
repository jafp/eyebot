
cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_COMPILER   arm-rpi-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-rpi-linux-gnueabihf-g++)

project(eyecam)
link_libraries(pthread m ${CMAKE_SOURCE_DIR}/confuse-2.7/src/.libs/libconfuse.a )
include_directories( ${CMAKE_SOURCE_DIR}/confuse-2.7/src  )
add_executable(eyecam configuration.c log.c i2c.c broadcast.c motor_ctrl.c camera.c main.c)
#target_link_libraries( ${CMAKE_SOURCE_DIR}/confuse-2.7/src/.libs/libconfuse.a )
