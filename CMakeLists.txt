
cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_COMPILER   arm-rpi-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-rpi-linux-gnueabihf-g++)

project(eyecam)
link_libraries(pthread m ${CMAKE_SOURCE_DIR}/confuse-2.7/src/.libs/libconfuse.a )
include_directories( ${CMAKE_SOURCE_DIR}/confuse-2.7/src  )

add_executable(eyecam configuration.c avg_num.c log.c i2c.c ioexp.c broadcast.c motor_ctrl.c camera.c image.c main.c)
add_executable(i2c_test i2c.c ioexp.c i2c_test.c)
add_executable(dist_test i2c.c motor_ctrl.c dist_test.c)
