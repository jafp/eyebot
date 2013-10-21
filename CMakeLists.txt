
cmake_minimum_required(VERSION 2.8)

set(CMAKE_C_COMPILER   arm-rpi-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-rpi-linux-gnueabihf-g++)

project(eyecam)
link_libraries(pthread m)
add_executable(eyecam log.c i2c.c broadcast.c motor_ctrl.c camera.c main.c)
