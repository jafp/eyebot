
cmake_minimum_required(VERSION 2.8)

project(eyecam)
link_libraries(pthread)
add_executable(eyecam i2c.c broadcast.c motor_ctrl.c cam.c main.c)
