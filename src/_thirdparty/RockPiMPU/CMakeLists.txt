cmake_minimum_required (VERSION 3.8)
project (RPIMAIN)

add_subdirectory(src)

include_directories(./src/MPU6500)

add_executable(RockPiMPU "main.cpp")
target_link_libraries(RockPiMPU ${RPIMPU})

