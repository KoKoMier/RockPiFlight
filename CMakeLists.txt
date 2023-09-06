set(MPUDriver "spidev" CACHE  INTERNAL "")

#CMake 最低版本要求
cmake_minimum_required (VERSION 2.8)

#项目信息
project (Demo)

set(src_dir main.cpp )
add_executable(flight ${src_dir})

