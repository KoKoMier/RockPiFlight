cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
project(RockPiFlight)

find_package(Threads REQUIRED)
add_subdirectory(src)
add_executable(RockPiFlight "main.cpp")
target_compile_definitions(RockPiFlight PRIVATE "-DGIT_COMMIT_HASH=\"${GIT_COMMIT_HASH}\"")
target_link_libraries(RockPiFlight PRIVATE gcc_pic) # this option must link with musl
target_link_libraries(RockPiFlight PRIVATE ROCKPIAPI)
target_link_libraries(RockPiFlight PRIVATE Threads::Threads)