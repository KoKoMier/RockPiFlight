cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
project(ROCKPIFlight)

find_package(Threads REQUIRED)
add_subdirectory(src)
add_executable(ROCKPIFlight "main.cpp")
target_compile_definitions(ROCKPIFlight PRIVATE "-DGIT_COMMIT_HASH=\"${GIT_COMMIT_HASH}\"")
target_link_libraries(ROCKPIFlight PRIVATE gcc_pic) # this option must link with musl
target_link_libraries(ROCKPIFlight PRIVATE ROCKPIAPI)
target_link_libraries(ROCKPIFlight PRIVATE Threads::Threads)