set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

aux_source_directory(. ROCKPIAPI)
aux_source_directory("_thirdparty/" RPIFlight3rd)
aux_source_directory("_thirdparty/PCA9685/." HardwareGenerator)
aux_source_directory("_thirdparty/RockPiMPU/src" RockPiMPU)

find_package(Threads REQUIRED)

add_library(ROCKPIAPI  ${ROCKPIAPI} ${RPIFlight3rd}  ${HardwareGenerator} ${RockPiMPU})
target_link_libraries(ROCKPIAPI PRIVATE Threads::Threads)