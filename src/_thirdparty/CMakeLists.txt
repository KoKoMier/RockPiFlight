aux_source_directory("PCA9685/." HardwareGenerator)
aux_source_directory("RockPiMPU/src" RockPiMPU)
aux_source_directory(. RPIFlight3rd)


add_library(ROCKPIFLIGHT ${HardwareGenerator} ${RockPiMPU} ${RPIFlight3rd})
target_link_libraries(RPiMPU)