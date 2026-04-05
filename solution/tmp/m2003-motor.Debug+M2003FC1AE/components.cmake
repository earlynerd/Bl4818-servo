# components.cmake

# component ARM::CMSIS:CORE@6.2.0
add_library(ARM_CMSIS_CORE_6_2_0 INTERFACE)
target_include_directories(ARM_CMSIS_CORE_6_2_0 INTERFACE
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  "${CMSIS_PACK_ROOT}/ARM/CMSIS/6.3.0/CMSIS/Core/Include"
)
target_compile_definitions(ARM_CMSIS_CORE_6_2_0 INTERFACE
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_link_libraries(ARM_CMSIS_CORE_6_2_0 INTERFACE
  ${CONTEXT}_ABSTRACTIONS
)

# component Nuvoton::Device:Startup@0.00.001
add_library(Nuvoton_Device_Startup_0_00_001 OBJECT
  "${SOLUTION_ROOT}/m2003-motor/RTE/Device/M2003FC1AE/startup_M2003.s"
  "${SOLUTION_ROOT}/m2003-motor/RTE/Device/M2003FC1AE/retarget.c"
  "${SOLUTION_ROOT}/m2003-motor/RTE/Device/M2003FC1AE/system_M2003.c"
)
target_include_directories(Nuvoton_Device_Startup_0_00_001 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_INCLUDE_DIRECTORIES>
  "${CMSIS_PACK_ROOT}/Nuvoton/NuMicroM23_DFP/1.0.4/Device/M2003/Include"
  "${CMSIS_PACK_ROOT}/Nuvoton/NuMicroM23_DFP/1.0.4/Device/M2003/StdDriver/inc"
)
target_compile_definitions(Nuvoton_Device_Startup_0_00_001 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_DEFINITIONS>
)
target_compile_options(Nuvoton_Device_Startup_0_00_001 PUBLIC
  $<TARGET_PROPERTY:${CONTEXT},INTERFACE_COMPILE_OPTIONS>
)
target_link_libraries(Nuvoton_Device_Startup_0_00_001 PUBLIC
  ${CONTEXT}_ABSTRACTIONS
)
set(COMPILE_DEFINITIONS
  _RTE_
)
cbuild_set_defines(AS_ARM COMPILE_DEFINITIONS)
set_source_files_properties("${SOLUTION_ROOT}/m2003-motor/RTE/Device/M2003FC1AE/startup_M2003.s" PROPERTIES
  COMPILE_FLAGS "${COMPILE_DEFINITIONS}"
)
