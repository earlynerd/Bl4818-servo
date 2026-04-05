cmake_minimum_required(VERSION 3.27)
include(ExternalProject)
	
project("m2003" NONE)

# Enable color diagnostics
set(CMAKE_COLOR_DIAGNOSTICS ON)

# Roots
include("roots.cmake")

# Context specific lists
set(CONTEXTS
  "m2003-motor.Debug+M2003FC1AE"
)
list(LENGTH CONTEXTS CONTEXTS_LENGTH)
math(EXPR CONTEXTS_LENGTH "${CONTEXTS_LENGTH}-1")

set(COMPILERS
  "AC6 V6.24.0"
)

set(DIRS
  "${CMAKE_CURRENT_SOURCE_DIR}/m2003-motor.Debug+M2003FC1AE"
)

set(OUTPUTS_1
  "${SOLUTION_ROOT}/out/m2003-motor/M2003FC1AE/Debug/m2003-motor.axf"
  "${SOLUTION_ROOT}/out/m2003-motor/M2003FC1AE/Debug/m2003-motor.hex"
  "${SOLUTION_ROOT}/out/m2003-motor/M2003FC1AE/Debug/m2003-motor.axf.map"
  "${SOLUTION_ROOT}/out/m2003-motor/M2003FC1AE/Debug/compile_commands.json"
)

set(ARGS
  "-DSOLUTION_ROOT=${SOLUTION_ROOT}"
  "-DCMSIS_PACK_ROOT=${CMSIS_PACK_ROOT}"
  "-DCMSIS_COMPILER_ROOT=${CMSIS_COMPILER_ROOT}"
)

# Compilation database
add_custom_target(database)

# Iterate over contexts
foreach(INDEX RANGE ${CONTEXTS_LENGTH})

  math(EXPR N "${INDEX}+1")
  list(GET CONTEXTS ${INDEX} CONTEXT)
  list(GET COMPILERS ${INDEX} COMPILER)
  list(GET DIRS ${INDEX} DIR)

  # Create external project, set configure and build steps
  ExternalProject_Add(${CONTEXT}
    PREFIX                ${DIR}
    SOURCE_DIR            ${DIR}
    BINARY_DIR            ${N}
    INSTALL_COMMAND       ""
    TEST_COMMAND          ""
    CONFIGURE_COMMAND     ${CMAKE_COMMAND} -G Ninja -S <SOURCE_DIR> -B <BINARY_DIR> ${ARGS} 
    BUILD_COMMAND         ${CMAKE_COMMAND} -E cmake_echo_color --blue --bold "Building CMake target '${CONTEXT}'"
    COMMAND               ${CMAKE_COMMAND} -E echo "Using compiler: ${COMPILER}"
    COMMAND               ${CMAKE_COMMAND} --build <BINARY_DIR>
    BUILD_ALWAYS          TRUE
    BUILD_BYPRODUCTS      ${OUTPUTS_${N}}
    LOG_CONFIGURE         ON
    LOG_OUTPUT_ON_FAILURE ON
    USES_TERMINAL_BUILD   ON
  )

  # Executes command step
  ExternalProject_Add_Step(${CONTEXT} executes
    DEPENDEES         build
  )

  ExternalProject_Add_StepTargets(${CONTEXT} build configure executes)

  # Debug
  message(VERBOSE "Configure Context: ${CMAKE_COMMAND} -G Ninja -S ${DIR} -B ${N}")

  # Database generation step
  ExternalProject_Add_Step(${CONTEXT} database
    COMMAND           ${CMAKE_COMMAND} --build <BINARY_DIR> --target database
    ALWAYS            TRUE
    USES_TERMINAL     ON
    DEPENDEES         configure
  )
  ExternalProject_Add_StepTargets(${CONTEXT} database)
  add_dependencies(database ${CONTEXT}-database)

endforeach()
