cmake_minimum_required (VERSION 3.5)

project (motor_test)

SET(LIBRARY_OUTPUT_PATH ./release) 
SET(EXECUTABLE_OUTPUT_PATH ./bin)

add_compile_options(-Wall  -g -fdiagnostics-color=auto -fPIC)

include_directories(./src ./libfinsh)

aux_source_directory(./libfinsh DIR_SRC)
aux_source_directory(./src DIR_SRC)
# aux_source_directory(./example DIR_EXAMPLE)

# add_library(slc STATIC ${DIR_SRC})

# link_directories(${CMAKE_CURRENT_BINARY_DIR}/release) 

add_executable(motor_test ${DIR_SRC})
target_link_libraries(motor_test -lpthread -lm)

# add_dependencies(slc_test slc)

# execute_process(COMMAND ${CMAKE_COMMAND} -E
# copy "../src/slc.h"  "${LIBRARY_OUTPUT_PATH}/"
# )