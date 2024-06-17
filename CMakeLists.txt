cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD                11)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)
set(CMAKE_CXX_STANDARD              20)
set(CMAKE_CXX_STANDARD_REQUIRED     ON)
set(CMAKE_CXX_EXTENSIONS            ON)
set(PROJ_PATH                       ${CMAKE_CURRENT_SOURCE_DIR})
message("Build type: "              ${CMAKE_BUILD_TYPE})

#
# Core project settings
#
project(spicy_beef)
enable_language(C CXX ASM)

#
# Core MCU flags, CPU, instruction set and FPU setup
# Needs to be set properly for your MCU
#
set(CPU_PARAMETERS
    -mthumb

    # This needs attention to properly set for used MCU
    -mcpu=cortex-m33
    -mfpu=fpv5-sp-d16
    -mfloat-abi=hard
)

# Set linker script
set(linker_script_SRC               ${PROJ_PATH}/STM32U575ZITXQ_FLASH.ld)
set(EXECUTABLE                      ${CMAKE_PROJECT_NAME})

# Create a variable that will recursively search the ${PROJ_PATH}/Core directory for all *.c and *.s files
# Some files are in the /Src and some in the Startup directory
file(GLOB_RECURSE core_SRCS
    ${PROJ_PATH}/Core/*.c
    ${PROJ_PATH}/Core/*.s
)

# Add all source files in the Drivers directory as well to the sources_SRCS variable
file(GLOB_RECURSE drivers_SRCS
    ${PROJ_PATH}/Drivers/*.c
)

# Add all source files in the Middlewares directory as well to the sources_SRCS variable
file(GLOB_RECURSE middlewares_SRCS
    ${PROJ_PATH}/Middlewares/*.c
)

# Add all source files in the FileX directory as well to the sources_SRCS variable
file(GLOB_RECURSE filex_SRCS
    ${PROJ_PATH}/FileX/*.c
)

#
# List of source files to compile
#
set(sources_SRCS
    ${core_SRCS}
    ${drivers_SRCS}
    ${middlewares_SRCS}
    ${filex_SRCS}
)

# print all the source files
message("Source files: " ${sources_SRCS}
)


# Function to collect directories containing *.h files
function(collect_include_dirs base_path result)
    file(GLOB_RECURSE header_files RELATIVE "${base_path}" "${base_path}/*.h")
    set(dir_list "")
    foreach(header_file ${header_files})
        get_filename_component(header_dir "${base_path}/${header_file}" PATH)
        list(APPEND dir_list "${header_dir}")
    endforeach()
    list(REMOVE_DUPLICATES dir_list)
    set(${result} ${dir_list} PARENT_SCOPE)
endfunction()

# Collect include directories
collect_include_dirs(${PROJ_PATH}/Core CORE_INCLUDE_DIRS)
collect_include_dirs(${PROJ_PATH}/Drivers DRIVERS_INCLUDE_DIRS)
collect_include_dirs(${PROJ_PATH}/Middlewares/ST MIDDLEWARES_INCLUDE_DIRS)
collect_include_dirs(${PROJ_PATH}/FileX FILEX_INCLUDE_DIRS)

#
# Include directories
#
set(include_path_DIRS
    ${CORE_INCLUDE_DIRS}
    ${DRIVERS_INCLUDE_DIRS}
    ${MIDDLEWARES_INCLUDE_DIRS}
    ${FILEX_INCLUDE_DIRS}
    ${PROJ_PATH}/Middlewares/Third_Party
    ${PROJ_PATH}/Middlewares/Third_Party/lvgl
)

# Add some include directories so that intellisense will detect them
include_directories(
    ${CORE_INCLUDE_DIRS}
    ${DRIVERS_INCLUDE_DIRS}
    ${MIDDLEWARES_INCLUDE_DIRS}
    ${FILEX_INCLUDE_DIRS}
    ${PROJ_PATH}/Middlewares/Third_Party
    ${PROJ_PATH}/Middlewares/Third_Party/lvgl
)

# Print all include directories (for debugging)
foreach(dir ${include_path_DIRS})
    message(STATUS "Include dir: ${dir}")
endforeach()


#
# Symbols definition
#
set(symbols_SYMB
    "DEBUG"
    "FX_INCLUDE_USER_DEFINE_FILE"
    "USE_HAL_DRIVER"
    "STM32U575xx"
)

# Executable files
add_executable(${EXECUTABLE} ${sources_SRCS})

# Include paths
target_include_directories(${EXECUTABLE} PRIVATE ${include_path_DIRS})

# Project symbols
target_compile_definitions(${EXECUTABLE} PRIVATE ${symbols_SYMB})

# Compiler options
target_compile_options(${EXECUTABLE} PRIVATE
    ${CPU_PARAMETERS}
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    # Full debug configuration
    -Og -g3 -ggdb
)

# Linker options
target_link_options(${EXECUTABLE} PRIVATE
    -T${linker_script_SRC}
    ${CPU_PARAMETERS}
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    --specs=nosys.specs
    -u _printf_float                # STDIO float formatting support
    -Wl,--start-group
    -lc
    -lm
    -lstdc++
    -lsupc++
    -Wl,--end-group
    -Wl,--print-memory-usage
)

# Execute post-build to print size
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>
)

# Convert output to hex and binary
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.hex
)

# Convert to bin file -> add conditional check?
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin
)