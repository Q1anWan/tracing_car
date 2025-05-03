cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME rt-ctr)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)
add_subdirectory(OneMessage om)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

file(GLOB TASKS_SRC_FILES ${PROJECT_SOURCE_DIR}/Tasks/Src/*)
file(GLOB MODULES_SRC_FILES ${PROJECT_SOURCE_DIR}/Modules/Src/*)
#file(GLOB OM_SRC_FILES ${PROJECT_SOURCE_DIR}/OneMessage/*.c
#        ${CMAKE_CURRENT_SOURCE_DIR}/OneMessage/app/*.c
#        ${CMAKE_CURRENT_SOURCE_DIR}/OneMessage/core/*.c
#        ${CMAKE_CURRENT_SOURCE_DIR}/OneMessage/comp/*.c)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${TASKS_SRC_FILES}
    ${MODULES_SRC_FILES}
#    ${OM_SRC_FILES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
    # Add user defined include paths
    Tasks/Inc
    Modules/Inc
    DSP/Include
    DSP/Include/dsp
    $<TARGET_PROPERTY:OneMessage,INCLUDE_DIRECTORIES>
#    OneMessage
#    OneMessage/app
#    OneMessage/core
#    OneMessage/comp
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    ${PROJECT_SOURCE_DIR}/DSP/libCMSISDSP.a
    OneMessage
    # Add user defined libraries
)
