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

add_subdirectory(OneMessage-4.3/src om.out)
target_compile_options(OneMessage PUBLIC -w)
target_include_directories(OneMessage PUBLIC $<TARGET_PROPERTY:stm32cubemx,INTERFACE_INCLUDE_DIRECTORIES>)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

file(GLOB TASKS_SRC_FILES ${PROJECT_SOURCE_DIR}/Tasks/Src/*)
file(GLOB MODULES_SRC_FILES ${PROJECT_SOURCE_DIR}/Modules/Src/*)


# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${TASKS_SRC_FILES}
    ${MODULES_SRC_FILES}

)


# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
    # Add user defined include paths
    Tasks/Inc
    Modules/Inc
    DSP/Include
    DSP/Include/dsp
    PUBLIC $<TARGET_PROPERTY:OneMessage,INTERFACE_INCLUDE_DIRECTORIES>
    mavlink/wheellink
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
