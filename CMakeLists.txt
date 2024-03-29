# Remake on 2023-01-15 by dgsyrc
# For RoboMaster Team Sirius(BJFU&BISTU)

# Create project
cmake_minimum_required(VERSION 3.10.0)
project(RobotVision_Sirius VERSION 2023.1.15 LANGUAGES CXX)

# Check system platform
if(NOT CMAKE_SYSTEM_NAME STREQUAL Linux)
    message(FATAL_ERROR "Not supported platform: ${CMAKE_SYSTEM_NAME}")
endif()

# Specialize CMake build flags for different build types
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-deprecated")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Use standard c++20
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add DEBUG/RELEASE marco to define if debug/release configured
add_compile_options("$<$<CONFIG:DEBUG>:-DDEBUG>")
add_compile_options("$<$<CONFIG:RELEASE>:-DRELEASE>")

# Don't build test by default
set(BUILD_TEST False)

# Prefer posix threads
set(THREADS_PREFER_PTHREAD_FLAG True)
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
find_package(Threads REQUIRED)

# Find and add opencv as required
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})

# Add fmtlib directory
add_subdirectory(3rdparty/fmt)
# Add fmtlib include to EXTRA_INCLUDES
list(APPEND EXTRA_INCLUDES 3rdparty/fmt/include)

# Add mindvision-sdk directory and include path for target
add_library(MVSDK SHARED IMPORTED)
get_filename_component(MVSDK_PATH 3rdparty/mindvision/linux REALPATH)
# Check system architecture for mindvision-sdk library
execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message(STATUS "Checking platform architecture for MVSDK: ${ARCHITECTURE}")
if(${ARCHITECTURE} STREQUAL x86_64)
    set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/x64/libMVSDK.so)
elseif(${ARCHITECTURE} STREQUAL x86)
    set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/x86/libMVSDK.so)
elseif(${ARCHITECTURE} STREQUAL aarch64)
    set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/arm64/libMVSDK.so)
elseif(${ARCHITECTURE} STREQUAL armv7)
    set(MIMDVISION_SDK_LIBRARY ${MVSDK_PATH}/lib/arm/libMVSDK.so)
else()
    message(FATAL_ERROR "Not supported architecture for MVSDK: ${ARCHITECTURE}")
endif()
set_property(TARGET MVSDK PROPERTY IMPORTED_LOCATION ${MIMDVISION_SDK_LIBRARY})
# Add mindvision lib to build target
list(APPEND EXTRA_LIBS MVSDK)
# Add mindvision include path
list(APPEND EXTRA_INCLUDES 3rdparty/mindvision/linux/include)

# Add project source dir to include
list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR})

# Add base src and header
list(APPEND EXTRA_INCLUDES base)

# Add device as shared library to build target
list(APPEND EXTRA_INCLUDES devices)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/devices/camera)
add_library(rvs-mv-video-capture SHARED devices/camera/mv_video_capture.cpp)
target_link_libraries(rvs-mv-video-capture
    PRIVATE
    MVSDK
)
list(APPEND RVS_EXTRA_LIBS rvs-mv-video-capture)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/devices/serial)
add_library(rvs-uart-serial SHARED devices/serial/uart_serial.cpp)
list(APPEND RVS_EXTRA_LIBS rvs-uart-serial)

# Add module as shared library to build target
list(APPEND EXTRA_INCLUDES module)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/angle_solve)
add_library(rvs-basic-pnp SHARED module/angle_solve/basic_pnp.cpp)
list(APPEND RVS_EXTRA_LIBS rvs-basic-pnp)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/angle_solve)
add_library(rvs-angle-solve SHARED module/angle_solve/angle_solve.cpp)
list(APPEND RVS_EXTRA_LIBS rvs-angle-solve)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/buff)
add_library(rvs-new-buff SHARED module/buff/new_buff.cpp)
list(APPEND RVS_EXTRA_LIBS rvs-new-buff)


list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/armor)
add_library(rvs-basic-armor SHARED module/armor/basic_armor.cpp)
list(APPEND RVS_EXTRA_LIBS rvs-basic-armor)
add_library(rvs-fan-armor SHARED module/armor/fan_armor.cpp)
list(APPEND RVS_EXTRA_LIBS rvs-fan-armor)


list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/buff)
add_library(rvs-basic-buff SHARED module/buff/basic_buff.cpp)
target_link_libraries(rvs-basic-buff
    PRIVATE
    rvs-fan-armor
    rvs-new-buff
)
list(APPEND RVS_EXTRA_LIBS rvs-basic-buff)



list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/filter)
add_library(rvs-basic-kalman SHARED module/filter/basic_kalman.cpp)
list(APPEND RVS_EXTRA_LIBS rvs-basic-kalman)


list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/record)
add_library(rvs-video-record SHARED module/record/record.cpp)
list(APPEND RVS_EXTRA_LIBS rvs-video-record)


list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/ml)
add_library(rvs-onnx-inferring SHARED module/ml/onnx_inferring.cpp)
list(APPEND RVS_EXTRA_LIBS rvs-onnx-inferring)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/roi)
add_library(rvs-basic-roi SHARED module/roi/basic_roi.cpp)
list(APPEND RVS_EXTRA_LIBS rvs-basic-roi)

list(APPEND EXTRA_INCLUDES ${PROJECT_SOURCE_DIR}/module/camera)
add_library(rvs-camera-calibration SHARED module/camera/camera_calibration.cpp)
list(APPEND RVS_EXTRA_LIBS rvs-camera-calibration)

# Add utils header
list(APPEND EXTRA_INCLUDES utils)

# Add EXTRA_INCLUDES to project includes
include_directories(${EXTRA_INCLUDES})

# Add base/RobotVision_Sirius to executable
add_executable(RobotVision_Sirius base/RobotVision_Sirius.cpp)

# Add shared library link to RobotVision_Sirius
target_link_libraries(RobotVision_Sirius
    PUBLIC
    Threads::Threads
    fmt::fmt
    ${OpenCV_LIBS}
    ${EXTRA_LIBS}

    PRIVATE
    ${RVS_EXTRA_LIBS}
)

# Set RobotVision_Sirius compile cxx_std features
target_compile_features(RobotVision_Sirius PUBLIC cxx_std_${CMAKE_CXX_STANDARD})

# Set RobotVision_Sirius linking time optimization to true
set_target_properties(RobotVision_Sirius PROPERTIES INTERPROCEDURAL_OPTIMIZATION True)

# Set built binary to ~/bin
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")

# Set built library to ~/lib
set(LIBRARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/lib")

# Set configs folder absolute path
# define
set(CONFIG_FILE_PATH ${PROJECT_SOURCE_DIR}/configs)
set(SOURCE_PATH ${PROJECT_SOURCE_DIR})
set(SAVE_FILE_PATH ${PROJECT_SOURCE_DIR}/configs/camera)
target_compile_definitions(RobotVision_Sirius PRIVATE "CONFIG_FILE_PATH=\"${CONFIG_FILE_PATH}\"")
target_compile_definitions(RobotVision_Sirius PRIVATE "SOURCE_PATH=\"${SOURCE_PATH}\"")
target_compile_definitions(rvs-basic-buff PRIVATE "CONFIG_FILE_PATH=\"${CONFIG_FILE_PATH}\"")
target_compile_definitions(rvs-onnx-inferring PRIVATE "CONFIG_FILE_PATH=\"${CONFIG_FILE_PATH}\"")
target_compile_definitions(rvs-video-record PRIVATE "CONFIG_FILE_PATH=\"${CONFIG_FILE_PATH}\"")
target_compile_definitions(rvs-camera-calibration PRIVATE "SAVE_FILE_PATH=\"${SAVE_FILE_PATH}\"")


# Add test_camera directory
if(BUILD_TEST)
    add_subdirectory(test/test_camera)
endif()
