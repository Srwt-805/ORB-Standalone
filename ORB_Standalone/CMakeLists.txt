cmake_minimum_required(VERSION 3.10)
project(ORB_Standalone CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Optimization and warning flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-O3 -Wall -Wextra -pedantic)
endif()

# ------------------------------------------------------------------------------
# 1. Standalone ORB Feature Extractor Library
#    Pure C++ computational core - ZERO OpenCV dependency.
# ------------------------------------------------------------------------------
add_library(orb_standalone_lib STATIC
    src/Border.cpp
    src/ImagePyramid.cpp
    src/FastDetector.cpp
    src/FeatureDistribution.cpp
    src/Atan2.cpp
    src/Orientation.cpp
    src/GaussianFilter.cpp
    src/BriefDescriptor.cpp
    src/ORBExtractor.cpp
)

target_include_directories(orb_standalone_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ------------------------------------------------------------------------------
# 2. Demonstration Application (Webcam Demo)
#    OpenCV is used ONLY here for camera acquisition and visual display.
# ------------------------------------------------------------------------------
find_package(OpenCV QUIET COMPONENTS core videoio highgui imgproc)

if(OpenCV_FOUND)
    message(STATUS "OpenCV ${OpenCV_VERSION} found: enabling webcam_demo executable.")
    add_executable(webcam_demo
        examples/webcam_demo.cpp
    )
    target_link_libraries(webcam_demo PRIVATE
        orb_standalone_lib
        ${OpenCV_LIBS}
    )
    target_include_directories(webcam_demo PRIVATE
        ${OpenCV_INCLUDE_DIRS}
    )
else()
    message(STATUS "OpenCV not found: skipping webcam_demo. Built standalone library orb_standalone_lib.")
endif()
