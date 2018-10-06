cmake_minimum_required(VERSION 3.0)
project(wrapper)
add_compile_options(-std=c++14 -Werror=return-type -Wall)

## Adding DEBUG_MODE for Connector::util.h
#OPTION(DEBUG_MODE "Enables the DEBUG messages and -O0 flags" 1)

if(DEBUG_MODE)
    message("DEBUG_MODE is ON")
    add_definitions(-DDEBUG_MODE=1)
else()
    message("DEBUG_MODE is OFF")
    add_definitions(-DDEBUG_MODE=0)
endif()

set(PYLON 0)
set(CUDNN 0)

if(CUDNN)
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
    find_package(CUDNN REQUIRED)
    add_definitions(-DCUDNN=1)
endif()

find_package(BLAS)
find_library(DARKNET darknet)
set(OpenCV_DIR "/usr/share/OpenCV/")
find_package(OpenCV 3.1.0 NO_DEFAULT_PATH REQUIRED)
find_package(Threads)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(OpenGL REQUIRED)
find_package(X11 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Freetype REQUIRED)

find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
pkg_check_modules(GST REQUIRED gstreamer-base-1.0
                               gstreamer-pbutils-1.0
                               gstreamer-app-1.0)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${CUDNN_INCLUDE_DIRS}
    ${PYLON_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${OpenGL_INLCUDE_DIR}
    eigen3
    src
    src/wrapper
    nvxio/include
    nvx/tracking
    nvx
)

link_directories(
    /usr/lib/x86_64-linux-gnu
    /usr/local/cuda/lib
    /usr/local/cuda/lib64
    libs
    src
    src/wrapper
)

add_executable(darknet-debug
    src/wrapper/darknet-debug.cpp
    src/wrapper/detector-wrapper.hpp
    src/wrapper/object.h
    src/kalman/kalman_filter.cpp src/kalman/kalman_filter.h)

target_link_libraries(darknet-debug
    ${OpenCV_LIBS}
    ${PYLON_LIBRARIES} 
    ${CUDNN_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${Boost_LIBRARIES}
    darknet
    cuda 
    cudart 
    cublas 
    curand
)
