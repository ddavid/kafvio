cmake_minimum_required(VERSION 3.0)
project(wrapper)
add_compile_options(-std=c++14 -Werror=return-type -Wall)

## Adding DEBUG_MODE for Connector::util.h
OPTION(DEBUG_MODE "Enables the DEBUG messages and -O0 flags" 1) 

if(DEBUG_MODE)
    message("DEBUG_MODE is ON")
    add_definitions(-DDEBUG_MODE=1)
else()
    message("DEBUG_MODE is OFF")
    add_definitions(-DDEBUG_MODE=0)
endif()

set(DEBUG 1)
#set(GPU 1)
set(PYLON 1)
set(CUDNN 1)
set(VISIONWORKS 1)

#set (CMAKE_BUILD_TYPE  DEBUG)

# Setting CUDA this way breaks because of `error: template with C linkage`
# 
#if(GPU)
#	find_package(CUDA REQUIRED)
#	add_definitions(-DGPU=1)
#endif()

if(CUDNN)
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
    find_package(CUDNN REQUIRED)
    add_definitions(-DCUDNN=1)
endif()

if(PYLON)
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules")
    find_package(Pylon REQUIRED)
endif()

if(VISIONWORKS)
    list(APPEND CMAKE_MODULE_PATH "/usr/share/visionworks-tracking/cmake")
    find_package(VisionWorks REQUIRED)
    #[[find_package(VisionWorks-Tracking REQUIRED)
    message(STATUS "==> VISIONWORKS-TRACKING " ${VisionWorks-Tracking_INCLUDE_DIRS})
    message(STATUS "==> VISIONWORKS-TRACKING " ${VisionWorks-Tracking_LIBRARIES})]]
endif()

find_package(BLAS)
find_library(DARKNET darknet)

if(DARKNET_FOUND)
    message(STATUS "==> DARKNET found")
    else()
    message(STATUS "==> DARKNET not found")
endif()

# debug information
if(DEBUG)

	if(OpenCV_FOUND)
		message(STATUS "==> OPENCV found")
	else()
		message(STATUS "==> OPENCV not found")
	endif()
	
	if(CUDA_FOUND)
        message(STATUS "==> CUDA found")
    else()
        message(STATUS "==> CUDA not found")
    endif()

    if(CUDNN_FOUND)
        message(STATUS "==> CUDNN found")
    else()
        message(STATUS "==> CUDNN not found")
    endif()

    if(Pylon_FOUND)
        message(STATUS "==> PYLON found")
    else()
        message(STATUS "==> PYLON not found")
    endif()

    #message(STATUS "==> COMMON_FLAGS " ${COMMON_FLAGS})
    #message(STATUS "==> CMAKE_C_FLAGS: " ${CMAKE_C_FLAGS})
    #message(STATUS "==> CMAKE_CXX_FLAGS: " ${CMAKE_CXX_FLAGS})
    message(STATUS "==> CUDA_NVCC_FLAGS: " ${CUDA_NVCC_FLAGS})
    message(STATUS "==> CUDA_curand_LIBRARY: " ${CUDA_curand_LIBRARY})
    message(STATUS "==> CUDA_LIBRARIES: " ${CUDA_LIBRARIES})
    message(STATUS "==> CUDA_CUBLAS_LIBRARIES: " ${CUDA_CUBLAS_LIBRARIES})
    message(STATUS "==> CUDA_curand_LIBRARY: " ${CUDA_curand_LIBRARY})
    message(STATUS "==> CUDA_INCLUDE_DIRS: " ${CUDA_INCLUDE_DIRS})
    message(STATUS "==> CUDNN_INCLUDE_DIRS: " ${CUDNN_INCLUDE_DIRS})
    message(STATUS "==> CUDNN_LIBRARIES: " ${CUDNN_LIBRARIES})
    message(STATUS "==> PYLON_LIBRARIES: " ${PYLON_INCLUDE_DIRS})
    message(STATUS "==> PYLON_LIBRARIES: " ${PYLON_LIBRARIES})

endif()

find_package(OpenCV REQUIRED)
find_package(Threads)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(connector 1.0 REQUIRED)
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
)

add_executable(main
    src/basler-demo.cpp
    src/yolo_v2_class.hpp
    src/object.h  
)

add_executable(tracker
    vision-works/main_object_tracker.cpp)

target_link_libraries(main 
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

target_link_libraries(tracker
    ${OpenCV_LIBS}
    ${PYLON_LIBRARIES} 
    ${CUDNN_LIBRARIES}
    ${Boost_LIBRARIES}
    ${VisionWorks_LIBRARIES} 
    ovx
    nvx
    ${GST_LIBRARIES}
    visionworks_tracking.so
    glfw
    #xx86
    ${X11_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${FREETYPE_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    darknet
    cuda 
    cudart 
    cublas 
    curand
)