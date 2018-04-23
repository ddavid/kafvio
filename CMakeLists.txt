cmake_minimum_required(VERSION 2.8.3)
project(wrapper)
add_compile_options(-std=c++14 -Werror=return-type -Wall)

add_library(darknet SHARED IMPORTED) # or STATIC instead of SHARED
set_target_properties(darknet PROPERTIES
  IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/darknet.so"
)

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

#set (CMAKE_BUILD_TYPE  DEBUG)
#set (OpenCV_DIR /opt/opencv320/share/OpenCV)

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

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${CUDNN_INCLUDE_DIRS}
    ${PYLON_INCLUDE_DIRS}
    src
)

link_directories(
    /usr/local/cuda/lib
    /usr/local/cuda/lib64
    lib
    src
)

add_executable(main
    src/yolo_console_dll.cpp
    #main.cpp
    #src/yolo_v2_class.cpp
    src/yolo_v2_class.hpp
    src/object.hpp
    src/object.cpp
    
)

target_link_libraries(main 
    ${OpenCV_LIBS}
    ${PYLON_LIBRARIES} 
    ${CUDNN_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    darknet
    cuda 
    cudart 
    cublas 
    curand
)
