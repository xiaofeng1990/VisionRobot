cmake_minimum_required(VERSION 3.10)
project(TensorrtTutorial CXX C)

# executable file output path
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin" CACHE PATH "output directory")
# library file output path
set(LIBRARY_OUTPUT_PATH    "${CMAKE_BINARY_DIR}/lib" CACHE PATH "library output directory")
# cmake install path
set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install")
# runtime 搜索路径
set(CMAKE_INSTALL_RPATH ".:lib:build/install/lib")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(XF_COMPILE_FLAGS "-g -O2 -ffunction-sections -fdata-sections -funwind-tables")
set(XF_COMPILE_FLAGS "${XF_COMPILE_FLAGS} -fstack-protector-strong")

set(CMAKE_C_FLAGS    "${XF_COMPILE_FLAGS}")
set(CMAKE_CXX_FLAGS  "${XF_COMPILE_FLAGS}")
link_directories("/usr/local/lib")

find_package(CUDA REQUIRED)
if(CUDA_FOUND)
    message(STATUS "CUDA library status:")
    message(STATUS "    version: ${CUDA_VERSION}")
    message(STATUS "    include path: ${CUDA_INCLUDE_DIRS}")
    message(STATUS "    lib path: ${CUDA_LIBRARIES}")
    message(STATUS "    install path: ${CUDA_TOOLKIT_ROOT_DIR}")
    set(CMAKE_CUDA_COMPILER ${CUDA_TOOLKIT_ROOT_DIR}/bin/nvcc) 
    include_directories(${CUDA_INCLUDE_DIRS})
    set(LINK_LIBS ${LINK_LIBS} ${CUDA_LIBRARIES} -lcuda)
else()
    message(FATA_ERROR "Do not find CUDA")
endif()

# opencv
# find_package(OpenCV  REQUIRED)
set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/3rd-party/opencv480_aarch64_gpu_install/lib/cmake/opencv4)
find_package(OpenCV PATHS ${OPENCV_DIR} NO_DEFAULT REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "OpenCV library status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "    lib : ${OpenCV_LIBS}")
    message(STATUS "    install path: ${OpenCV_INSTALL_PATH}")
    link_directories(${OpenCV_INSTALL_PATH}/lib)
    link_directories(${OpenCV_INSTALL_PATH}/share/OpenCV/3rdparty)
    include_directories(${OpenCV_INCLUDE_DIRS})
    set(LINK_LIBS ${LINK_LIBS} ${OpenCV_LIBS})
else()
    message(FATA_ERROR "Do not find OpenCV")
endif()

# Orbec
set(OrbbecSDK_DIR ${CMAKE_SOURCE_DIR}/3rd-party/OrbbecSDK-2.3.5)
find_package(OrbbecSDK REQUIRED)
if(OrbbecSDK_FOUND)
  message(STATUS "Orbec library status:")
  message(STATUS "    include path: ${OrbbecSDK_INCLUDE_DIRS}")
  message(STATUS "    lib path: ${OrbbecSDK_LIBS_DIR}")
  message(STATUS "    lib: ${OrbbecSDK_LIBS}")
  link_directories(${OrbbecSDK_LIBS_DIR})
  # message(STATUS     OrbbecSDK::OrbbecSDK)
else(OrbbecSDK_FOUND)
  message(FATA_ERROR "Do not find Orbec")
endif()

# nlohmann
include_directories(${CMAKE_SOURCE_DIR}/3rd-party/nlohmann)

include_directories(.)
add_subdirectory(vision_3dof)
add_subdirectory(orbbec_test)
