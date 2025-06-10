cmake_minimum_required(VERSION 3.10)

project(vision_3dof CXX C)
set(TARGET_NAME vision_3dof)

set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin" CACHE PATH "output directory")
set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install")

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

if (NOT WIN32)
    set(EXTERNAL_LIB "-ldl -lrt -lgomp -lz -lm -lpthread")
    set(LINK_LIBS ${LINK_LIBS} ${EXTERNAL_LIB})
endif()

find_package(Git QUIET)
if (GIT_FOUND)
    execute_process(COMMAND "${GIT_EXECUTABLE}" describe --tags --always --dirty --match "[0-9].[0-9].[0-9]*"
        WORKING_DIRECTORY "${${PROJECT_NAME}_SOURCE_DIR}"
        OUTPUT_VARIABLE XT_GIT_ONELINE
        RESULT_VARIABLE GET_RESULT
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if (NOT GET_RESULT EQUAL 0)
        set(XT_GIT_ONELINE "unknow")
    endif()
else()
    set(XT_GIT_ONELINE "unknow")
endif()

message(STATUS "LINK_LIBS " ${LINK_LIBS})

set(XT_COMPILE_FLAGS "-g -O0 -ffunction-sections -fdata-sections -funwind-tables")
set(XT_COMPILE_FLAGS "${XT_COMPILE_FLAGS} -fstack-protector-strong")

set(CMAKE_C_FLAGS    "${XT_COMPILE_FLAGS}")
set(CMAKE_CXX_FLAGS  "${XT_COMPILE_FLAGS}")

add_subdirectory(coordinate_transform)
