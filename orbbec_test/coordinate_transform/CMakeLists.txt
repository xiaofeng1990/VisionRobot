# Copyright (c) Orbbec Inc. All Rights Reserved.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 3.0)
project(ob_coordinate_transform CXX C)
set(TARGET_NAME ${PROJECT_NAME})

# 添加目录中所有源文件
# aux_source_directory(${PROJECT_SOURCE_DIR}/src   SRC_FILES)
file(GLOB_RECURSE SRC_FILES ${PROJECT_SOURCE_DIR}/*.cc)

cuda_add_executable(${TARGET_NAME} ${SRC_FILES})
# cuda_add_executable(${APP_NAME} ${SRC_FILES})
target_link_libraries(${TARGET_NAME} ${LINK_LIBS}   OrbbecSDK::OrbbecSDK)
# message("link libs ${LINK_LIBS}")
install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})


