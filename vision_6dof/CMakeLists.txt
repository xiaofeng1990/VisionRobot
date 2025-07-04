cmake_minimum_required(VERSION 3.10)

project(vision_6dof CXX C)
set(TARGET_NAME vision_6dof)

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

aux_source_directory(${PROJECT_SOURCE_DIR}/orbbec           ORBBEC_SRC_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/episode          EPISODE_SRC_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/teach_mode       TEACH_MODE_SRC_FILES)
aux_source_directory(${PROJECT_SOURCE_DIR}/calibration      CALIBRATION_SRC_FILES)
aux_source_directory(${CMAKE_SOURCE_DIR}/common             COMMON_SRC_FILES)


include_directories(.)

# include_directories(/usr/include/freetype2)
cuda_add_executable(${TARGET_NAME} 
    main.cc
    ${COMMON_SRC_FILES}
    ${ORBBEC_SRC_FILES}
    ${EPISODE_SRC_FILES}
    ${CALIBRATION_SRC_FILES}
    ${TEACH_MODE_SRC_FILES}
    )

target_link_libraries(${TARGET_NAME} OrbbecSDK::OrbbecSDK -lboost_filesystem -lboost_system
                    ${LINK_LIBS})

# install(FILES ${PADDLE_LIB}/paddle/lib/libpaddle_inference.so DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
# set_target_properties(${TARGET_NAME}  PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags,-rpath,${CMAKE_INSTALL_PREFIX}/lib:${FASTDEPLOY_ASCEND_DIR}/third_libs/install/opencv/lib")
# set_target_properties(${TARGET_NAME}  PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags,-rpath,${CMAKE_INSTALL_PREFIX}/lib")

# set_target_properties(${TARGET_NAME} PROPERTIES
#     BUILD_WITH_INSTALL_RPATH TRUE      # 使用安装路径构建 RPATH
#     INSTALL_RPATH "${OpenCV_LIB_DIR}"  # 设置安装后的库路径
#     SKIP_BUILD_RPATH FALSE             # 不跳过构建时的 RPATH
#     BUILD_RPATH "${CMAKE_INSTALL_PREFIX}/lib:${OpenCV_INSTALL_PATH}/lib:${OrbbecSDK_LIBS_DIR}"    # 设置构建时的库路径
#     INSTALL_RPATH_USE_LINK_PATH TRUE   # 包含链接路径
# )



install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})