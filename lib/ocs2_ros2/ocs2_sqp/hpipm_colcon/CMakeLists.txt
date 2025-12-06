cmake_minimum_required(VERSION 3.14)
project(hpipm_colcon)

set(CMAKE_BUILD_TYPE Release)

set(DEFAULT_HPIPM_GIT_TAG "255ffdf38d3a5e2c3285b29568ce65ae286e5faf")

# Set TARGET based on the processor architecture
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    message(STATUS "Building for Intel x86_64 architecture")
    set(HPIPM_CPU_TARGET "AVX")
    set(GIT_TAG "${DEFAULT_HPIPM_GIT_TAG}")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    message(STATUS "Building for ARM64 architecture")
    set(HPIPM_CPU_TARGET "GENERIC")
    set(GIT_TAG "${DEFAULT_HPIPM_GIT_TAG}")
else()
    message(STATUS "Building for unknown architecture")
    set(HPIPM_CPU_TARGET "GENERIC")
    set(GIT_TAG "${DEFAULT_HPIPM_GIT_TAG}")
endif()

## Find ament_cmake macros and libraries
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log log_setup)
find_package(blasfeo_colcon REQUIRED)
find_package(ocs2_qp_solver REQUIRED)
include(FetchContent)

# Define directories used to stage third-party artifacts before install
set(HPIPM_LIB_STAGE_DIR ${CMAKE_CURRENT_BINARY_DIR}/hpipm_lib)
set(HPIPM_INCLUDE_STAGE_DIR ${CMAKE_CURRENT_BINARY_DIR}/hpipm_include)
file(MAKE_DIRECTORY ${HPIPM_LIB_STAGE_DIR})
file(MAKE_DIRECTORY ${HPIPM_INCLUDE_STAGE_DIR})

message(STATUS "Building hpipm with BLASFEO at: ${BLASFEO_PATH}")

# HPIPM Settings
set(BLASFEO_PATH ${BLASFEO_PATH} CACHE STRING "BLASFEO installation path" FORCE)
set(BLASFEO_INCLUDE_DIR ${BLASFEO_PATH}/include CACHE STRING "Path to BLASFEO header files." FORCE)
set(BUILD_SHARED_LIBS ON CACHE STRING "Build shared libraries" FORCE)
set(HPIPM_TESTING OFF CACHE BOOL "Examples enabled")

# Download & build source
FetchContent_Declare(hpipmDownload
        GIT_REPOSITORY https://github.com/giaf/hpipm
        GIT_TAG ${GIT_TAG}
        UPDATE_COMMAND ""
)

# Force hpipm to use the desired CPU target while it is added to the build
set(TARGET ${HPIPM_CPU_TARGET} CACHE STRING "Set CPU architecture target" FORCE)
FetchContent_MakeAvailable(hpipmDownload)
unset(TARGET CACHE)

FetchContent_GetProperties(hpipmDownload)
set(HPIPM_DOWNLOAD_DIR "${hpipmDownload_SOURCE_DIR}")
set(HPIPM_BUILD_DIR "${hpipmDownload_BINARY_DIR}")

# Fallback to default FetchContent paths if the helper variables are empty (older CMake)
if(NOT HPIPM_DOWNLOAD_DIR)
    set(HPIPM_DOWNLOAD_DIR "${CMAKE_CURRENT_BINARY_DIR}/_deps/hpipmdownload-src")
endif()
if(NOT HPIPM_BUILD_DIR)
    set(HPIPM_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}/_deps/hpipmdownload-build")
endif()

message(STATUS "HPIPM source dir: ${HPIPM_DOWNLOAD_DIR}")
message(STATUS "HPIPM build dir: ${HPIPM_BUILD_DIR}")

# Ensure HPIPM artifacts end up in our install location
set_target_properties(hpipm PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY ${HPIPM_LIB_STAGE_DIR}
        LIBRARY_OUTPUT_DIRECTORY ${HPIPM_LIB_STAGE_DIR}
)

# Copy headers locally so ament can stage them
if(NOT EXISTS "${HPIPM_DOWNLOAD_DIR}/include")
    message(FATAL_ERROR "hpipm include directory not found at ${HPIPM_DOWNLOAD_DIR}/include")
endif()
file(COPY ${HPIPM_DOWNLOAD_DIR}/include/ DESTINATION ${HPIPM_INCLUDE_STAGE_DIR})

###########
## Build ##
###########
set(dependencies
        blasfeo_colcon
        ocs2_qp_solver
)

# Hpipm interface
add_library(${PROJECT_NAME}
        src/HpipmInterface.cpp
        src/HpipmInterfaceSettings.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${HPIPM_DOWNLOAD_DIR}/include>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} hpipm)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

#############
## Install ##
#############
install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY ${HPIPM_LIB_STAGE_DIR}/
        DESTINATION lib
)

install(
        DIRECTORY ${HPIPM_INCLUDE_STAGE_DIR}/
        DESTINATION include
)


ament_export_dependencies(${dependencies} hpipm)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_${PROJECT_NAME} test/testHpipmInterface.cpp)
    ament_target_dependencies(test_${PROJECT_NAME} ${dependencies})
    target_link_libraries(test_${PROJECT_NAME}
            ${PROJECT_NAME}
            hpipm
    )

endif ()

ament_package()
