cmake_minimum_required(VERSION 3.16)
project(mujoco_vendor)

if(BUILD_HOST_YOCTO)
    message("Building for ONEX OS, skipping mujoco_vendor")
    find_package(ament_cmake REQUIRED)
    ament_package()
    return()
endif()

find_package(ament_cmake REQUIRED)

include(ExternalProject)

# Set the Mujoco version and download details
set(MUJOCO_VERSION "3.3.4")

# Pick the correct archive for the build architecture (default to x86_64).
set(MUJOCO_ARCH "linux-x86_64")
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  set(MUJOCO_ARCH "linux-aarch64")
endif()

set(MUJOCO_ARCHIVE "mujoco-${MUJOCO_VERSION}-${MUJOCO_ARCH}.tar.gz")
set(MUJOCO_DOWNLOAD_URL "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/${MUJOCO_ARCHIVE}")
set(MUJOCO_TARBALL "${CMAKE_BINARY_DIR}/mujoco_prebuilt/src/${MUJOCO_ARCHIVE}")
set(MUJOCO_INSTALL_DIR ${CMAKE_BINARY_DIR}/mujoco) 

set(MUJOCO_URL_HASH "")
# SHA256 for the official x86_64 archive (kept to preserve integrity checks).
if(MUJOCO_ARCH STREQUAL "linux-x86_64")
  set(MUJOCO_URL_HASH "SHA256=ecf1a17459a342badf2b4f32dd4677a6a0e5fd393c5143993eb3e81b8e44609b")
endif()
if(MUJOCO_URL_HASH)
  set(MUJOCO_HASH_ARG URL_HASH ${MUJOCO_URL_HASH})
endif()

# Add Mujoco as an external project
ExternalProject_Add(mujoco_prebuilt
    URL ${MUJOCO_DOWNLOAD_URL}
    ${MUJOCO_HASH_ARG}
    PREFIX ${CMAKE_BINARY_DIR}/mujoco_prebuilt
    INSTALL_DIR ${MUJOCO_INSTALL_DIR}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND tar --strip-components=1 -xzf ${MUJOCO_TARBALL} -C ${MUJOCO_INSTALL_DIR}
)

# Create an interface library for linking with Mujoco
add_library(mujoco INTERFACE)
add_dependencies(mujoco mujoco_prebuilt)

# Specify include directories and link libraries for Mujoco
target_include_directories(mujoco INTERFACE
    ${MUJOCO_INSTALL_DIR}/include
)
target_link_libraries(mujoco INTERFACE
  ${MUJOCO_INSTALL_DIR}/lib/libmujoco.so.${MUJOCO_VERSION}
)

# Install Mujoco headers and library
install(DIRECTORY ${MUJOCO_INSTALL_DIR}/include/
        DESTINATION include)

install(FILES ${MUJOCO_INSTALL_DIR}/lib/libmujoco.so.${MUJOCO_VERSION}
        DESTINATION lib)

# Create a symlink to the versioned target 
install(CODE "
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E create_symlink libmujoco.so.${MUJOCO_VERSION} libmujoco.so
        WORKING_DIRECTORY \${CMAKE_INSTALL_PREFIX}/lib
    )
")
      
# Export Mujoco as a dependency
ament_export_include_directories(${MUJOCO_INSTALL_DIR}/include)
ament_export_libraries(mujoco)
ament_package()
