cmake_minimum_required(VERSION 3.14)
project(canevo_sdk_x86 LANGUAGES CXX)

find_package(spdlog REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(CMakePackageConfigHelpers)
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/version" CANEVO_SDK_VERSION LIMIT_COUNT 1)
string(STRIP "${CANEVO_SDK_VERSION}" CANEVO_SDK_VERSION)
if(NOT CANEVO_SDK_VERSION)
  message(FATAL_ERROR "SDK version file is empty: ${CMAKE_CURRENT_SOURCE_DIR}/version")
endif()
set(CANEVO_EXAMPLE_SOURCE_BUILD OFF CACHE BOOL "Build examples with source target instead of find_package")
option(CANEVO_BUILD_EXAMPLES "Build CanEvo SDK examples" ON)

set(_canevo_sdk_top_level OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(_canevo_sdk_top_level ON)
endif()

set(CANEVO_HAVE_NECRO OFF)
find_package(Necro QUIET)
if(Necro_FOUND AND TARGET Necro::qiuniu)
  set(CANEVO_HAVE_NECRO ON)
  message(STATUS "canevo_sdk: NECRO found — linking Necro::qiuniu")
else()
  message(STATUS "canevo_sdk: NECRO not found — using POSIX fallback")
endif()

function(canevo_sdk_enable_necro target_name)
  if(CANEVO_HAVE_NECRO)
    target_compile_definitions(${target_name} PRIVATE CANEVO_HAVE_NECRO=1 __QIUNIU__)
    target_compile_options(${target_name} PRIVATE -fasynchronous-unwind-tables)
    target_link_libraries(${target_name} PRIVATE Necro::qiuniu)
  else()
    target_compile_definitions(${target_name} PRIVATE CANEVO_HAVE_NECRO=0)
  endif()
endfunction()

# 独立构建时才覆盖默认安装前缀
if(_canevo_sdk_top_level AND (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT OR CMAKE_INSTALL_PREFIX STREQUAL "/usr/local"))
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Install path prefix" FORCE)
endif()

add_library(canevo_sdk_x86 SHARED
    src/canevo_impl.cpp
    src/modi_joint_canevo.cpp
)

target_include_directories(canevo_sdk_x86
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
)

target_link_libraries(canevo_sdk_x86
    PRIVATE pthread spdlog::spdlog
)

canevo_sdk_enable_necro(canevo_sdk_x86)

# Make the SDK .so prefer packaged C++ runtime libs.
set_target_properties(canevo_sdk_x86 PROPERTIES
    INSTALL_RPATH "\$ORIGIN/runtime"
)

# ---- 示例 ----
if(CANEVO_BUILD_EXAMPLES)
  add_subdirectory(example)
endif()

# ---- 安装 ----
install(TARGETS canevo_sdk_x86
    EXPORT canevo_sdk_x86Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(FILES
    include/modi_joint_canevo.h
    DESTINATION include
)
install(PROGRAMS
    script/candump_rotate.sh
    DESTINATION script
)
install(DIRECTORY docs/ DESTINATION docs)

execute_process(
    COMMAND "${CMAKE_CXX_COMPILER}" -print-file-name=libstdc++.so.6
    OUTPUT_VARIABLE CANEVO_LIBSTDCXX_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
    COMMAND "${CMAKE_CXX_COMPILER}" -print-file-name=libgcc_s.so.1
    OUTPUT_VARIABLE CANEVO_LIBGCC_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

foreach(_runtime_lib IN ITEMS "${CANEVO_LIBSTDCXX_PATH}" "${CANEVO_LIBGCC_PATH}")
  if(NOT _runtime_lib OR _runtime_lib STREQUAL "libstdc++.so.6" OR _runtime_lib STREQUAL "libgcc_s.so.1" OR NOT EXISTS "${_runtime_lib}")
    message(FATAL_ERROR "Failed to locate runtime library: ${_runtime_lib}")
  endif()
endforeach()

get_filename_component(CANEVO_LIBSTDCXX_REALPATH "${CANEVO_LIBSTDCXX_PATH}" REALPATH)
get_filename_component(CANEVO_LIBGCC_REALPATH "${CANEVO_LIBGCC_PATH}" REALPATH)

install(FILES
    "${CANEVO_LIBSTDCXX_REALPATH}"
    DESTINATION lib/runtime
    RENAME libstdc++.so.6
)
install(FILES
    "${CANEVO_LIBGCC_REALPATH}"
    DESTINATION lib/runtime
    RENAME libgcc_s.so.1
)

install(FILES
    example/CMakeLists.txt
    DESTINATION example
)
install(DIRECTORY
    example/example_single_joint
    example/example_multi_joints
    DESTINATION example
    FILES_MATCHING
      PATTERN "*.cpp"
      PATTERN "*.hpp"
)

install(EXPORT canevo_sdk_x86Targets
    FILE canevo_sdk_x86Targets.cmake
    NAMESPACE canevo_sdk_x86::
    DESTINATION lib/cmake/canevo_sdk_x86
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/canevo_sdk_x86Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/canevo_sdk_x86Config.cmake"
    INSTALL_DESTINATION lib/cmake/canevo_sdk_x86
)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/canevo_sdk_x86ConfigVersion.cmake"
    VERSION "${CANEVO_SDK_VERSION}"
    COMPATIBILITY SameMajorVersion
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/canevo_sdk_x86Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/canevo_sdk_x86ConfigVersion.cmake"
    DESTINATION lib/cmake/canevo_sdk_x86
)

# ---- 打包（生成 make package 目标）----
if(_canevo_sdk_top_level)
set(CPACK_PACKAGE_NAME "modi_sdk")
set(CPACK_PACKAGE_VERSION "${CANEVO_SDK_VERSION}")
set(CPACK_PACKAGE_VENDOR "modi")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "canevo sdk for x86")
set(CPACK_GENERATOR "ZIP")
execute_process(
  COMMAND git rev-parse --short HEAD
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  OUTPUT_VARIABLE GIT_COMMIT_SHORT
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
if(NOT GIT_COMMIT_SHORT)
  set(GIT_COMMIT_SHORT "unknown")
endif()

string(TOLOWER "${CMAKE_SYSTEM_NAME}" CANEVO_PACKAGE_PLATFORM)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(CANEVO_PACKAGE_ARCH "arm64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(CANEVO_PACKAGE_ARCH "x86_64")
else()
  set(CANEVO_PACKAGE_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Keep the standalone SDK artifact naming aligned with package_modi_release.sh:
# modi_sdk_<version>_<platform>_<arch>_<commit>.zip
set(CPACK_PACKAGE_FILE_NAME
    "modi_sdk_${CPACK_PACKAGE_VERSION}_${CANEVO_PACKAGE_PLATFORM}_${CANEVO_PACKAGE_ARCH}_${GIT_COMMIT_SHORT}")
include(CPack)
endif()
