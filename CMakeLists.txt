cmake_minimum_required(VERSION 3.14)
project(canevo_sdk_x86 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置安装路径为 build/install（如果未指定）
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT OR CMAKE_INSTALL_PREFIX STREQUAL "/usr/local")
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Install path prefix" FORCE)
endif()

# ---- 库 ----
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
    PRIVATE pthread
)

# ---- 示例 ----
add_subdirectory(example)

# ---- 安装 ----
install(TARGETS canevo_sdk_x86
    EXPORT canevo_sdk_x86Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(FILES include/modi_joint_canevo.h DESTINATION include)

install(EXPORT canevo_sdk_x86Targets
    FILE canevo_sdk_x86Targets.cmake
    NAMESPACE canevo_sdk_x86::
    DESTINATION lib/cmake/canevo_sdk_x86
)
