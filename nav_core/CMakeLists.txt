cmake_minimum_required(VERSION 3.14)
project(nav_core LANGUAGES C CXX)

# ================== 通用编译选项 ==================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

if (MSVC)
    add_compile_options(/W4 /permissive-)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wformat=2 -Wundef)
endif()

option(BUILD_NAV_TOOLS "Build nav_core tools (dump_nav_logs etc.)" ON)

# ================== shared_msgs（来自 UnderwaterRobotSystem/shared） ==================
#
# 期望目录关系：
# UnderwaterRobotSystem/
# ├── shared/
# └── Underwater-robot-navigation/
#     └── nav_core/   (本文件)
#
# nav_core -> 上三级到 UnderwaterRobotSystem
# ================== locate UnderwaterRobotSystem root ==================
# Find the nearest ancestor that contains shared/CMakeLists.txt
set(_probe_dir "${CMAKE_CURRENT_LIST_DIR}")
set(UWSYS_ROOT "")
foreach(_i RANGE 0 6)  # 最多往上找 7 层，足够覆盖你当前结构
    if (EXISTS "${_probe_dir}/shared/CMakeLists.txt")
        set(UWSYS_ROOT "${_probe_dir}")
        break()
    endif()
    get_filename_component(_probe_dir "${_probe_dir}/.." ABSOLUTE)
endforeach()

if (UWSYS_ROOT STREQUAL "")
    message(FATAL_ERROR
        "[nav_core] Cannot locate repo root containing shared/CMakeLists.txt.\n"
        "  start from: ${CMAKE_CURRENT_LIST_DIR}\n"
        "  please check directory layout."
    )
endif()

set(UWSYS_SHARED_DIR "${UWSYS_ROOT}/shared")
message(STATUS "[nav_core] UWSYS_ROOT       = ${UWSYS_ROOT}")
message(STATUS "[nav_core] UWSYS_SHARED_DIR = ${UWSYS_SHARED_DIR}")


# 避免重复 add_subdirectory(shared)（例如超级工程/多子项目同时构建）
if (NOT TARGET shared_msgs)
    if (NOT EXISTS "${UWSYS_SHARED_DIR}/CMakeLists.txt")
        message(FATAL_ERROR
            "[nav_core] shared/ not found at: ${UWSYS_SHARED_DIR}\n"
            "Check your repo layout or UWSYS_ROOT computation."
        )
    endif()

    add_subdirectory(
        "${UWSYS_SHARED_DIR}"
        "${CMAKE_BINARY_DIR}/_deps/shared_msgs-build"
    )
endif()

# ================== nav_core 静态库 ==================
add_library(nav_core STATIC
    src/timebase.cpp
    src/imu_driver_wit.cpp
    src/dvl_driver.cpp
    src/eskf.cpp
    src/bin_logger.cpp
    src/imu_rt_filter.cpp
    src/nav_state_publisher.cpp
    third_party/witmotion/wit_c_sdk.c
)

# shared_msgs：提供 shared/msg/*.hpp include，并作为跨工程一致依赖
target_link_libraries(nav_core PUBLIC shared_msgs)

# ================== 仅对 C++ 源码启用更严格告警 ==================
if (NOT MSVC)
    target_compile_options(nav_core PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-Wconversion -Wsign-conversion>
    )
endif()

# ================== 头文件路径 ==================
target_include_directories(nav_core
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/third_party/witmotion
)

# ================== 链接线程库 &（可选）rt 库 ==================
find_package(Threads REQUIRED)
target_link_libraries(nav_core PUBLIC Threads::Threads)

if (UNIX AND NOT APPLE)
    find_library(RT_LIB rt)
    if (RT_LIB)
        target_link_libraries(nav_core PUBLIC ${RT_LIB})
    endif()
endif()

# ================== 导航守护进程 uwnav_navd ==================
add_executable(uwnav_navd src/nav_daemon.cpp)
target_link_libraries(uwnav_navd PRIVATE nav_core)
