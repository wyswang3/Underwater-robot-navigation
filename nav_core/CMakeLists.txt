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

option(BUILD_NAV_TOOLS "Build nav_core tools (DVL_selftest etc.)" ON)

# ================== locate UnderwaterRobotSystem root ==================
set(_probe_dir "${CMAKE_CURRENT_LIST_DIR}")
set(UWSYS_ROOT "")
foreach(_i RANGE 0 6)
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

# ================== third_party: witmotion SDK（单独 target + 降噪） ==================
add_library(witmotion_sdk STATIC
    third_party/witmotion/wit_c_sdk.c
)

target_include_directories(witmotion_sdk
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/third_party/witmotion
)

# 仅对官方库淡化告警（不影响你们自己的 C/C++）
if (MSVC)
    target_compile_options(witmotion_sdk PRIVATE /W0)
else()
    target_compile_options(witmotion_sdk PRIVATE
        -Wno-sign-compare
        -Wno-implicit-fallthrough
        -Wno-restrict
        -Wno-maybe-uninitialized
    )
endif()

# ================== nav_core 静态库（按当前 src/ 目录结构） ==================
add_library(nav_core STATIC
    # ---- core ----
    src/nav_core/core/logging.cpp
    src/nav_core/core/timebase.cpp

    # ---- drivers ----
    src/nav_core/drivers/imu_driver_wit.cpp
    src/nav_core/drivers/dvl_driver.cpp
    src/nav_core/drivers/dvl_protocol.cpp

    # ---- filters ----
    src/nav_core/filters/imu_rt_filter.cpp
    src/nav_core/filters/eskf.cpp

    # ---- estimator ----
    src/nav_core/estimator/online_estimator.cpp
    src/nav_core/estimator/graph_smoother_2d.cpp
    src/nav_core/estimator/nav_health_monitor.cpp

    # ---- io ----
    src/nav_core/io/bin_logger.cpp
    src/nav_core/io/nav_state_publisher.cpp

    # 如后续增加 nav_daemon_config.cpp 等，可以在这里继续补充
    # ---- app ----
    src/nav_core/app/nav_daemon_config.cpp
)

# shared_msgs：提供 shared/msg/*.hpp include，并作为跨工程一致依赖
target_link_libraries(nav_core PUBLIC shared_msgs)

# 链接 witmotion 官方 SDK（warning 已隔离）
target_link_libraries(nav_core PRIVATE witmotion_sdk)

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
)

# ================== 链接线程库 &（可选）rt 库 ==================
find_package(Threads REQUIRED)
target_link_libraries(nav_core PUBLIC Threads::Threads)
find_package(yaml-cpp REQUIRED)
target_link_libraries(nav_core PUBLIC yaml-cpp)


if (UNIX AND NOT APPLE)
    find_library(RT_LIB rt)
    if (RT_LIB)
        target_link_libraries(nav_core PUBLIC ${RT_LIB})
    endif()
endif()

# ================== 导航守护进程 uwnav_navd ==================
add_executable(uwnav_navd
    src/nav_core/app/nav_daemon.cpp
)
target_link_libraries(uwnav_navd PRIVATE nav_core)

# ================== 工具：DVL_selftest ==================
if (BUILD_NAV_TOOLS)
    add_executable(uwnav_dvl_selftest
        src/nav_core/app/tools/DVL_selftest.cpp
    )
    target_link_libraries(uwnav_dvl_selftest PRIVATE nav_core)
endif()
