cmake_minimum_required(VERSION 3.14)

project(nav_core
    LANGUAGES C CXX
)

# ================== 通用编译选项 ==================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 默认构建类型：在板子上用 RelWithDebInfo 较合适
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# 编译警告选项（区分 MSVC / GNU/Clang）
if (MSVC)
    add_compile_options(
        /W4            # 较高警告等级
        /permissive-   # 更接近标准 C++
    )
else()
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Wconversion
        -Wsign-conversion
        -Wshadow
        -Wformat=2
        -Wundef
    )
endif()

# 是否编译工具程序（例如 dump_nav_logs）
option(BUILD_NAV_TOOLS "Build nav_core tools (dump_nav_logs etc.)" ON)

# ================== nav_core 库 ==================
#
# 只放“核心模块”：时间基、驱动、滤波、ESKF、日志、导航状态发布等
# 不把可执行程序（nav_daemon / dump_nav_logs）编进库
# =================================================
add_library(nav_core
    src/timebase.cpp
    src/imu_driver_wit.cpp
    src/dvl_driver.cpp
    src/eskf.cpp
    src/bin_logger.cpp
    src/imu_rt_filter.cpp
    src/nav_state_publisher.cpp          # ★ 新增：导航状态发布实现

    third_party/witmotion/wit_c_sdk.c    # 维特官方 C SDK
)

# ================== 头文件路径 ==================
#
# include 结构：
#   nav_core/include/nav_core/*.h
#   D:/UnderwaterRobotSystem/shared/msg/nav_state.hpp
#
# 由于代码里用的是：
#   #include "nav_core/xxx.h"
#   #include "shared/msg/nav_state.hpp"
#
# 所以：
#   - PUBLIC include:  nav_core/include        → "nav_core/xxx.h"
#   - PUBLIC project_root: ../..               → "shared/msg/nav_state.hpp"
#   - PRIVATE witmotion: third_party/witmotion → 官方 SDK 头
# =================================================
target_include_directories(nav_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include   # 提供 "nav_core/xxx.h"
        ${CMAKE_CURRENT_SOURCE_DIR}/../..     # 提供 "shared/msg/nav_state.hpp"
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/witmotion
)

# 线程库
find_package(Threads REQUIRED)
target_link_libraries(nav_core
    PUBLIC
        Threads::Threads
)

# ================== 导航守护进程 uwnav_navd ==================
#
# 主程序文件：src/nav_daemon.cpp
# 仅链接 nav_core，不直接依赖第三方 C SDK / pthread 等
# nav_core 已经通过 PUBLIC include 暴露了 shared/ 和 nav_core/ 头
# ==============================================================
add_executable(uwnav_navd
    src/nav_daemon.cpp
)

target_link_libraries(uwnav_navd
    PRIVATE
        nav_core
)

# ================== 工具程序：dump_nav_logs ==================
#
# 用于离线解析 imu.bin / dvl.bin / eskf.bin
# 源文件已更名为 src/dump_nav_logs.cpp
# ============================================================
if (BUILD_NAV_TOOLS)
    add_executable(dump_nav_logs
        src/dump_nav_logs.cpp
    )

    target_link_libraries(dump_nav_logs
        PRIVATE
            nav_core
    )
endif()
