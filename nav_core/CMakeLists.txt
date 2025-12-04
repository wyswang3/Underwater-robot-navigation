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

# 全局基础告警（避免第三方 C 库被 -Wconversion 炮轰）
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
        -Wshadow
        -Wformat=2
        -Wundef
    )
endif()

# 是否编译工具程序（例如 dump_nav_logs）
option(BUILD_NAV_TOOLS "Build nav_core tools (dump_nav_logs etc.)" ON)

# ================== 路径约定 ==================
#
# 仓库结构假定为：
#   Underwater-robot-navigation/
#   ├── shared/
#   │   └── msg/nav_state.hpp
#   └── nav_core/
#       ├── CMakeLists.txt          ← 本文件
#       ├── include/nav_core/*.hpp
#       └── src/*.cpp
#
# 因此：UWSYS_ROOT = nav_core/.. = 仓库根目录
# =================================================
get_filename_component(UWSYS_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)

# ================== nav_core 静态库 ==================
#
# 只放“核心模块”：时间基、驱动、滤波、ESKF、日志、导航状态发布等
# 可执行程序（uwnav_navd / dump_nav_logs）单独 add_executable
# =================================================
add_library(nav_core STATIC
    src/timebase.cpp
    src/imu_driver_wit.cpp
    src/dvl_driver.cpp
    src/eskf.cpp
    src/bin_logger.cpp
    src/imu_rt_filter.cpp
    src/nav_state_publisher.cpp          # 导航状态发布

    third_party/witmotion/wit_c_sdk.c    # 维特官方 C SDK（C 源）
)

# 仅对 C++ 源码启用“严格转换告警”，避免第三方 C 库一堆 -Wconversion 警告
if (NOT MSVC)
    target_compile_options(nav_core
        PRIVATE
            $<$<COMPILE_LANGUAGE:CXX>:-Wconversion -Wsign-conversion>
    )
endif()

# ================== 头文件路径 ==================
#
# 代码中使用：
#   #include "nav_core/xxx.hpp"
#   #include "shared/msg/nav_state.hpp"
#
# 因此需要：
#   - PUBLIC: nav_core/include         → 提供 "nav_core/xxx.hpp"
#   - PUBLIC: UWSYS_ROOT              → 提供 "shared/msg/nav_state.hpp"
#   - PRIVATE: third_party/witmotion  → 官方 SDK 头文件
# =================================================
target_include_directories(nav_core
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include   # nav_core/include
        ${UWSYS_ROOT}                       # 仓库根目录（包含 shared/）
    PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/third_party/witmotion
)

# ================== 链接线程库 &（可选）rt 库 ==================
find_package(Threads REQUIRED)

target_link_libraries(nav_core
    PUBLIC
        Threads::Threads
)

# 部分旧版 Linux 仍要求显式链接 librt（shm_open/timer_XXX）
if (UNIX AND NOT APPLE)
    find_library(RT_LIB rt)
    if (RT_LIB)
        target_link_libraries(nav_core PUBLIC ${RT_LIB})
    endif()
endif()

# ================== 导航守护进程 uwnav_navd ==================
#
# 主程序文件：src/nav_daemon.cpp
# 仅链接 nav_core，所有依赖（头文件路径、线程库等）由 nav_core 传递
# ==============================================================
add_executable(uwnav_navd
    src/nav_daemon.cpp
)

target_link_libraries(uwnav_navd
    PRIVATE
        nav_core
)

