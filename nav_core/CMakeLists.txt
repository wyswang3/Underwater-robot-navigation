cmake_minimum_required(VERSION 3.14)

project(nav_core
    LANGUAGES C CXX
)

# ================== 通用编译选项 ==================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 默认构建类型（若未指定），在嵌入式/板子上用 RelWithDebInfo 一般比较合适
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

# 是否编译工具程序（例如 dump_imu_bin）
option(BUILD_NAV_TOOLS "Build nav_core tools (dump_imu_bin etc.)" ON)

# ================== nav_core 库 ==================
#
# 只放“核心模块”：时间基、驱动、滤波、ESKF、日志等
# 不把可执行程序（nav_daemon / dump_imu_bin）编进库
# =================================================
add_library(nav_core
    src/timebase.cpp
    src/imu_driver_wit.cpp
    src/dvl_driver.cpp
    src/eskf.cpp
    src/bin_logger.cpp
    src/imu_rt_filter.cpp

    third_party/witmotion/wit_c_sdk.c   # ← 维特官方 C SDK
)

# 头文件路径：
#   - 公共 API 头文件：  nav_core/include/nav_core/*.h
#     调用方式示例：    #include "nav_core/eskf.h"
#   - WitMotion SDK 头：nav_core/third_party/witmotion/*.h
target_include_directories(nav_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include             # 提供 "nav_core/xxx.h"
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/witmotion
)

# 线程库（Linux/Orange Pi 上使用 pthread，Windows 用 CMake 的 Threads 抽象）
find_package(Threads REQUIRED)
target_link_libraries(nav_core
    PUBLIC
        Threads::Threads
)

# ================== 导航守护进程 uwnav_navd ==================
#
# 主程序文件：src/nav_daemon.cpp
# 仅链接 nav_core，不直接依赖第三方 C SDK / pthread 等
# ==============================================================
add_executable(uwnav_navd
    src/nav_daemon.cpp
)

target_link_libraries(uwnav_navd
    PRIVATE
        nav_core
)

# ================== 工具程序：dump_imu_bin ==================
#
# 用于离线解析 imu.bin / dvl.bin / eskf.bin
# 默认开启（BUILD_NAV_TOOLS=ON），也可以在嵌入式精简构建时关闭
# ============================================================
if (BUILD_NAV_TOOLS)
    add_executable(dump_imu_bin
        src/dump_imu_bin.cpp
    )

    target_link_libraries(dump_imu_bin
        PRIVATE
            nav_core
    )
endif()
