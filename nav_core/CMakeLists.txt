cmake_minimum_required(VERSION 3.14)
project(nav_core LANGUAGES C CXX)


# ================== 编译选项 ==================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ================== nav_core 库 ==================
add_library(nav_core
    src/timebase.cpp
    src/imu_driver_wit.cpp
    src/dvl_driver.cpp
    src/eskf.cpp
    src/bin_logger.cpp

    third_party/witmotion/wit_c_sdk.c   # ← 必须是这个相对路径
)


# 头文件路径：
#   - 公共 API 头文件：  nav_core/include/nav_core/*.h
#     调用方式：        #include "nav_core/eskf.h"
#   - WitMotion SDK 头：nav_core/third_party/witmotion/*.h
target_include_directories(nav_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include        # 提供 "nav_core/xxx.h"
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/witmotion
)

# 线程库（Linux/Orange Pi 上使用 pthread，Windows 用 CMake 的 Threads 抽象）
find_package(Threads REQUIRED)
target_link_libraries(nav_core PUBLIC Threads::Threads)

# ================== 导航守护进程 uwnav_navd ==================
add_executable(uwnav_navd
    src/nav_daemon.cpp
)

target_link_libraries(uwnav_navd PRIVATE nav_core)
