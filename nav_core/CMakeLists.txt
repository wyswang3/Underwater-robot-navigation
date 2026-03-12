cmake_minimum_required(VERSION 3.14)

project(nav_core LANGUAGES C CXX)

# =========================================
# 1) 通用编译选项
# =========================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4 /permissive-)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 是否编译因子图 / 在线估计等实验模块（默认关闭）
option(NAV_CORE_ENABLE_GRAPH "Build graph smoother / online estimator modules" OFF)
option(NAV_CORE_BUILD_TESTS "Build nav_core unit tests" OFF)

# =========================================
# 2) 依赖库：Eigen / 线程
# =========================================

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)


# =========================================
# 3) 工程根目录（用于共享头文件）
#
# 当前目录层级约定：
#   UnderwaterRobotSystem/
#     ├── shared/                        # 共享消息定义（例如 shared/msg/nav_state.hpp）
#     └── Underwater-robot-navigation/
#         └── nav_core/                  # 本项目（当前 CMakeLists 所在目录）
#
# 因此：
#   CMAKE_CURRENT_SOURCE_DIR = UnderwaterRobotSystem/Underwater-robot-navigation/nav_core
#   UNDERWATER_ROOT          = UnderwaterRobotSystem
#
# 之后代码中可以统一使用：
#   #include "shared/msg/nav_state.hpp"
# 由本处的 include path 解析到：
#   UnderwaterRobotSystem/shared/msg/nav_state.hpp
# =========================================

set(UNDERWATER_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../.."
    CACHE PATH "UnderwaterRobotSystem project root (contains shared/ and Underwater-robot-navigation/)"
)

# =========================================
# 4) 源码列表：只包含“在线导航管线”所需模块
#
# nav_core 负责：
#   - core: 基础类型 / 时间基 / 日志工具
#   - drivers: IMU / DVL 驱动（只负责串口协议，不做物理过滤）
#   - preprocess: IMU / DVL 实时预处理（零偏 / 扣重力 / 噪声地板 / BottomTrack 过滤等）
#   - estimator: ESKF（IMU 传播 + DVL XY/Z 更新 + yaw 伪观测）
#   - io: 二进制日志 + NavState 共享内存发布
#   - app: nav_daemon 配置解析
# =========================================

set(NAV_CORE_SOURCES
    # ---- core ----
    src/nav_core/core/logging.cpp
    src/nav_core/core/timebase.cpp

    # ---- drivers ----
    src/nav_core/drivers/imu_driver_wit.cpp
    src/nav_core/drivers/dvl_protocol.cpp
    src/nav_core/drivers/dvl_driver.cpp
    # ---- third_party: WitMotion 官方 C SDK ----
    third_party/witmotion/wit_c_sdk.c

    # ---- preprocess ----
    # 说明：
    #   - IMU 实时预处理负责：
    #       * 基本有效性检查
    #       * 零偏估计与去除
    #       * 扣重力（输出线加速度，body=FRD）
    #       * 一阶低通 / deadband 抑制噪声
    #   - DVL 实时预处理负责：
    #       * 仅保留 BottomTrack + BottomLock 数据
    #       * 按坐标系区分 BI/BS（体速） 与 BE（地速）
    #       * XY 用 BI body 速度 + 噪声地板
    #       * Z 用 BE ENU 垂向速度 + 噪声地板
    src/nav_core/preprocess/imu_rt_preprocessor.cpp
    src/nav_core/preprocess/dvl_rt_preprocessor.cpp

    # ---- estimator / ESKF ----
    # 说明：
    #   - eskf_core.cpp: 构造 / reset / state 导出等公共逻辑
    #   - eskf_propagate.cpp: 仅包含 IMU 传播（使用“线加速度 + IMU yaw/pitch/roll”）
    #   - eskf_update_dvl.cpp: DVL XY（BI + yaw-only 投影）/ DVL Z（BE vU）更新
    #   - eskf_update_yaw.cpp: yaw 伪观测更新
    #   - eskf.cpp: 统合入口，负责对外接口和整体注释说明
    src/nav_core/estimator/eskf_config_yaml.cpp
    src/nav_core/estimator/eskf_core.cpp
    src/nav_core/estimator/eskf_propagate.cpp
    src/nav_core/estimator/eskf_update_dvl.cpp
    src/nav_core/estimator/eskf_update_yaw.cpp
    src/nav_core/estimator/eskf.cpp

    # ---- io ----
    src/nav_core/io/bin_logger.cpp
    src/nav_core/io/nav_state_publisher.cpp

    # ---- app ----
    src/nav_core/app/nav_runtime_status.cpp
    src/nav_core/app/nav_daemon_config.cpp
)

# 如需要，将因子图 / 在线估计相关模块编译进扩展库（默认关闭）
if(NAV_CORE_ENABLE_GRAPH)
    list(APPEND NAV_CORE_SOURCES
        src/nav_core/estimator/graph_smoother_2d.cpp
        src/nav_core/estimator/nav_health_monitor.cpp
        src/nav_core/estimator/online_estimator.cpp
    )
endif()

# =========================================
# 5) nav_core 静态库
# =========================================

add_library(nav_core STATIC ${NAV_CORE_SOURCES})

# nav_core 对外公开的头文件路径
#  - include/nav_core        : 本库的公有接口
#  - ${UNDERWATER_ROOT}      : 工程根目录（提供 shared/msg/nav_state.hpp 等共享头文件）
#  - third_party/witmotion   : 厂家 IMU SDK
target_include_directories(nav_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${UNDERWATER_ROOT}
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/witmotion
)

target_link_libraries(nav_core
    PUBLIC
        Eigen3::Eigen
        Threads::Threads
        yaml-cpp
        rt
)
# 将 CMake 选项 NAV_CORE_ENABLE_GRAPH 透传为 C++ 宏（0/1），
# 方便在代码中用 #if NAV_CORE_ENABLE_GRAPH 包裹实验性模块。
target_compile_definitions(nav_core
    PUBLIC NAV_CORE_ENABLE_GRAPH=$<BOOL:${NAV_CORE_ENABLE_GRAPH}>
)

# =========================================
# 6) 可执行程序
#
#   1) uwnav_navd：
#        - 在线导航主进程
#        - 只做 wiring：
#            * 加载 nav_daemon.yaml / eskf.yaml
#            * 构造驱动 / 预处理 / ESKF / IO
#            * 调用 run_nav_daemon(...) 进入主循环
#
#   2) uwnav_dvl_selftest：
#        - DVL 自检工具
#        - 用于在下水前检查 DVL 串口通信 / CZ/CS 命令效果
# =========================================

add_executable(uwnav_navd
    src/nav_core/app/nav_daemon.cpp
    src/nav_core/app/nav_daemon_runner.cpp
)
target_link_libraries(uwnav_navd PRIVATE nav_core)

add_executable(uwnav_dvl_selftest
    src/nav_core/app/tools/DVL_selftest.cpp
)
target_link_libraries(uwnav_dvl_selftest PRIVATE nav_core)

# 统一把可执行文件放到 build/bin 目录，方便在板子上运行：
#   ./bin/uwnav_navd
#   ./bin/uwnav_dvl_selftest
foreach(tgt uwnav_navd uwnav_dvl_selftest)
    set_target_properties(${tgt} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
endforeach()

if(NAV_CORE_BUILD_TESTS)
    enable_testing()

    add_executable(test_nav_runtime_status
        tests/test_nav_runtime_status.cpp
    )
    target_link_libraries(test_nav_runtime_status PRIVATE nav_core)
    add_test(NAME nav_core_test_nav_runtime_status COMMAND test_nav_runtime_status)
endif()
