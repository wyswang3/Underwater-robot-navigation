# nav_core/CMakeLists.txt
cmake_minimum_required(VERSION 3.14)
project(nav_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(nav_core
    src/eskf.cpp
    src/timebase.cpp
    src/imu_driver.cpp
    src/dvl_driver.cpp
    src/volt_driver.cpp
    src/bin_logger.cpp
)

target_include_directories(nav_core PUBLIC include)

add_executable(uwnav_navd src/nav_daemon.cpp)
target_link_libraries(uwnav_navd PRIVATE nav_core pthread)
