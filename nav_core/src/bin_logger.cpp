#include "nav_core/bin_logger.h"        // 二进制日志写入

#include <cstdio>
#include <cstring>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <cerrno>

#ifdef _WIN32
#include <direct.h>
#endif

namespace nav_core {

namespace {

// ========== 小工具：判断目录是否存在 ==========
bool dirExists(const std::string& path)
{
    if (path.empty()) {
        return false;
    }

    struct stat st{};
    if (stat(path.c_str(), &st) != 0) {
        return false;
    }
    return (st.st_mode & S_IFDIR) != 0;
}

// ========== 小工具：创建单层目录 ==========
bool makeDir(const std::string& path)
{
    if (path.empty()) {
        return false;
    }
    if (dirExists(path)) {
        return true;
    }

#ifdef _WIN32
    const int ret = _mkdir(path.c_str());
#else
    const int ret = mkdir(path.c_str(), 0755);
#endif

    if (ret != 0 && errno != EEXIST) {
        std::cerr << "[BinLogger] mkdir('" << path << "') failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }
    return true;
}

// ========== 小工具：递归创建目录（mkdir -p）==========
//
// 例：filepath = "logs/2025-11-25/imu.bin"
// 将依次尝试创建：
//   "logs"
//   "logs/2025-11-25"
bool ensureDirForFile(const std::string& filepath)
{
    // 找到最后一个 '/' 或 '\'
    const std::size_t pos = filepath.find_last_of("/\\");
    if (pos == std::string::npos) {
        return true; // 没有目录成分
    }

    const std::string dir = filepath.substr(0, pos);
    if (dir.empty()) {
        return true;
    }

    // 逐级创建 logs / 2025-11-25 / subdir ...
    std::size_t start = 0;
    while (start < dir.size()) {
        const std::size_t next = dir.find_first_of("/\\", start);
        const std::size_t len  = (next == std::string::npos)
                               ? dir.size()
                               : next;

        const std::string sub = dir.substr(0, len);
        if (!sub.empty() && !dirExists(sub)) {
            if (!makeDir(sub)) {
                return false;
            }
        }

        if (next == std::string::npos) {
            break;
        }
        start = next + 1;
    }

    return true;
}

} // anonymous namespace


// ======================================================
//                    BinLogger
// ======================================================

BinLogger::BinLogger(const std::string& path)
{
    open(path, false);
}

BinLogger::~BinLogger()
{
    close();
}

bool BinLogger::open(const std::string& path, bool append)
{
    std::lock_guard<std::mutex> lock(mtx_);

    // 关闭已有文件
    if (fp_) {
        std::fclose(fp_);
        fp_ = nullptr;
    }

    // 自动创建路径：例如 logs/2025-11-25/*
    if (!ensureDirForFile(path)) {
        std::cerr << "[BinLogger] ensureDirForFile('" << path << "') failed\n";
        return false;
    }

    const char* mode = append ? "ab" : "wb";
    fp_ = std::fopen(path.c_str(), mode);

    if (!fp_) {
        std::cerr << "[BinLogger] fopen('" << path << "', '" << mode
                  << "') failed: " << std::strerror(errno) << "\n";
        return false;
    }

    return true;
}

void BinLogger::close()
{
    std::lock_guard<std::mutex> lock(mtx_);

    if (fp_) {
        std::fclose(fp_);
        fp_ = nullptr;
    }
}

bool BinLogger::write(const void* data, std::size_t size)
{
    if (!data || size == 0) {
        return true;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    if (!fp_) {
        return false;
    }

    const std::size_t written = std::fwrite(data, 1, size, fp_);
    if (written != size) {
        // 出现短写，先尝试 flush，再让调用方感知失败
        std::fflush(fp_);
        return false;
    }

    return true;
}

void BinLogger::flush()
{
    std::lock_guard<std::mutex> lock(mtx_);
    if (fp_) {
        std::fflush(fp_);
    }
}

} // namespace nav_core
