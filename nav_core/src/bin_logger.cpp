#include "bin_logger.h"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#ifdef _WIN32
#include <direct.h>
#endif

namespace nav_core {

// ========== 小工具：判断目录是否存在 ==========
static bool dirExists(const std::string& path) {
    if (path.empty()) return false;

    struct stat st{};
    if (stat(path.c_str(), &st) != 0) {
        return false;
    }
    return (st.st_mode & S_IFDIR) != 0;
}

// ========== 小工具：创建单层目录 ==========
static bool makeDir(const std::string& path) {
    if (path.empty()) return false;
    if (dirExists(path)) return true;

#ifdef _WIN32
    int ret = _mkdir(path.c_str());
#else
    int ret = mkdir(path.c_str(), 0755);
#endif

    if (ret != 0 && errno != EEXIST) {
        std::cerr << "[BinLogger] mkdir(" << path << ") failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }
    return true;
}

// ========== 小工具：递归创建目录（mkdir -p）==========
static bool ensureDirForFile(const std::string& filepath) {
    // 找到最后一个 / 或 \
    std::size_t pos = filepath.find_last_of("/\\");
    if (pos == std::string::npos) return true;

    std::string dir = filepath.substr(0, pos);
    if (dir.empty()) return true;

    // 逐级创建 logs / 2025-11-25 / subdir
    std::size_t start = 0;
    while (start < dir.size()) {
        std::size_t next = dir.find_first_of("/\\", start);
        std::string sub;

        if (next == std::string::npos) {
            sub = dir.substr(0, dir.size());
            start = dir.size();
        } else {
            sub = dir.substr(0, next);
            start = next + 1;
        }

        if (!sub.empty() && !dirExists(sub)) {
            if (!makeDir(sub)) return false;
        }
    }

    return true;
}

// ======================================================
//                    BinLogger
// ======================================================

BinLogger::BinLogger(const std::string& path) {
    open(path, false);
}

BinLogger::~BinLogger() {
    close();
}

bool BinLogger::open(const std::string& path, bool append) {
    std::lock_guard<std::mutex> lock(mtx_);

    // 关闭已有文件
    if (fp_) {
        std::fclose(fp_);
        fp_ = nullptr;
    }

    // 自动创建路径：logs/2025-11-25/*
    if (!ensureDirForFile(path)) {
        std::cerr << "[BinLogger] ensureDirForFile(" << path << ") failed\n";
        return false;
    }

    const char* mode = append ? "ab" : "wb";
    fp_ = std::fopen(path.c_str(), mode);

    if (!fp_) {
        std::cerr << "[BinLogger] fopen(" << path << ", " << mode << ") failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }

    return true;
}

void BinLogger::close() {
    std::lock_guard<std::mutex> lock(mtx_);

    if (fp_) {
        std::fclose(fp_);
        fp_ = nullptr;
    }
}

bool BinLogger::write(const void* data, std::size_t size) {
    if (!data || size == 0) return true;

    std::lock_guard<std::mutex> lock(mtx_);
    if (!fp_) return false;

    std::size_t written = std::fwrite(data, 1, size, fp_);
    if (written != size) {
        std::fflush(fp_);
        return false;
    }

    return true;
}

void BinLogger::flush() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (fp_) std::fflush(fp_);
}

} // namespace nav_core
