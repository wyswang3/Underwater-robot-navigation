// nav_core/src/bin_logger.cpp
//
// 二进制日志工具：
//   - 线程安全、支持追加/覆盖写；
//   - 自动递归创建目录（类似 mkdir -p）；
//   - 支持简单的按 POD 结构写入/读取；
//   - 不依赖任何第三方库，Control 侧和 Nav 侧都能直接复用。

#include <nav_core/bin_logger.hpp>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include <sys/stat.h>
#include <sys/types.h>

#ifdef _WIN32
#  include <direct.h>
#endif

namespace nav_core {

namespace {

// ================= 工具函数：目录操作 =================

// 判断目录是否存在
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

// 创建单层目录
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
    const int ret = ::mkdir(path.c_str(), 0755);
#endif

    if (ret != 0 && errno != EEXIST) {
        std::cerr << "[BinLogger] mkdir('" << path << "') failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }
    return true;
}

// 递归创建文件所在目录（类似 mkdir -p）
//
// 例： filepath = "logs/2025-11-25/imu.bin"
// 将依次尝试创建：
//   "logs"
//   "logs/2025-11-25"
bool ensureDirForFile(const std::string& filepath)
{
    const std::size_t pos = filepath.find_last_of("/\\");
    if (pos == std::string::npos) {
        // 没有目录成分，直接返回
        return true;
    }

    const std::string dir = filepath.substr(0, pos);
    if (dir.empty()) {
        return true;
    }

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

// ============================================================================
//                            BinLogger 实现
// ============================================================================

BinLogger::BinLogger(const std::string& path)
{
    // 默认构造行为：直接以“覆盖写”方式打开
    open(path, /*append=*/false);
}

BinLogger::~BinLogger()
{
    close();
}

bool BinLogger::open(const std::string& path, bool append)
{
    std::lock_guard<std::mutex> lock(mtx_);

    // 先关闭已有文件
    if (fp_) {
        std::fclose(fp_);
        fp_ = nullptr;
    }

    // 写模式：自动创建上层目录
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

bool BinLogger::openForRead(const std::string& path)
{
    std::lock_guard<std::mutex> lock(mtx_);

    // 关闭已有文件
    if (fp_) {
        std::fclose(fp_);
        fp_ = nullptr;
    }

    fp_ = std::fopen(path.c_str(), "rb");
    if (!fp_) {
        std::cerr << "[BinLogger] fopen('" << path << "', 'rb') failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }

    return true;
}

void BinLogger::close() noexcept
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
        // 空写视作成功，便于调用方少做判断
        return true;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    if (!fp_) {
        return false;
    }

    const std::size_t written = std::fwrite(data, 1, size, fp_);
    if (written != size) {
        // 出现短写：尽量 flush 一下，再把失败结果反馈给上层
        std::fflush(fp_);
        std::cerr << "[BinLogger] short write: expect " << size
                  << " bytes, got " << written << "\n";
        return false;
    }

    return true;
}

bool BinLogger::read(void* data, std::size_t size)
{
    if (!data || size == 0) {
        return true;
    }

    std::lock_guard<std::mutex> lock(mtx_);
    if (!fp_) {
        return false;
    }

    const std::size_t n = std::fread(data, 1, size, fp_);
    if (n != size) {
        if (std::feof(fp_)) {
            // 正常 EOF：调用方据返回值 false 判断结束
            return false;
        }
        if (std::ferror(fp_)) {
            std::cerr << "[BinLogger] read() error: "
                      << std::strerror(errno) << "\n";
            return false;
        }
        // 其他情况也按失败处理
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
