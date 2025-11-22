#include "nav_core/bin_logger.h"

#include <cstdio>
#include <utility>

namespace nav_core {

BinLogger::BinLogger(const std::string& path) {
    open(path, false);
}

BinLogger::~BinLogger() {
    close();
}

bool BinLogger::open(const std::string& path, bool append) {
    std::lock_guard<std::mutex> lock(mtx_);

    if (fp_ != nullptr) {
        std::fclose(fp_);
        fp_ = nullptr;
    }

    const char* mode = append ? "ab" : "wb";
    fp_ = std::fopen(path.c_str(), mode);
    if (!fp_) {
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
    if (!data || size == 0) {
        return true;
    }

    std::lock_guard<std::mutex> lock(mtx_);

    if (!fp_) {
        return false;
    }

    std::size_t written = std::fwrite(data, 1, size, fp_);
    if (written != size) {
        // 写失败，尝试 flush 以尽可能保留已写数据
        std::fflush(fp_);
        return false;
    }

    // 出于性能考虑，这里不每次都 fflush，由上层根据需要调用 flush()
    return true;
}

void BinLogger::flush() {
    std::lock_guard<std::mutex> lock(mtx_);

    if (fp_) {
        std::fflush(fp_);
    }
}

} // namespace nav_core
