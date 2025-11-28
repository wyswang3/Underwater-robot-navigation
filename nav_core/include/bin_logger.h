#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>

namespace nav_core {

/**
 * @brief 简单的二进制日志记录器（单文件）
 *
 * 特点：
 *  - 线程安全（内部使用 mutex）
 *  - 只负责顺序写入原始字节流，不关心数据格式
 *  - 可用于记录 IMU / DVL / ESKF 状态等
 *
 * 用法示例：
 *   BinLogger imu_logger;
 *   imu_logger.open("logs/2025-11-25/imu_raw.bin");
 *   ImuFrame frm{...};
 *   imu_logger.writePod(frm);
 */
class BinLogger {
public:
    BinLogger() = default;
    explicit BinLogger(const std::string& path);

    ~BinLogger();

    /// 打开文件（append=true 时以追加模式打开）
    bool open(const std::string& path, bool append = false);

    /// 关闭文件
    void close();

    /// 是否成功打开
    bool isOpen() const noexcept { return fp_ != nullptr; }

    /// 写入原始字节块（线程安全）
    bool write(const void* data, std::size_t size);

    /// 写入一个 POD 类型（例如 struct，无指针、无虚函数）
    template <typename T>
    bool writePod(const T& value) {
        return write(&value, sizeof(T));
    }

    /// （可选）写一行文本（自动追加换行，主要用于 nav_info.txt 这类简单诊断）
    bool writeLine(const std::string& line);

    /// 强制 flush 到磁盘
    void flush();

private:
    // 禁止拷贝
    BinLogger(const BinLogger&) = delete;
    BinLogger& operator=(const BinLogger&) = delete;

private:
    std::FILE* fp_ = nullptr;
    mutable std::mutex mtx_;
};

} // namespace nav_core
