#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>

namespace nav_core {

/**
 * @brief 简单的二进制日志记录器
 *
 * 特点：
 *  - 线程安全（内部使用 mutex）
 *  - 只负责“顺序写入原始字节流”，不关心数据格式
 *  - 可用于记录 IMU/DVL/ESKF 状态等
 *
 * 用法示例：
 *   BinLogger logger("data.bin");
 *   MyPacket pkt{...};
 *   logger.writePod(pkt);
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

    /// 强制 flush 到磁盘
    void flush();

private:
    // 禁止拷贝，只允许移动（如果你需要可以后续实现 move 语义）
    BinLogger(const BinLogger&) = delete;
    BinLogger& operator=(const BinLogger&) = delete;

private:
    std::FILE* fp_ = nullptr;
    mutable std::mutex mtx_;
};

} // namespace nav_core
