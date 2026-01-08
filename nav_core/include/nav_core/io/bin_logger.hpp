// nav_core/include/nav_core/bin_logger.hpp
#pragma once

#include <cstddef>
#include <cstdio>
#include <mutex>
#include <string>

#include <nav_core/core/types.hpp>

namespace nav_core {

/**
 * @brief 简单高效的二进制日志读写工具
 *
 * 特点：
 *  -线程安全（内部锁）
 *  -支持 append/overwrite
 *  -支持 POD 结构直接写入（禁止内部带指针/虚函数）
 *  -read/write 均为固定长度操作，便于脱机解析
 *
 * 用法：
 *  BinLogger logger("/tmp/imu_20250101.bin");
 *  logger.writePod(sample);
 *  logger.flush();
 */
class BinLogger {
public:
    BinLogger() noexcept = default;

    /// 构造时直接打开文件（写模式，覆盖）
    explicit BinLogger(const std::string& path);

    /// 禁止拷贝（文件句柄无法安全共享）
    BinLogger(const BinLogger&) = delete;
    BinLogger& operator=(const BinLogger&) = delete;

    /// 允许移动（便于容器或成员对象转移）
    BinLogger(BinLogger&& other) noexcept;
    BinLogger& operator=(BinLogger&& other) noexcept;

    ~BinLogger();

    /// 以写模式打开（二进制）
    /// append = false → 覆盖原文件
    /// append = true  → 追加写入
    bool open(const std::string& path, bool append = false);

    /// 以读模式打开（二进制）
    bool openForRead(const std::string& path);

    /// 是否成功打开文件
    bool isOpen() const noexcept { return fp_ != nullptr; }

    /// 关闭文件句柄
    void close() noexcept;

    /// 写入二进制块（线程安全）
    bool write(const void* data, std::size_t size);

    /// 读取固定长度二进制块（线程安全）
    /// 返回 false 表示 EOF 或错误
    bool read(void* data, std::size_t size);

    /// 写入 POD 结构
    template <typename T>
    bool writePod(const T& pod) {
        return write(&pod, sizeof(T));
    }

    /// 读取 POD 结构；到 EOF 时返回 false
    template <typename T>
    bool readPod(T& pod) {
        return read(&pod, sizeof(T));
    }

    /// 刷新输出缓冲
    void flush();

private:
    std::FILE* fp_{nullptr};
    mutable std::mutex mtx_;      ///< 保证读写线程安全

    void swap(BinLogger& other) noexcept;
};

} // namespace nav_core
