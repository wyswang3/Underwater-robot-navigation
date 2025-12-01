#pragma once

#include <cstddef>
#include <cstdio>
#include <mutex>
#include <string>

namespace nav_core {

class BinLogger {
public:
    BinLogger() = default;

    /// 构造时直接以写模式打开（保持原有行为）
    explicit BinLogger(const std::string& path);

    ~BinLogger();

    /// 以写模式打开（二进制），append=false 覆盖，append=true 追加
    bool open(const std::string& path, bool append);

    /// 以读模式打开（二进制），用于离线解析 .bin 文件
    bool openForRead(const std::string& path);

    /// 关闭文件句柄
    void close();

    /// 写入任意二进制块
    bool write(const void* data, std::size_t size);

    /// 读取固定长度二进制块
    /// 返回 true 表示成功读取 size 字节；false 表示 EOF 或错误
    bool read(void* data, std::size_t size);

    /// 写入 POD 结构（结构内不应含有指针/虚函数等）
    template <typename T>
    bool writePod(const T& pod) {
        return write(&pod, sizeof(T));
    }

    /// 读取 POD 结构；到达 EOF 或出错时返回 false
    template <typename T>
    bool readPod(T& pod) {
        return read(&pod, sizeof(T));
    }

    /// 刷新缓冲
    void flush();

private:
    std::FILE*     fp_  = nullptr;
    std::mutex     mtx_;
};

} // namespace nav_core
