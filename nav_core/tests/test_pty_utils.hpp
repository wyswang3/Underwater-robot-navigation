#pragma once

#include <chrono>
#include <cerrno>
#include <cstdint>
#include <fcntl.h>
#include <optional>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace test_support {

struct PtyPeer final {
    int         master_fd{-1};
    std::string slave_path{};

    PtyPeer() = default;
    ~PtyPeer() { close(); }

    PtyPeer(const PtyPeer&) = delete;
    PtyPeer& operator=(const PtyPeer&) = delete;

    PtyPeer(PtyPeer&& other) noexcept
        : master_fd(other.master_fd), slave_path(std::move(other.slave_path))
    {
        other.master_fd = -1;
    }

    PtyPeer& operator=(PtyPeer&& other) noexcept
    {
        if (this == &other) {
            return *this;
        }
        close();
        master_fd = other.master_fd;
        slave_path = std::move(other.slave_path);
        other.master_fd = -1;
        return *this;
    }

    static std::optional<PtyPeer> create()
    {
        PtyPeer out{};
        out.master_fd = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (out.master_fd < 0) {
            return std::nullopt;
        }
        if (::grantpt(out.master_fd) != 0 || ::unlockpt(out.master_fd) != 0) {
            out.close();
            return std::nullopt;
        }

        char* slave = ::ptsname(out.master_fd);
        if (slave == nullptr) {
            out.close();
            return std::nullopt;
        }

        out.slave_path = slave;
        return out;
    }

    void close_master()
    {
        if (master_fd >= 0) {
            ::close(master_fd);
            master_fd = -1;
        }
    }

    void close()
    {
        close_master();
    }

    bool write_line(const std::string& line) const
    {
        return write_bytes(reinterpret_cast<const std::uint8_t*>(line.data()), line.size());
    }

    bool write_bytes(const std::vector<std::uint8_t>& bytes) const
    {
        return write_bytes(bytes.data(), bytes.size());
    }

    bool wait_for_tx_bytes(std::chrono::milliseconds timeout) const
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (master_fd < 0) {
                return false;
            }

            std::uint8_t buf[256];
            const ssize_t n = ::read(master_fd, buf, sizeof(buf));
            if (n > 0) {
                return true;
            }
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return false;
    }

    std::vector<std::uint8_t> read_tx_bytes(std::chrono::milliseconds timeout) const
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (master_fd < 0) {
                return {};
            }

            std::uint8_t buf[256];
            const ssize_t n = ::read(master_fd, buf, sizeof(buf));
            if (n > 0) {
                return std::vector<std::uint8_t>(buf, buf + n);
            }
            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                return {};
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return {};
    }

private:
    bool write_bytes(const std::uint8_t* data, std::size_t size) const
    {
        if (master_fd < 0 || data == nullptr) {
            return false;
        }
        return ::write(master_fd, data, size) == static_cast<ssize_t>(size);
    }
};

template <class Pred>
bool wait_until(std::chrono::milliseconds timeout, Pred pred)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return pred();
}

} // namespace test_support
