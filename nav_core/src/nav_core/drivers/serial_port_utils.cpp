#include "nav_core/drivers/serial_port_utils.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace nav_core::drivers {
namespace {

speed_t baud_to_termios(int baud)
{
    switch (baud) {
    case 4800:   return B4800;
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
    default:
        return B115200;
    }
}

void set_error(std::string* error, const std::string& value)
{
    if (error != nullptr) {
        *error = value;
    }
}

} // namespace

bool configure_serial_port_raw(int fd,
                               int baud,
                               int read_timeout_ds,
                               std::string* error)
{
    if (fd < 0) {
        set_error(error, "invalid fd");
        return false;
    }

    termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
        set_error(error, std::string("tcgetattr failed: ") + std::strerror(errno));
        return false;
    }

    cfmakeraw(&tio);

    const speed_t speed = baud_to_termios(baud);
    if (cfsetispeed(&tio, speed) != 0 || cfsetospeed(&tio, speed) != 0) {
        set_error(error, std::string("cfsetispeed/cfsetospeed failed: ") + std::strerror(errno));
        return false;
    }

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
#ifdef CRTSCTS
    tio.c_cflag &= ~CRTSCTS;
#endif

    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = static_cast<cc_t>(read_timeout_ds >= 0 ? read_timeout_ds : 0);

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        set_error(error, std::string("tcsetattr failed: ") + std::strerror(errno));
        return false;
    }

    tcflush(fd, TCIOFLUSH);
    return true;
}

int open_serial_port_raw(const std::string& path,
                         int baud,
                         int read_timeout_ds,
                         std::string* error)
{
    const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        set_error(error, std::string("open(") + path + ") failed: " + std::strerror(errno));
        return -1;
    }

    std::string config_error;
    if (!configure_serial_port_raw(fd, baud, read_timeout_ds, &config_error)) {
        ::close(fd);
        set_error(error, config_error);
        return -1;
    }
    return fd;
}

} // namespace nav_core::drivers
