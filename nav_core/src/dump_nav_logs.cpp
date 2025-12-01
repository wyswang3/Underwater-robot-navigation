#include <iostream>
#include <string>
#include <iomanip>

#include "nav_core/bin_logger.h"        // 二进制日志写入
#include "nav_core/log_packets.h"    // ImuLogPacket / DvlLogPacket / EskfLogPacket

using namespace nav_core;

static void print_usage(const char* exe)
{
    std::cout << "Usage: " << exe << " <bin_file> [--imu|--dvl|--eskf]\n"
              << "Example:\n"
              << "  " << exe << " imu.bin --imu\n";
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    std::string path = argv[1];
    std::string mode = "--imu";   // 默认按 IMU 包解析

    if (argc >= 3) {
        mode = argv[2];
    }

    BinLogger reader;
    if (!reader.openForRead(path)) {
        std::cerr << "Failed to open " << path << "\n";
        return 1;
    }

    std::cout << std::fixed << std::setprecision(6);

    if (mode == "--imu") {
        ImuLogPacket pkt;
        std::cout << "# mono_ns, est_ns, ax, ay, az, gx, gy, gz, roll, pitch, yaw, temp, valid\n";

        while (reader.readPod(pkt)) {
            std::cout << pkt.mono_ns << ","
                      << pkt.est_ns  << ","
                      << pkt.lin_acc[0] << ","
                      << pkt.lin_acc[1] << ","
                      << pkt.lin_acc[2] << ","
                      << pkt.ang_vel[0] << ","
                      << pkt.ang_vel[1] << ","
                      << pkt.ang_vel[2] << ","
                      << pkt.euler[0]   << ","
                      << pkt.euler[1]   << ","
                      << pkt.euler[2]   << ","
                      << pkt.temperature << ","
                      << int(pkt.valid)
                      << "\n";
        }
    }
    else if (mode == "--dvl") {
        DvlLogPacket pkt;
        std::cout << "# mono_ns, est_ns, vx, vy, vz, valid, quality\n";

        while (reader.readPod(pkt)) {
            std::cout << pkt.mono_ns << ","
                      << pkt.est_ns  << ","
                      << pkt.vel[0]  << ","
                      << pkt.vel[1]  << ","
                      << pkt.vel[2]  << ","
                      << pkt.valid   << ","
                      << pkt.quality
                      << "\n";
        }
    }
    else if (mode == "--eskf") {
        EskfLogPacket pkt;
        std::cout << "# mono_ns, est_ns, px, py, pz, vx, vy, vz, roll, pitch, yaw, bax, bay, baz, bgx, bgy, bgz, valid, status\n";

        while (reader.readPod(pkt)) {
            std::cout << pkt.mono_ns << ","
                      << pkt.est_ns  << ","
                      << pkt.pos[0]  << ","
                      << pkt.pos[1]  << ","
                      << pkt.pos[2]  << ","
                      << pkt.vel[0]  << ","
                      << pkt.vel[1]  << ","
                      << pkt.vel[2]  << ","
                      << pkt.euler[0]<< ","
                      << pkt.euler[1]<< ","
                      << pkt.euler[2]<< ","
                      << pkt.bias_accel[0] << ","
                      << pkt.bias_accel[1] << ","
                      << pkt.bias_accel[2] << ","
                      << pkt.bias_gyro[0]  << ","
                      << pkt.bias_gyro[1]  << ","
                      << pkt.bias_gyro[2]  << ","
                      << int(pkt.valid)    << ","
                      << pkt.status
                      << "\n";
        }
    }
    else {
        print_usage(argv[0]);
        return 1;
    }

    return 0;
}
