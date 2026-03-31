#include "nav_core/app/nav_daemon_devices.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <sstream>
#include <string>

#include "nav_core/io/log_packets.hpp"

namespace nav_core::app {
namespace {

MonoTimeNs monotonic_now_ns()
{
    using clock = std::chrono::steady_clock;
    const auto now = clock::now().time_since_epoch();
    return static_cast<MonoTimeNs>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

std::int64_t offline_timeout_ns(const drivers::SerialBindingConfig& cfg) noexcept
{
    return static_cast<std::int64_t>(cfg.offline_timeout_ms) * 1000000ll;
}

std::uint16_t device_trace_flags(DeviceConnectionState state) noexcept
{
    switch (state) {
    case DeviceConnectionState::ONLINE:
        return io::kTimingTraceDeviceOnline;
    case DeviceConnectionState::MISMATCH:
        return io::kTimingTraceDeviceMismatch;
    case DeviceConnectionState::PROBING:
    case DeviceConnectionState::CONNECTING:
    case DeviceConnectionState::ERROR_BACKOFF:
    case DeviceConnectionState::RECONNECTING:
        return io::kTimingTraceDeviceReconnecting;
    case DeviceConnectionState::DISCONNECTED:
    default:
        return io::kTimingTraceNone;
    }
}

void report_device_transition(const char* label,
                              io::TimingTraceKind trace_kind,
                              ManagedDeviceRuntime& runtime,
                              nav_core::BinLogger* timing_logger,
                              NavEventCsvLogger* event_logger)
{
    const auto& status = runtime.binder.status();
    if (status.last_transition_ns == runtime.last_reported_transition_ns &&
        status.state == runtime.last_reported_state) {
        return;
    }

    runtime.last_reported_transition_ns = status.last_transition_ns;
    runtime.last_reported_state = status.state;

    std::fprintf(stderr,
                 "[nav_daemon] %s state=%s path=%s reason=%s\n",
                 label,
                 device_state_name(status.state),
                 status.active_path.empty() ? "<none>" : status.active_path.c_str(),
                 status.reason.empty() ? "<none>" : status.reason.c_str());

    if (timing_logger != nullptr) {
        io::TimingTracePacketV1 pkt{};
        pkt.kind = static_cast<std::uint16_t>(trace_kind);
        pkt.flags = device_trace_flags(status.state);
        pkt.publish_mono_ns = status.last_transition_ns;
        pkt.age_ms = kAgeUnknownMs;
        pkt.fault_code = static_cast<std::uint32_t>(status.state);
        timing_logger->writePod(pkt);
    }

    if (event_logger != nullptr) {
        event_logger->log_device_bind_state_changed(status.last_transition_ns, status);
    }
}

bool start_imu_driver_on_path(const NavDaemonConfig& cfg,
                              SharedSensorState& shared_state,
                              drivers::ImuDriverWit& imu_driver,
                              NavEventCsvLogger* event_logger,
                              const std::string& path)
{
    drivers::ImuConfig dcfg = cfg.imu.driver;
    dcfg.port = path;

    const auto on_frame = make_imu_frame_callback(shared_state);
    drivers::ImuDriverWit::RawCallback on_raw;

    if (!imu_driver.init(dcfg, on_frame, on_raw)) {
        std::fprintf(stderr,
                     "[nav_daemon] ERROR: ImuDriverWit::init failed (port=%s, baud=%d)\n",
                     dcfg.port.c_str(),
                     dcfg.baud);
        std::fprintf(stderr,
                     "[nav_daemon] HINT: 请检查 IMU 串口号 / 波特率 / 设备权限 / 设备是否插好\n");
        if (event_logger != nullptr) {
            event_logger->log_serial_open_failed(
                monotonic_now_ns(), "imu", dcfg.port, dcfg.baud, "driver_init_failed");
        }
        return false;
    }
    if (!imu_driver.start()) {
        std::fprintf(stderr, "[nav_daemon] ERROR: ImuDriverWit::start failed\n");
        std::fprintf(stderr,
                     "[nav_daemon] HINT: 请检查串口是否被其它进程占用，或驱动内部异常\n");
        if (event_logger != nullptr) {
            event_logger->log_serial_open_failed(
                monotonic_now_ns(), "imu", dcfg.port, dcfg.baud, "driver_start_failed");
        }
        return false;
    }

    std::fprintf(stderr,
                 "[nav_daemon] IMU driver started on %s @ %d\n",
                 dcfg.port.c_str(),
                 dcfg.baud);
    return true;
}

bool start_dvl_driver_on_path(const NavDaemonConfig& cfg,
                              SharedSensorState& shared_state,
                              drivers::DvlDriver& dvl_driver,
                              NavEventCsvLogger* event_logger,
                              const std::string& path)
{
    drivers::DvlConfig dcfg = cfg.dvl.driver;
    dcfg.port = path;

    const auto on_raw = make_dvl_raw_callback(shared_state);

    if (!dvl_driver.init(dcfg, on_raw)) {
        std::fprintf(stderr,
                     "[nav_daemon] ERROR: DvlDriver::init failed (port=%s, baud=%d)\n",
                     dcfg.port.c_str(),
                     dcfg.baud);
        std::fprintf(stderr,
                     "[nav_daemon] HINT: 请检查 DVL 串口 / 波特率 / 供电 / 厂家协议配置\n");
        if (event_logger != nullptr) {
            event_logger->log_serial_open_failed(
                monotonic_now_ns(), "dvl", dcfg.port, dcfg.baud, "driver_init_failed");
        }
        return false;
    }

    if (!dvl_driver.start()) {
        std::fprintf(stderr, "[nav_daemon] ERROR: DvlDriver::start failed\n");
        if (event_logger != nullptr) {
            event_logger->log_serial_open_failed(
                monotonic_now_ns(), "dvl", dcfg.port, dcfg.baud, "driver_start_failed");
        }
        return false;
    }

    std::fprintf(stderr,
                 "[nav_daemon] DVL driver started on %s @ %d\n",
                 dcfg.port.c_str(),
                 dcfg.baud);
    return true;
}

void log_imu_no_frame_after_connect(const NavDaemonConfig& cfg,
                                    const ManagedDeviceRuntime& runtime,
                                    const drivers::ImuDriverWit& imu_driver,
                                    NavEventCsvLogger* event_logger,
                                    MonoTimeNs now_mono_ns)
{
    const auto& status = runtime.binder.status();
    const auto waited_ms =
        (status.last_transition_ns > 0 && now_mono_ns > status.last_transition_ns)
            ? static_cast<long long>((now_mono_ns - status.last_transition_ns) / 1000000ll)
            : static_cast<long long>(cfg.imu.driver.binding.offline_timeout_ms);

    const std::string& path =
        status.active_path.empty() ? cfg.imu.driver.port : status.active_path;

    std::fprintf(stderr,
                 "[nav_daemon] IMU opened but no parseable frame arrived within %lld ms "
                 "(path=%s, baud=%d, addr=0x%02x)\n",
                 waited_ms,
                 path.c_str(),
                 cfg.imu.driver.baud,
                 static_cast<unsigned int>(cfg.imu.driver.slave_addr));

    const auto serial_diag = imu_driver.serialDebugSnapshot();
    std::fprintf(stderr,
                 "[nav_daemon] IMU serial diagnostic: kind=%s rx_bytes=%zu summary=%s\n",
                 drivers::imu_serial_peer_kind_name(serial_diag.peer_kind),
                 serial_diag.total_rx_bytes,
                 serial_diag.summary.c_str());
    if (!serial_diag.preview_text.empty()) {
        std::fprintf(stderr,
                     "[nav_daemon] IMU serial preview(text): %s\n",
                     serial_diag.preview_text.c_str());
    }
    if (!serial_diag.preview_hex.empty()) {
        std::fprintf(stderr,
                     "[nav_daemon] IMU serial preview(hex): %s\n",
                     serial_diag.preview_hex.c_str());
    }
    if (!serial_diag.frame_dump_hex.empty()) {
        std::fprintf(stderr,
                     "[nav_daemon] IMU serial frame_dump(hex): %s\n",
                     serial_diag.frame_dump_hex.c_str());
    }

    if (event_logger != nullptr &&
        (serial_diag.total_rx_bytes > 0 || !serial_diag.summary.empty())) {
        event_logger->log_protocol_diagnostic(
            now_mono_ns,
            "imu",
            drivers::imu_serial_peer_kind_name(serial_diag.peer_kind),
            serial_diag.summary,
            serial_diag.preview_text,
            serial_diag.preview_hex);
    }

    if (serial_diag.peer_kind == drivers::ImuSerialPeerKind::kVolt32Ascii) {
        std::fprintf(stderr,
                     "[nav_daemon] HINT: 当前串口更像 Volt32 文本电压板；"
                     "请检查 IMU/Volt 串口是否发生重枚举跳变\n");
    } else if (serial_diag.peer_kind == drivers::ImuSerialPeerKind::kImuModbusReply) {
        std::fprintf(stderr,
                     "[nav_daemon] HINT: 已经看到 IMU Modbus 回复，但 SDK 仍未产出寄存器更新；"
                     "请检查寄存器窗口、slave_addr 和设备协议模式\n");
    } else {
        std::fprintf(stderr,
                     "[nav_daemon] HINT: 串口已打开，但 IMU 没有返回可解析的 Modbus 数据；"
                     "请检查供电、RS485 A/B、波特率、slave_addr 和 USB-RS485 转接器\n");
    }
}

} // namespace

void clear_imu_shared_state(SharedSensorState& shared_state)
{
    std::lock_guard<std::mutex> lock(shared_state.m);
    shared_state.last_imu.reset();
    shared_state.last_imu_mono_ns = 0;
}

void clear_dvl_shared_state(SharedSensorState& shared_state)
{
    std::lock_guard<std::mutex> lock(shared_state.m);
    shared_state.last_dvl_raw.reset();
    shared_state.last_dvl_mono_ns = 0;
}

drivers::ImuDriverWit::FrameCallback make_imu_frame_callback(SharedSensorState& shared_state)
{
    return [&shared_state](const ImuFrame& frame) {
        MonoTimeNs mono_ns = frame.mono_ns;
        if (mono_ns <= 0) {
            mono_ns = monotonic_now_ns();
        }

        std::lock_guard<std::mutex> lock(shared_state.m);
        shared_state.last_imu = frame;
        shared_state.last_imu_mono_ns = mono_ns;
    };
}

drivers::DvlDriver::RawCallback make_dvl_raw_callback(SharedSensorState& shared_state)
{
    using drivers::DvlCoordFrame;
    using drivers::DvlRawData;
    using preprocess::DvlRawSample;

    return [&shared_state](const DvlRawData& raw) {
        DvlRawSample sample{};
        sample.sensor_time_ns = raw.sensor_time_ns;
        sample.recv_mono_ns = raw.recv_mono_ns;
        sample.consume_mono_ns = 0;
        sample.mono_ns = raw.mono_ns;
        sample.est_ns = raw.est_ns;

        const auto& parsed = raw.parsed;
        std::uint8_t kind = 0;
        if (parsed.src == "BI") {
            kind = 0;
        } else if (parsed.src == "BE") {
            kind = 1;
        } else if (parsed.src == "BS") {
            kind = 2;
        } else if (parsed.src == "BD") {
            kind = 3;
        }
        sample.kind = kind;
        sample.bottom_lock = parsed.bottom_lock;

        sample.vel_inst_mps = {0.f, 0.f, 0.f};
        sample.vel_earth_mps = {0.f, 0.f, 0.f};
        if (parsed.coord_frame == DvlCoordFrame::Earth) {
            sample.vel_earth_mps.x = static_cast<float>(parsed.ve_mps());
            sample.vel_earth_mps.y = static_cast<float>(parsed.vn_mps());
            sample.vel_earth_mps.z = static_cast<float>(parsed.vu_mps());
        } else {
            sample.vel_inst_mps.x = static_cast<float>(parsed.ve_mps());
            sample.vel_inst_mps.y = static_cast<float>(parsed.vn_mps());
            sample.vel_inst_mps.z = static_cast<float>(parsed.vu_mps());
        }

        std::lock_guard<std::mutex> lock(shared_state.m);
        shared_state.last_dvl_raw = sample;
        shared_state.last_dvl_mono_ns = sample.mono_ns;
    };
}

bool service_imu_device(const NavDaemonConfig& cfg,
                        SharedSensorState& shared_state,
                        drivers::ImuDriverWit& imu_driver,
                        ManagedDeviceRuntime& runtime,
                        nav_core::BinLogger* timing_logger,
                        NavEventCsvLogger* event_logger,
                        MonoTimeNs now_mono_ns)
{
    bool reset_pipeline = false;
    const auto timeout_ns = offline_timeout_ns(cfg.imu.driver.binding);

    if (runtime.binder.status().state == DeviceConnectionState::ONLINE) {
        bool offline = false;
        std::string reason;

        if (!imu_driver.isRunning()) {
            offline = true;
            reason = "driver thread stopped";
        } else if (!imu_driver.isPortOpen()) {
            offline = true;
            reason = "serial port closed";
        } else if (timeout_ns > 0) {
            if (!imu_driver.hasEverReceivedFrame()) {
                if (runtime.binder.status().last_transition_ns > 0 &&
                    now_mono_ns - runtime.binder.status().last_transition_ns > timeout_ns) {
                    offline = true;
                    reason = "no IMU frame after connect";
                }
            } else {
                const MonoTimeNs last_frame_ns = imu_driver.lastFrameMonoNs();
                if (last_frame_ns > 0 && now_mono_ns - last_frame_ns > timeout_ns) {
                    offline = true;
                    reason = "IMU frame timeout";
                }
            }
        }

        if (offline) {
            if (reason == "no IMU frame after connect") {
                log_imu_no_frame_after_connect(cfg, runtime, imu_driver, event_logger, now_mono_ns);
            }
            imu_driver.stop();
            clear_imu_shared_state(shared_state);
            runtime.binder.mark_disconnected(now_mono_ns, reason);
            reset_pipeline = true;
        }
    }

    if (runtime.binder.should_probe(now_mono_ns)) {
        const auto device = runtime.binder.probe(now_mono_ns);
        if (device.has_value()) {
            imu_driver.stop();
            clear_imu_shared_state(shared_state);

            if (!start_imu_driver_on_path(cfg, shared_state, imu_driver, event_logger, device->path)) {
                runtime.binder.mark_connect_failure(now_mono_ns, "driver start failed");
            } else {
                runtime.binder.mark_connect_success(now_mono_ns, *device);
                reset_pipeline = true;
            }
        }
    }

    report_device_transition(
        "IMU", io::TimingTraceKind::kImuDeviceState, runtime, timing_logger, event_logger);
    return reset_pipeline;
}

bool service_dvl_device(const NavDaemonConfig& cfg,
                        SharedSensorState& shared_state,
                        drivers::DvlDriver& dvl_driver,
                        ManagedDeviceRuntime& runtime,
                        nav_core::BinLogger* timing_logger,
                        NavEventCsvLogger* event_logger,
                        MonoTimeNs now_mono_ns)
{
    bool reset_cache = false;
    const auto timeout_ns = offline_timeout_ns(cfg.dvl.driver.binding);

    if (runtime.binder.status().state == DeviceConnectionState::ONLINE) {
        bool offline = false;
        std::string reason;

        if (!dvl_driver.running()) {
            offline = true;
            reason = "driver thread stopped";
        } else if (!dvl_driver.isPortOpen()) {
            offline = true;
            reason = "serial port closed";
        } else if (timeout_ns > 0) {
            const MonoTimeNs last_rx_ns = dvl_driver.lastRxMonoNs();
            if (last_rx_ns <= 0) {
                if (runtime.binder.status().last_transition_ns > 0 &&
                    now_mono_ns - runtime.binder.status().last_transition_ns > timeout_ns) {
                    offline = true;
                    reason = "no DVL frame after connect";
                }
            } else if (now_mono_ns - last_rx_ns > timeout_ns) {
                offline = true;
                reason = "DVL frame timeout";
            }
        }

        if (offline) {
            dvl_driver.stop();
            clear_dvl_shared_state(shared_state);
            runtime.binder.mark_disconnected(now_mono_ns, reason);
            reset_cache = true;
        }
    }

    if (runtime.binder.should_probe(now_mono_ns)) {
        const auto device = runtime.binder.probe(now_mono_ns);
        if (device.has_value()) {
            dvl_driver.stop();
            clear_dvl_shared_state(shared_state);

            if (!start_dvl_driver_on_path(cfg, shared_state, dvl_driver, event_logger, device->path)) {
                runtime.binder.mark_connect_failure(now_mono_ns, "driver start failed");
            } else {
                runtime.binder.mark_connect_success(now_mono_ns, *device);
                reset_cache = true;
            }
        }
    }

    report_device_transition(
        "DVL", io::TimingTraceKind::kDvlDeviceState, runtime, timing_logger, event_logger);
    return reset_cache;
}

} // namespace nav_core::app
