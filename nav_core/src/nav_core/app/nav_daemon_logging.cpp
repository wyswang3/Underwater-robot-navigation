#include "nav_core/app/nav_daemon_logging.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>

#include <unistd.h>

namespace nav_core::app {
namespace {

namespace smsg = shared::msg;

std::string csv_escape(const std::string& value)
{
    std::string out;
    out.reserve(value.size() + 2);
    out.push_back('"');
    for (const char ch : value) {
        if (ch == '"') {
            out.push_back('"');
        }
        out.push_back(ch);
    }
    out.push_back('"');
    return out;
}

std::string wall_time_now_string()
{
    const auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

std::string resolve_run_id(const char* process_name)
{
    if (const char* env = std::getenv("ROV_RUN_ID"); env != nullptr && env[0] != '\0') {
        return env;
    }

    std::time_t tt = std::time(nullptr);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif
    std::ostringstream oss;
    oss << process_name << "-" << ::getpid() << "-" << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

std::filesystem::path resolve_nav_log_dir(const NavDaemonConfig& cfg)
{
    const auto& lcfg = cfg.logging;
    std::filesystem::path base(lcfg.base_dir);
    if (lcfg.split_by_date) {
        const auto now = std::chrono::system_clock::now();
        std::time_t tt = std::chrono::system_clock::to_time_t(now);
        std::tm tm{};
#if defined(_WIN32)
        localtime_s(&tm, &tt);
#else
        localtime_r(&tt, &tm);
#endif
        char buf[16];
        std::snprintf(buf,
                      sizeof(buf),
                      "%04d-%02d-%02d",
                      tm.tm_year + 1900,
                      tm.tm_mon + 1,
                      tm.tm_mday);
        base /= buf;
    }
    base /= "nav";
    return base;
}

bool is_imu_label(const std::string& label)
{
    return label == "imu" || label == "IMU";
}

smsg::NavFaultCode device_state_fault_code(const DeviceBindingStatus& status)
{
    const bool imu = is_imu_label(status.label);
    switch (status.state) {
    case DeviceConnectionState::ONLINE:
        return smsg::NavFaultCode::kNone;
    case DeviceConnectionState::MISMATCH:
        return imu ? smsg::NavFaultCode::kImuDeviceMismatch
                   : smsg::NavFaultCode::kDvlDeviceMismatch;
    case DeviceConnectionState::DISCONNECTED:
        if (!status.active_path.empty() ||
            status.reason.find("timeout") != std::string::npos ||
            status.reason.find("closed") != std::string::npos ||
            status.reason.find("driver") != std::string::npos) {
            return imu ? smsg::NavFaultCode::kImuDisconnected
                       : smsg::NavFaultCode::kDvlDisconnected;
        }
        return imu ? smsg::NavFaultCode::kImuDeviceNotFound
                   : smsg::NavFaultCode::kDvlDeviceNotFound;
    case DeviceConnectionState::PROBING:
    case DeviceConnectionState::CONNECTING:
    case DeviceConnectionState::ERROR_BACKOFF:
    case DeviceConnectionState::RECONNECTING:
    default:
        return !status.active_path.empty()
            ? (imu ? smsg::NavFaultCode::kImuDisconnected
                   : smsg::NavFaultCode::kDvlDisconnected)
            : (imu ? smsg::NavFaultCode::kImuDeviceNotFound
                   : smsg::NavFaultCode::kDvlDeviceNotFound);
    }
}

std::string compact_preview(const std::string& value, std::size_t max_len = 160u)
{
    if (value.size() <= max_len) {
        return value;
    }
    return value.substr(0, max_len) + "...";
}

} // namespace

bool NavPublishSnapshot::operator==(const NavPublishSnapshot& rhs) const noexcept
{
    return valid == rhs.valid &&
           stale == rhs.stale &&
           degraded == rhs.degraded &&
           fault_code == rhs.fault_code &&
           nav_status_flags == rhs.nav_status_flags;
}

bool NavEventCsvLogger::init(const NavDaemonConfig& cfg)
{
    if (!cfg.logging.enable) {
        return false;
    }

    std::error_code ec;
    const auto dir = resolve_nav_log_dir(cfg);
    if (!std::filesystem::exists(dir, ec) && !std::filesystem::create_directories(dir, ec)) {
        return false;
    }

    const auto path = dir / "nav_events.csv";
    const bool need_header = !std::filesystem::exists(path, ec) ||
                             std::filesystem::file_size(path, ec) == 0;
    ofs_.open(path, std::ios::out | std::ios::app);
    if (!ofs_.is_open()) {
        return false;
    }

    run_id_ = resolve_run_id("uwnav_navd");
    if (need_header) {
        ofs_
            << "mono_ns,wall_time,component,event,level,run_id,process_name,pid"
            << ",fault_code,nav_valid,nav_stale,nav_degraded,nav_status_flags,message"
            << ",device_label,device_path,state,reason,sensor_id,reason_class,sample_age_ms\n";
        ofs_.flush();
    }
    return true;
}

void NavEventCsvLogger::log_device_bind_state_changed(MonoTimeNs mono_ns,
                                                      const DeviceBindingStatus& status)
{
    const auto fault_code = device_state_fault_code(status);
    std::ostringstream msg;
    msg << "device bind state changed to " << device_state_name(status.state)
        << " path=" << (status.active_path.empty() ? "<none>" : status.active_path)
        << " reason=" << (status.reason.empty() ? "<none>" : status.reason);
    log_row(mono_ns,
            "device_bind_state_changed",
            status.state == DeviceConnectionState::ONLINE ? "info" : "warn",
            static_cast<std::uint16_t>(fault_code),
            {},
            status.label,
            status.active_path,
            device_state_name(status.state),
            status.reason,
            {},
            {},
            kAgeUnknownMs,
            msg.str());
}

void NavEventCsvLogger::log_serial_open_failed(MonoTimeNs mono_ns,
                                               const char* device_label,
                                               const std::string& device_path,
                                               int baud,
                                               const char* reason)
{
    const bool imu = (device_label != nullptr && std::string(device_label) == "imu");
    const auto fault_code = imu ? smsg::NavFaultCode::kImuDisconnected
                                : smsg::NavFaultCode::kDvlDisconnected;
    std::ostringstream msg;
    msg << "serial open failed on " << device_path << " @ " << baud
        << " reason=" << (reason != nullptr ? reason : "unknown");
    log_row(mono_ns,
            "serial_open_failed",
            "error",
            static_cast<std::uint16_t>(fault_code),
            {},
            device_label != nullptr ? device_label : "",
            device_path,
            {},
            reason != nullptr ? reason : "",
            {},
            {},
            kAgeUnknownMs,
            msg.str());
}

void NavEventCsvLogger::log_sensor_update_rejected(MonoTimeNs mono_ns,
                                                   const char* sensor_id,
                                                   const char* reason_class,
                                                   std::uint32_t sample_age_ms,
                                                   std::uint16_t fault_code)
{
    std::ostringstream msg;
    msg << (sensor_id != nullptr ? sensor_id : "sensor")
        << " update rejected: " << (reason_class != nullptr ? reason_class : "unknown")
        << " age_ms=" << sample_age_ms;
    log_row(mono_ns,
            "sensor_update_rejected",
            sample_age_ms == kAgeUnknownMs ? "info" : "warn",
            fault_code,
            {},
            {},
            {},
            {},
            {},
            sensor_id != nullptr ? sensor_id : "",
            reason_class != nullptr ? reason_class : "",
            sample_age_ms,
            msg.str());
}

void NavEventCsvLogger::log_nav_publish_state_changed(MonoTimeNs mono_ns,
                                                      const shared::msg::NavState& nav)
{
    std::ostringstream msg;
    msg << "nav publish state changed valid=" << static_cast<unsigned>(nav.valid)
        << " stale=" << static_cast<unsigned>(nav.stale)
        << " degraded=" << static_cast<unsigned>(nav.degraded)
        << " fault=" << static_cast<unsigned>(nav.fault_code)
        << " flags=0x" << std::hex << static_cast<unsigned>(nav.status_flags);
    const NavPublishSnapshot snap{nav.valid != 0,
                                  nav.stale != 0,
                                  nav.degraded != 0,
                                  static_cast<std::uint16_t>(nav.fault_code),
                                  nav.status_flags};
    log_row(mono_ns,
            "nav_publish_state_changed",
            snap.valid && !snap.stale ? "info" : "warn",
            snap.fault_code,
            snap,
            {},
            {},
            {},
            {},
            {},
            {},
            nav.age_ms,
            msg.str());
}

void NavEventCsvLogger::log_protocol_diagnostic(MonoTimeNs mono_ns,
                                                const char* device_label,
                                                const char* signature,
                                                const std::string& summary,
                                                const std::string& preview_text,
                                                const std::string& preview_hex)
{
    const bool imu = (device_label != nullptr && std::string(device_label) == "imu");
    const auto fault_code = imu ? smsg::NavFaultCode::kImuDeviceMismatch
                                : smsg::NavFaultCode::kDvlDeviceMismatch;

    std::ostringstream msg;
    msg << summary;
    if (!preview_text.empty()) {
        msg << " preview_text=" << compact_preview(preview_text);
    }
    if (!preview_hex.empty()) {
        msg << " preview_hex=" << compact_preview(preview_hex);
    }

    log_row(mono_ns,
            "serial_protocol_diagnostic",
            "warn",
            static_cast<std::uint16_t>(fault_code),
            {},
            device_label != nullptr ? device_label : "",
            {},
            {},
            summary,
            {},
            signature != nullptr ? signature : "",
            kAgeUnknownMs,
            msg.str());
}

void NavEventCsvLogger::log_row(MonoTimeNs mono_ns,
                                const char* event,
                                const char* level,
                                std::uint16_t fault_code,
                                const NavPublishSnapshot& nav_snapshot,
                                const std::string& device_label,
                                const std::string& device_path,
                                const std::string& state,
                                const std::string& reason,
                                const std::string& sensor_id,
                                const std::string& reason_class,
                                std::uint32_t sample_age_ms,
                                const std::string& message)
{
    if (!ofs_.is_open()) {
        return;
    }

    ofs_
        << mono_ns
        << "," << csv_escape(wall_time_now_string())
        << "," << csv_escape("uwnav_navd")
        << "," << csv_escape(event != nullptr ? event : "")
        << "," << csv_escape(level != nullptr ? level : "")
        << "," << csv_escape(run_id_)
        << "," << csv_escape("uwnav_navd")
        << "," << ::getpid()
        << "," << fault_code
        << "," << (nav_snapshot.valid ? 1 : 0)
        << "," << (nav_snapshot.stale ? 1 : 0)
        << "," << (nav_snapshot.degraded ? 1 : 0)
        << "," << nav_snapshot.nav_status_flags
        << "," << csv_escape(message)
        << "," << csv_escape(device_label)
        << "," << csv_escape(device_path)
        << "," << csv_escape(state)
        << "," << csv_escape(reason)
        << "," << csv_escape(sensor_id)
        << "," << csv_escape(reason_class)
        << "," << sample_age_ms
        << "\n";
    ofs_.flush();
}

bool init_nav_state_publisher(const NavDaemonConfig& cfg,
                              io::NavStatePublisher& pub)
{
    io::NavStatePublisherConfig shm_cfg = cfg.publisher.shm;
    if (!shm_cfg.enable) {
        std::fprintf(stderr, "[nav_daemon] NavStatePublisher disabled by config\n");
        return false;
    }
    if (!pub.init(shm_cfg)) {
        std::fprintf(stderr,
                     "[nav_daemon] ERROR: NavStatePublisher::init failed (shm=%s)\n",
                     shm_cfg.shm_name.c_str());
        std::fprintf(stderr,
                     "[nav_daemon] HINT: 请检查 shm_name 是否与控制侧一致、系统共享内存权限是否足够\n");
        return false;
    }
    std::fprintf(stderr,
                 "[nav_daemon] NavStatePublisher initialized (shm=%s)\n",
                 shm_cfg.shm_name.c_str());
    return true;
}

bool init_named_bin_logger(const NavDaemonConfig& cfg,
                           const char* filename,
                           nav_core::BinLogger& logger)
{
    const auto& lcfg = cfg.logging;
    if (!lcfg.enable || filename == nullptr || filename[0] == '\0') {
        std::fprintf(stderr,
                     "[nav_daemon] bin_logger disabled by config or invalid filename\n");
        return false;
    }

    const std::filesystem::path file = resolve_nav_log_dir(cfg) / filename;
    if (!logger.open(file.string(), /*append=*/true)) {
        std::fprintf(stderr,
                     "[nav_daemon] WARNING: BinLogger::open('%s') failed, disable logging\n",
                     file.string().c_str());
        return false;
    }

    std::fprintf(stderr,
                 "[nav_daemon] bin_logger logging to %s\n",
                 file.string().c_str());
    return true;
}

void write_timing_trace(nav_core::BinLogger* logger,
                        io::TimingTraceKind kind,
                        const SampleTiming& timing,
                        MonoTimeNs publish_mono_ns,
                        std::uint32_t age_ms,
                        std::uint16_t flags,
                        shared::msg::NavFaultCode fault_code)
{
    if (logger == nullptr) {
        return;
    }

    io::TimingTracePacketV1 pkt{};
    pkt.kind = static_cast<std::uint16_t>(kind);
    pkt.flags = flags;
    pkt.sensor_time_ns = timing.sensor_time_ns;
    pkt.recv_mono_ns = timing.recv_mono_ns;
    pkt.consume_mono_ns = timing.consume_mono_ns;
    pkt.publish_mono_ns = publish_mono_ns;
    pkt.age_ms = age_ms;
    pkt.fault_code = static_cast<std::uint32_t>(fault_code);
    logger->writePod(pkt);
}

} // namespace nav_core::app
