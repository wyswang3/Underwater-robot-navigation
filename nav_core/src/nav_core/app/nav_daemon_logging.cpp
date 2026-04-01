#include "nav_core/app/nav_daemon_logging.hpp"

#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include "nav_core/core/timebase.hpp"
#include "nav_core/drivers/serial_port_utils.hpp"
namespace nav_core::app {
namespace drivers = nav_core::drivers;
namespace {

namespace smsg = shared::msg;
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

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

std::string wall_time_stamp_compact()
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
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

std::string date_stamp()
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
    oss << std::put_time(&tm, "%Y-%m-%d");
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

std::filesystem::path resolve_sensor_log_dir(const NavDaemonConfig& cfg,
                                             const char* sensor_label)
{
    std::filesystem::path base(cfg.logging.sensor_data_root.empty()
                                   ? cfg.logging.base_dir
                                   : cfg.logging.sensor_data_root);
    if (cfg.logging.split_by_date) {
        base /= date_stamp();
    }
    base /= sensor_label;
    return base;
}

double wall_time_unix_seconds()
{
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration<double>(now).count();
}

void write_optional(std::ostream& os, const std::optional<double>& value)
{
    if (value.has_value()) {
        os << *value;
    }
}

std::optional<double> to_optional(double value, bool enabled = true)
{
    if (!enabled || !std::isfinite(value)) {
        return std::nullopt;
    }
    return value;
}

std::string trim_copy(std::string_view text)
{
    std::size_t start = 0;
    std::size_t end = text.size();
    while (start < end && std::isspace(static_cast<unsigned char>(text[start])) != 0) {
        ++start;
    }
    while (end > start && std::isspace(static_cast<unsigned char>(text[end - 1])) != 0) {
        --end;
    }
    return std::string(text.substr(start, end - start));
}

bool parse_channel_line(const std::string& line, int& index, std::string& raw_value)
{
    const std::string trimmed = trim_copy(line);
    if (trimmed.size() < 4u) {
        return false;
    }
    if ((trimmed[0] != 'C' && trimmed[0] != 'c') || (trimmed[1] != 'H' && trimmed[1] != 'h')) {
        return false;
    }
    std::size_t pos = 2u;
    while (pos < trimmed.size() && std::isdigit(static_cast<unsigned char>(trimmed[pos])) != 0) {
        ++pos;
    }
    if (pos == 2u || pos >= trimmed.size() || trimmed[pos] != ':') {
        return false;
    }
    const std::string idx_text = trimmed.substr(2u, pos - 2u);
    try {
        index = std::stoi(idx_text);
    } catch (...) {
        return false;
    }
    raw_value = trim_copy(trimmed.substr(pos + 1u));
    return true;
}

class ChannelFrameBuffer {
public:
    explicit ChannelFrameBuffer(int channels)
        : channels_(channels > 0 ? channels : 0),
          values_(static_cast<std::size_t>(channels_), std::nullopt)
    {}

    std::optional<std::vector<std::string>> update(int index, const std::string& raw_value)
    {
        if (index < 0 || index >= channels_) {
            return std::nullopt;
        }
        values_[static_cast<std::size_t>(index)] = raw_value;
        for (const auto& item : values_) {
            if (!item.has_value()) {
                return std::nullopt;
            }
        }
        std::vector<std::string> out;
        out.reserve(values_.size());
        for (auto& item : values_) {
            out.push_back(item.value_or(std::string{}));
            item.reset();
        }
        return out;
    }

private:
    int channels_{0};
    std::vector<std::optional<std::string>> values_;
};

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

class NavSensorCsvLogger::Volt32CsvLogger {
public:
    bool start(const NavDaemonConfig& cfg)
    {
        if (!cfg.volt.enable || !cfg.logging.enable || !cfg.logging.log_sensor_csv ||
            !cfg.logging.log_volt_raw) {
            return false;
        }

        channels_ = cfg.volt.channels > 0 ? cfg.volt.channels : 16;
        std::error_code ec;
        auto dir = resolve_sensor_log_dir(cfg, "volt");
        if (!std::filesystem::exists(dir, ec) && !std::filesystem::create_directories(dir, ec)) {
            return false;
        }

        const auto file_stamp = wall_time_stamp_compact();
        const auto path = dir / ("motor_data_" + file_stamp + ".csv");
        ofs_.open(path, std::ios::out | std::ios::app);
        if (!ofs_.is_open()) {
            return false;
        }
        if (ofs_.tellp() == 0) {
            ofs_ << "MonoNS,EstNS,MonoS,EstS";
            for (int i = 0; i < channels_; ++i) {
                ofs_ << ",CH" << i;
            }
            ofs_ << "\n";
            ofs_.flush();
        }

        fd_ = drivers::open_serial_port_raw(cfg.volt.port, cfg.volt.baud, 5, &last_error_);
        if (fd_ < 0) {
            ofs_.close();
            return false;
        }

        running_.store(true);
        th_ = std::thread(&Volt32CsvLogger::thread_loop, this);
        return true;
    }

    void stop()
    {
        running_.store(false);
        if (th_.joinable()) {
            th_.join();
        }
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
        if (ofs_.is_open()) {
            ofs_.flush();
            ofs_.close();
        }
    }

private:
    void thread_loop()
    {
        ChannelFrameBuffer frame_buf(channels_);
        std::string buffer;
        buffer.reserve(512);

        while (running_.load()) {
            char tmp[256];
            const ssize_t n = ::read(fd_, tmp, sizeof(tmp));
            if (n <= 0) {
                continue;
            }
            buffer.append(tmp, tmp + n);
            for (;;) {
                std::size_t pos = buffer.find_first_of("\r\n");
                if (pos == std::string::npos) {
                    break;
                }
                std::string line = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);
                handle_line(line, frame_buf);
            }
        }
    }

    void handle_line(const std::string& line, ChannelFrameBuffer& frame_buf)
    {
        int index = -1;
        std::string raw_value;
        if (!parse_channel_line(line, index, raw_value)) {
            return;
        }
        const auto row = frame_buf.update(index, raw_value);
        if (!row.has_value()) {
            return;
        }

        const auto ts = nav_core::core::timebase::stamp("volt0",
                                                        nav_core::core::timebase::SensorKind::OTHER);
        const auto mono_ns = ts.host_time_ns;
        const auto est_ns = ts.corrected_time_ns;
        ofs_ << mono_ns << "," << est_ns << ","
             << static_cast<double>(mono_ns) / 1e9 << ","
             << static_cast<double>(est_ns) / 1e9;
        for (const auto& val : *row) {
            ofs_ << "," << csv_escape(val);
        }
        ofs_ << "\n";
        if ((++flush_counter_ % 50) == 0) {
            ofs_.flush();
        }
    }

    int fd_{-1};
    int channels_{16};
    std::string last_error_{};
    std::ofstream ofs_{};
    std::thread th_{};
    std::atomic<bool> running_{false};
    std::uint64_t flush_counter_{0};
};

void NavSensorCsvLogger::Volt32CsvLoggerDeleter::operator()(Volt32CsvLogger* ptr) const
{
    delete ptr;
}

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

void NavEventCsvLogger::log_health_audit_changed(MonoTimeNs mono_ns,
                                                 const shared::msg::NavState& nav,
                                                 const char* root_cause,
                                                 const std::string& summary)
{
    const NavPublishSnapshot snap{nav.valid != 0,
                                  nav.stale != 0,
                                  nav.degraded != 0,
                                  static_cast<std::uint16_t>(nav.fault_code),
                                  nav.status_flags};

    const char* level = "info";
    if (nav.health == smsg::NavHealth::INVALID) {
        level = "error";
    } else if (nav.health == smsg::NavHealth::DEGRADED) {
        level = "warn";
    }

    std::ostringstream msg;
    msg << "nav health audit changed root_cause="
        << (root_cause != nullptr ? root_cause : "unknown")
        << " summary=" << summary;

    log_row(mono_ns,
            "nav_health_audit_changed",
            level,
            snap.fault_code,
            snap,
            {},
            {},
            {},
            {},
            {},
            root_cause != nullptr ? root_cause : "",
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

bool NavSensorCsvLogger::init(const NavDaemonConfig& cfg)
{
    if (!cfg.logging.enable || !cfg.logging.log_sensor_csv) {
        return false;
    }

    const auto file_stamp = wall_time_stamp_compact();
    std::error_code ec;

    if (cfg.imu.enable && cfg.logging.log_imu_raw) {
        auto imu_dir = resolve_sensor_log_dir(cfg, "imu");
        if (!std::filesystem::exists(imu_dir, ec) &&
            !std::filesystem::create_directories(imu_dir, ec)) {
            return false;
        }
        imu_raw_.open(imu_dir / ("imu_raw_log_" + file_stamp + ".csv"),
                      std::ios::out | std::ios::app);
        imu_min_.open(imu_dir / ("min_imu_tb_" + file_stamp + ".csv"),
                      std::ios::out | std::ios::app);
        if (!imu_raw_.is_open() || !imu_min_.is_open()) {
            return false;
        }
        if (imu_raw_.tellp() == 0) {
            imu_raw_
                << "MonoNS,EstNS,MonoS,EstS,t_unix,"
                << "AccX,AccY,AccZ,"
                << "AsX,AsY,AsZ,"
                << "HX,HY,HZ,"
                << "AngX,AngY,AngZ,"
                << "TemperatureC\n";
        }
        if (imu_min_.tellp() == 0) {
            imu_min_
                << "MonoNS,EstNS,MonoS,EstS,"
                << "AccX,AccY,AccZ,"
                << "GyroX,GyroY,GyroZ,"
                << "YawDeg,"
                << "AngX,AngY,AngZ\n";
        }
        imu_raw_.flush();
        imu_min_.flush();
        imu_raw_g_to_mps2_ = cfg.imu.driver.raw_g_to_mps2;
        imu_enabled_ = true;
    }

    if (cfg.dvl.enable && cfg.logging.log_dvl_raw) {
        auto dvl_dir = resolve_sensor_log_dir(cfg, "dvl");
        if (!std::filesystem::exists(dvl_dir, ec) &&
            !std::filesystem::create_directories(dvl_dir, ec)) {
            return false;
        }
        dvl_raw_.open(dvl_dir / ("dvl_raw_lines_" + file_stamp + ".csv"),
                      std::ios::out | std::ios::app);
        dvl_parsed_.open(dvl_dir / ("dvl_parsed_" + file_stamp + ".csv"),
                         std::ios::out | std::ios::app);
        if (!dvl_raw_.is_open() || !dvl_parsed_.is_open()) {
            return false;
        }
        if (dvl_raw_.tellp() == 0) {
            dvl_raw_ << "Timestamp(s),SensorID,RawLine,CmdOrNote\n";
        }
        if (dvl_parsed_.tellp() == 0) {
            dvl_parsed_
                << "Timestamp(s),SensorID,Src,"
                << "Vx_body(mm_s),Vy_body(mm_s),Vz_body(mm_s),"
                << "Vx_body(m_s),Vy_body(m_s),Vz_body(m_s),"
                << "Ve_enu(mm_s),Vn_enu(mm_s),Vu_enu(mm_s),"
                << "Ve_enu(m_s),Vn_enu(m_s),Vu_enu(m_s),"
                << "De_enu(m),Dn_enu(m),Du_enu(m),"
                << "Depth(m),E(m),N(m),U(m),"
                << "Valid,ValidFlag,IsWaterMass\n";
        }
        dvl_raw_.flush();
        dvl_parsed_.flush();
        dvl_enabled_ = true;
    }

    if (cfg.volt.enable && cfg.logging.log_volt_raw) {
        volt_logger_.reset(new Volt32CsvLogger());
        if (!volt_logger_->start(cfg)) {
            volt_logger_.reset();
        }
    }

    return imu_enabled_ || dvl_enabled_ || (volt_logger_ != nullptr);
}

NavSensorCsvLogger::~NavSensorCsvLogger()
{
    shutdown();
}

void NavSensorCsvLogger::shutdown()
{
    if (volt_logger_) {
        volt_logger_->stop();
        volt_logger_.reset();
    }

    if (imu_raw_.is_open()) {
        imu_raw_.flush();
        imu_raw_.close();
    }
    if (imu_min_.is_open()) {
        imu_min_.flush();
        imu_min_.close();
    }

    if (dvl_raw_.is_open()) {
        dvl_raw_.flush();
        dvl_raw_.close();
    }
    if (dvl_parsed_.is_open()) {
        dvl_parsed_.flush();
        dvl_parsed_.close();
    }
}

void NavSensorCsvLogger::log_imu_frame(const ImuFrame& frame)
{
    if (!imu_enabled_) {
        return;
    }
    std::lock_guard<std::mutex> lock(imu_mu_);
    const auto mono_ns = frame.recv_mono_ns > 0 ? frame.recv_mono_ns : frame.mono_ns;
    const auto est_ns = frame.sensor_time_ns > 0 ? frame.sensor_time_ns : mono_ns;
    const double mono_s = static_cast<double>(mono_ns) / 1e9;
    const double est_s = static_cast<double>(est_ns) / 1e9;
    const double t_unix = wall_time_unix_seconds();
    const double g_scale = (imu_raw_g_to_mps2_ > 0.0) ? imu_raw_g_to_mps2_ : 9.78;

    const double acc_x = frame.lin_acc[0] / g_scale;
    const double acc_y = frame.lin_acc[1] / g_scale;
    const double acc_z = frame.lin_acc[2] / g_scale;
    const double gyr_x = frame.ang_vel[0] * kRadToDeg;
    const double gyr_y = frame.ang_vel[1] * kRadToDeg;
    const double gyr_z = frame.ang_vel[2] * kRadToDeg;
    const double ang_x = frame.euler[0] * kRadToDeg;
    const double ang_y = frame.euler[1] * kRadToDeg;
    const double ang_z = frame.euler[2] * kRadToDeg;

    imu_raw_
        << mono_ns << "," << est_ns << "," << mono_s << "," << est_s << ","
        << t_unix << ","
        << acc_x << "," << acc_y << "," << acc_z << ","
        << gyr_x << "," << gyr_y << "," << gyr_z << ","
        << "," << "," << ","
        << ang_x << "," << ang_y << "," << ang_z << ","
        << frame.temperature << "\n";

    imu_min_
        << mono_ns << "," << est_ns << "," << mono_s << "," << est_s << ","
        << acc_x << "," << acc_y << "," << acc_z << ","
        << gyr_x << "," << gyr_y << "," << gyr_z << ","
        << ang_z << ","
        << ang_x << "," << ang_y << "," << ang_z << "\n";

    if ((++imu_flush_counter_ % 50) == 0) {
        imu_raw_.flush();
        imu_min_.flush();
    }
}

void NavSensorCsvLogger::log_dvl_raw(const drivers::DvlRawData& raw)
{
    if (!dvl_enabled_) {
        return;
    }
    std::lock_guard<std::mutex> lock(dvl_mu_);
    const auto ts_ns = raw.sensor_time_ns > 0 ? raw.sensor_time_ns : raw.recv_mono_ns;
    const double ts_s = static_cast<double>(ts_ns) / 1e9;
    const auto& parsed = raw.parsed;
    const bool is_water = parsed.is_water_track();
    const bool is_earth = parsed.coord_frame == drivers::DvlCoordFrame::Earth;
    const bool is_body = parsed.coord_frame == drivers::DvlCoordFrame::Instrument ||
                         parsed.coord_frame == drivers::DvlCoordFrame::Ship;

    dvl_raw_
        << ts_s << ",DVL_H1000,"
        << csv_escape(raw.raw_line) << ",\n";

    const auto vx_body_mm = to_optional(parsed.ve_mm, is_body);
    const auto vy_body_mm = to_optional(parsed.vn_mm, is_body);
    const auto vz_body_mm = to_optional(parsed.vu_mm, is_body);
    const auto ve_enu_mm = to_optional(parsed.ve_mm, is_earth);
    const auto vn_enu_mm = to_optional(parsed.vn_mm, is_earth);
    const auto vu_enu_mm = to_optional(parsed.vu_mm, is_earth);

    dvl_parsed_ << ts_s << ",DVL_H1000," << parsed.src << ",";
    write_optional(dvl_parsed_, vx_body_mm); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, vy_body_mm); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, vz_body_mm); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, vx_body_mm ? std::optional<double>(*vx_body_mm * 1e-3) : std::nullopt); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, vy_body_mm ? std::optional<double>(*vy_body_mm * 1e-3) : std::nullopt); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, vz_body_mm ? std::optional<double>(*vz_body_mm * 1e-3) : std::nullopt); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, ve_enu_mm); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, vn_enu_mm); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, vu_enu_mm); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, ve_enu_mm ? std::optional<double>(*ve_enu_mm * 1e-3) : std::nullopt); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, vn_enu_mm ? std::optional<double>(*vn_enu_mm * 1e-3) : std::nullopt); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, vu_enu_mm ? std::optional<double>(*vu_enu_mm * 1e-3) : std::nullopt); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, to_optional(parsed.e_m, parsed.has_e)); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, to_optional(parsed.n_m, parsed.has_n)); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, to_optional(parsed.u_m, parsed.has_u)); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, to_optional(parsed.depth_m, parsed.has_depth)); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, std::nullopt); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, std::nullopt); dvl_parsed_ << ",";
    write_optional(dvl_parsed_, std::nullopt); dvl_parsed_ << ",";
    if (parsed.valid >= 0) {
        dvl_parsed_ << (parsed.valid == 1);
    }
    dvl_parsed_ << ",";
    if (parsed.valid == 1) {
        dvl_parsed_ << "A";
    } else if (parsed.valid == 0) {
        dvl_parsed_ << "V";
    }
    dvl_parsed_ << "," << (is_water ? "1" : "0") << "\n";

    if ((++dvl_flush_counter_ % 50) == 0) {
        dvl_raw_.flush();
        dvl_parsed_.flush();
    }
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
