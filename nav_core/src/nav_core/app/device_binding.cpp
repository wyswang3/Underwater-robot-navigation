#include "nav_core/app/device_binding.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <set>

namespace nav_core::app {

namespace fs = std::filesystem;

namespace {

std::string trim_copy(std::string value)
{
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front()))) {
        value.erase(value.begin());
    }
    while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back()))) {
        value.pop_back();
    }
    return value;
}

std::string lowercase_copy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

std::string read_text_file(const fs::path& path)
{
    std::ifstream in(path);
    if (!in.is_open()) {
        return {};
    }

    std::string value;
    std::getline(in, value);
    return trim_copy(value);
}

SerialPortIdentity make_identity_from_path(const fs::path& path)
{
    SerialPortIdentity out{};
    out.path = path.string();

    std::error_code ec;
    const fs::path canonical = fs::weakly_canonical(path, ec);
    out.canonical_path = canonical.empty() ? out.path : canonical.string();

    fs::path tty_name = canonical.filename();
    if (tty_name.empty()) {
        tty_name = path.filename();
    }
    if (tty_name.empty()) {
        return out;
    }

    fs::path sysfs = fs::path("/sys/class/tty") / tty_name / "device";
    if (!fs::exists(sysfs, ec)) {
        return out;
    }

    fs::path cur = fs::weakly_canonical(sysfs, ec);
    if (cur.empty()) {
        cur = sysfs;
    }

    while (!cur.empty()) {
        const std::string vid = read_text_file(cur / "idVendor");
        const std::string pid = read_text_file(cur / "idProduct");
        const std::string serial = read_text_file(cur / "serial");

        if (!vid.empty() || !pid.empty() || !serial.empty()) {
            out.vendor_id = lowercase_copy(vid);
            out.product_id = lowercase_copy(pid);
            out.serial = serial;
            break;
        }

        const fs::path parent = cur.parent_path();
        if (parent == cur) {
            break;
        }
        cur = parent;
    }

    return out;
}

const SerialPortIdentity* find_by_path(const std::vector<SerialPortIdentity>& devices,
                                       const std::string&                    path)
{
    if (path.empty()) {
        return nullptr;
    }
    for (const auto& device : devices) {
        if (device.path == path || device.canonical_path == path) {
            return &device;
        }
    }
    return nullptr;
}

bool identity_filters_configured(const drivers::SerialBindingConfig& cfg) noexcept
{
    return !cfg.expected_by_id_substring.empty() ||
           !cfg.expected_vid.empty() ||
           !cfg.expected_pid.empty() ||
           !cfg.expected_serial.empty();
}

} // namespace

const char* device_state_name(DeviceConnectionState state) noexcept
{
    switch (state) {
    case DeviceConnectionState::DISCONNECTED: return "DISCONNECTED";
    case DeviceConnectionState::PROBING: return "PROBING";
    case DeviceConnectionState::CONNECTING: return "CONNECTING";
    case DeviceConnectionState::ONLINE: return "ONLINE";
    case DeviceConnectionState::MISMATCH: return "MISMATCH";
    case DeviceConnectionState::ERROR_BACKOFF: return "ERROR_BACKOFF";
    case DeviceConnectionState::RECONNECTING: return "RECONNECTING";
    default: return "UNKNOWN";
    }
}

std::vector<SerialPortIdentity> scan_serial_port_identities()
{
    std::vector<fs::path> paths;

    std::error_code ec;
    const fs::path by_id_dir("/dev/serial/by-id");
    if (fs::exists(by_id_dir, ec)) {
        for (const auto& entry : fs::directory_iterator(by_id_dir, ec)) {
            if (entry.is_symlink(ec) || entry.is_character_file(ec)) {
                paths.push_back(entry.path());
            }
        }
    }

    const fs::path dev_dir("/dev");
    if (fs::exists(dev_dir, ec)) {
        for (const auto& entry : fs::directory_iterator(dev_dir, ec)) {
            const std::string name = entry.path().filename().string();
            if (name.rfind("ttyUSB", 0) == 0 || name.rfind("ttyACM", 0) == 0) {
                paths.push_back(entry.path());
            }
        }
    }

    std::vector<SerialPortIdentity> devices;
    std::set<std::string> seen;
    for (const auto& path : paths) {
        SerialPortIdentity identity = make_identity_from_path(path);
        const std::string key =
            !identity.canonical_path.empty() ? identity.canonical_path : identity.path;
        if (!seen.insert(key).second) {
            continue;
        }
        devices.push_back(std::move(identity));
    }

    return devices;
}

bool matches_binding_identity(const SerialPortIdentity&           device,
                              const drivers::SerialBindingConfig& cfg,
                              const std::string&                  preferred_path) noexcept
{
    const bool any_filter = identity_filters_configured(cfg);

    if (!cfg.expected_by_id_substring.empty()) {
        if (device.path.find(cfg.expected_by_id_substring) == std::string::npos &&
            device.canonical_path.find(cfg.expected_by_id_substring) == std::string::npos) {
            return false;
        }
    }

    if (!cfg.expected_vid.empty() &&
        lowercase_copy(device.vendor_id) != lowercase_copy(cfg.expected_vid)) {
        return false;
    }
    if (!cfg.expected_pid.empty() &&
        lowercase_copy(device.product_id) != lowercase_copy(cfg.expected_pid)) {
        return false;
    }
    if (!cfg.expected_serial.empty() &&
        device.serial.find(cfg.expected_serial) == std::string::npos) {
        return false;
    }

    if (!any_filter && !preferred_path.empty()) {
        return device.path == preferred_path || device.canonical_path == preferred_path;
    }

    return any_filter || preferred_path.empty() || device.path == preferred_path ||
           device.canonical_path == preferred_path;
}

DeviceBinder::DeviceBinder(std::string                  label,
                           std::string                  preferred_path,
                           drivers::SerialBindingConfig cfg,
                           DiscoverFn                   discover,
                           SelectFn                     select)
    : preferred_path_(std::move(preferred_path)),
      cfg_(std::move(cfg)),
      discover_fn_(discover ? std::move(discover) : DiscoverFn(scan_serial_port_identities)),
      select_fn_(std::move(select))
{
    status_.label = std::move(label);
    status_.reason = "not probed";
}

bool DeviceBinder::should_probe(MonoTimeNs now_ns) const noexcept
{
    switch (status_.state) {
    case DeviceConnectionState::ONLINE:
    case DeviceConnectionState::PROBING:
    case DeviceConnectionState::CONNECTING:
        return false;
    case DeviceConnectionState::DISCONNECTED:
    case DeviceConnectionState::MISMATCH:
    case DeviceConnectionState::ERROR_BACKOFF:
    case DeviceConnectionState::RECONNECTING:
    default:
        return now_ns >= status_.next_action_ns;
    }
}

std::vector<SerialPortIdentity> DeviceBinder::discover_() const
{
    return discover_fn_ ? discover_fn_() : std::vector<SerialPortIdentity>{};
}

std::optional<SerialPortIdentity> DeviceBinder::probe(MonoTimeNs now_ns)
{
    transition_(now_ns, DeviceConnectionState::PROBING, "scanning candidates");

    const auto devices = discover_();
    if (select_fn_) {
        const DeviceProbeDecision decision = select_fn_(devices, cfg_, preferred_path_);
        if (decision.selected_device.has_value()) {
            const std::string reason =
                decision.reason.empty() ? "selected custom-probed device" : decision.reason;
            transition_(now_ns, DeviceConnectionState::CONNECTING, reason,
                        *decision.selected_device);
            return decision.selected_device;
        }

        const DeviceConnectionState next_state =
            (decision.state == DeviceConnectionState::CONNECTING ||
             decision.state == DeviceConnectionState::ONLINE)
                ? DeviceConnectionState::MISMATCH
                : decision.state;
        transition_(now_ns,
                    next_state,
                    decision.reason.empty() ? "custom selector rejected all candidates"
                                            : decision.reason);
        return std::nullopt;
    }

    const bool filters_configured = identity_filters_configured(cfg_);

    if (!preferred_path_.empty()) {
        if (const auto* preferred = find_by_path(devices, preferred_path_)) {
            if (matches_binding_identity(*preferred, cfg_, preferred_path_)) {
                transition_(now_ns, DeviceConnectionState::CONNECTING,
                            "selected preferred path", *preferred);
                return *preferred;
            }
            transition_(now_ns, DeviceConnectionState::MISMATCH,
                        "preferred path exists but identity mismatched", *preferred);
            return std::nullopt;
        }
    }

    for (const auto& path : cfg_.candidate_paths) {
        if (const auto* candidate = find_by_path(devices, path)) {
            if (matches_binding_identity(*candidate, cfg_, preferred_path_)) {
                transition_(now_ns, DeviceConnectionState::CONNECTING,
                            "selected candidate path", *candidate);
                return *candidate;
            }
        }
    }

    std::vector<SerialPortIdentity> matches;
    for (const auto& device : devices) {
        if (matches_binding_identity(device, cfg_, preferred_path_)) {
            matches.push_back(device);
        }
    }

    if (!matches.empty()) {
        if (!filters_configured && matches.size() > 1) {
            transition_(now_ns,
                        DeviceConnectionState::MISMATCH,
                        "multiple candidate serial devices present without identity filter");
            return std::nullopt;
        }

        transition_(now_ns, DeviceConnectionState::CONNECTING,
                    "selected auto-discovered device", matches.front());
        return matches.front();
    }

    if (!devices.empty()) {
        transition_(now_ns, DeviceConnectionState::MISMATCH,
                    "serial candidates found but identity filter rejected all");
        return std::nullopt;
    }

    transition_(now_ns, DeviceConnectionState::DISCONNECTED, "no candidate serial device found");
    return std::nullopt;
}

void DeviceBinder::mark_connect_success(MonoTimeNs now_ns, const SerialPortIdentity& device)
{
    transition_(now_ns, DeviceConnectionState::ONLINE, "driver online", device);
}

void DeviceBinder::mark_connect_failure(MonoTimeNs now_ns, const std::string& reason)
{
    transition_(now_ns, DeviceConnectionState::ERROR_BACKOFF, reason);
}

void DeviceBinder::mark_disconnected(MonoTimeNs now_ns, const std::string& reason)
{
    transition_(now_ns, DeviceConnectionState::RECONNECTING, reason);
}

void DeviceBinder::transition_(MonoTimeNs                        now_ns,
                               DeviceConnectionState             next_state,
                               std::string                       reason,
                               std::optional<SerialPortIdentity> identity)
{
    status_.state = next_state;
    status_.reason = std::move(reason);
    status_.last_transition_ns = now_ns;

    if (identity.has_value()) {
        status_.active_identity = *identity;
        status_.active_path = identity->path;
    } else if (next_state != DeviceConnectionState::ONLINE &&
               next_state != DeviceConnectionState::CONNECTING) {
        status_.active_identity = SerialPortIdentity{};
        status_.active_path.clear();
    }

    switch (next_state) {
    case DeviceConnectionState::DISCONNECTED:
    case DeviceConnectionState::MISMATCH:
    case DeviceConnectionState::ERROR_BACKOFF:
    case DeviceConnectionState::RECONNECTING:
        status_.next_action_ns =
            now_ns + static_cast<MonoTimeNs>(cfg_.reconnect_backoff_ms) * 1000000ll;
        break;
    case DeviceConnectionState::PROBING:
    case DeviceConnectionState::CONNECTING:
    case DeviceConnectionState::ONLINE:
    default:
        status_.next_action_ns = 0;
        break;
    }
}

} // namespace nav_core::app
