// src/nav_core/app/nav_daemon.cpp
//
// 简化版 nav_daemon：
//   - 从 YAML 读取 NavDaemonConfig
//   - 初始化 IMU / DVL 驱动
//   - 启动后台线程：IMU / DVL 通过回调推送数据
//   - 主循环（nav_loop_hz）：
//       * 取最新 IMU / DVL 样本喂给 OnlineEstimator
//       * 从 OnlineEstimator 取 NavState
//       * 写入 NavState SHM（NavStatePublisher）
//       * 选用 stdout 打印少量调试信息
//
// 说明：
//   - 本版本不启用因子图平滑和 NavHealthMonitor，仅做“在线导航 + 发布”；
//   - 为了减少和未展示头文件 API 不一致的风险，暂不接入 BinLogger、ImuRtFilter，
//     后续可以在 TODO 标注的位置补上。

#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "nav_core/core/types.hpp"
#include "nav_core/app/nav_daemon_config.hpp"
#include "nav_core/drivers/imu_driver_wit.hpp"
#include "nav_core/drivers/dvl_driver.hpp"
#include "nav_core/estimator/online_estimator.hpp"
#include "nav_core/io/nav_state_publisher.hpp"

// 如果你后面想用自定义 logging，可以再加：
// #include "nav_core/core/logging.hpp"

namespace {

using nav_core::MonoTimeNs;
using nav_core::SysTimeNs;
using nav_core::ImuFrame;
using nav_core::DvlFrame;
namespace app       = nav_core::app;
namespace drivers   = nav_core::drivers;
namespace estimator = nav_core::estimator;
namespace io        = nav_core::io;
namespace smsg      = shared::msg;

// --------- 全局停止标志（信号处理用） ---------

std::atomic<bool> g_stop_requested{false};

void signal_handler(int sig)
{
    (void)sig;
    g_stop_requested.store(true, std::memory_order_relaxed);
}

// --------- 简单时间工具（避免依赖项目内 timebase 细节） ---------

inline MonoTimeNs monotonic_now_ns()
{
    using clock = std::chrono::steady_clock;
    auto now    = clock::now().time_since_epoch();
    return static_cast<MonoTimeNs>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count()
    );
}

inline SysTimeNs system_now_ns()
{
    using clock = std::chrono::system_clock;
    auto now    = clock::now().time_since_epoch();
    return static_cast<SysTimeNs>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count()
    );
}

// --------- IMU / DVL 回调共享状态 ---------

struct SharedSensorState {
    // 互斥锁保护下列字段
    std::mutex m;

    // IMU
    std::optional<ImuFrame> last_imu;
    MonoTimeNs              last_imu_mono_ns{0};

    // DVL
    std::optional<DvlFrame> last_dvl;
    MonoTimeNs              last_dvl_mono_ns{0};

    // 最近一次主循环节拍（仅用于调试打印）
    std::uint64_t loop_counter{0};
};

// --------- 简单命令行解析 ---------

struct CmdlineOptions {
    std::string config_path{"config/nav_daemon.yaml"};
};

CmdlineOptions parse_cmdline(int argc, char** argv)
{
    CmdlineOptions opt;
    for (int i = 1; i < argc; ++i) {
        const char* arg = argv[i];
        if ((std::strcmp(arg, "--config") == 0 || std::strcmp(arg, "-c") == 0)
            && i + 1 < argc)
        {
            opt.config_path = argv[++i];
        } else if (std::strcmp(arg, "--help") == 0 || std::strcmp(arg, "-h") == 0) {
            std::fprintf(stderr,
                "Usage: %s [--config nav_daemon.yaml]\n", argv[0]);
            std::exit(0);
        } else {
            std::fprintf(stderr, "[nav_daemon] WARNING: unknown option: %s\n", arg);
        }
    }
    return opt;
}

// --------- 初始化 IMU 驱动 ---------

bool init_imu(const app::NavDaemonConfig& cfg,
              SharedSensorState&          shared_state,
              drivers::ImuDriverWit&      imu_driver)
{
    if (!cfg.imu.enable) {
        std::fprintf(stderr, "[nav_daemon] IMU disabled by config\n");
        return true; // 非致命：允许没有 IMU 的模式（虽然在线导航就意义不大）
    }

    const app::ImuSectionConfig& icfg = cfg.imu;
    drivers::ImuConfig dcfg           = icfg.driver;  // 拷贝一份

    auto on_frame = [&shared_state](const ImuFrame& frame) {
        MonoTimeNs mono_ns = frame.mono_ns;
        // 如果 frame 里没有 mono_ns，你可以在驱动里赋值；这里假设已经有。
        if (mono_ns <= 0) {
            mono_ns = monotonic_now_ns();
        }

        std::lock_guard<std::mutex> lock(shared_state.m);
        shared_state.last_imu          = frame;
        shared_state.last_imu_mono_ns  = mono_ns;
    };

    // 原始寄存器回调暂时不用，用于调试可以后续接 bin_logger。
    drivers::ImuDriverWit::RawCallback on_raw;

    if (!imu_driver.init(dcfg, on_frame, on_raw)) {
        std::fprintf(stderr, "[nav_daemon] ERROR: ImuDriverWit::init failed "
                             "(port=%s, baud=%d)\n",
                     dcfg.port.c_str(), dcfg.baud);
        return false;
    }

    if (!imu_driver.start()) {
        std::fprintf(stderr, "[nav_daemon] ERROR: ImuDriverWit::start failed\n");
        return false;
    }

    std::fprintf(stderr, "[nav_daemon] IMU driver started on %s @ %d\n",
                 dcfg.port.c_str(), dcfg.baud);
    return true;
}

// --------- 初始化 DVL 驱动 ---------

bool init_dvl(const app::NavDaemonConfig& cfg,
              SharedSensorState&          shared_state,
              drivers::DvlDriver&         dvl_driver)
{
    if (!cfg.dvl.enable) {
        std::fprintf(stderr, "[nav_daemon] DVL disabled by config\n");
        return true; // 可允许 DVL 关闭（IMU-only 模式）
    }

    const app::DvlSectionConfig& dcfg_section = cfg.dvl;
    drivers::DvlConfig dcfg                   = dcfg_section.driver; // 拷贝一份

    auto on_frame = [&shared_state](const DvlFrame& frame) {
        MonoTimeNs mono_ns = frame.mono_ns;
        if (mono_ns <= 0) {
            mono_ns = monotonic_now_ns();
        }

        std::lock_guard<std::mutex> lock(shared_state.m);
        shared_state.last_dvl         = frame;
        shared_state.last_dvl_mono_ns = mono_ns;
    };

    // RawCallback 目前只用于调试 / 日志，先留空。
    drivers::DvlDriver::RawCallback on_raw;

    if (!dvl_driver.init(dcfg, on_frame, on_raw)) {
        std::fprintf(stderr, "[nav_daemon] ERROR: DvlDriver::init failed "
                             "(port=%s, baud=%d)\n",
                     dcfg.port.c_str(), dcfg.baud);
        return false;
    }

    if (!dvl_driver.start()) {
        std::fprintf(stderr, "[nav_daemon] ERROR: DvlDriver::start failed\n");
        return false;
    }

    std::fprintf(stderr, "[nav_daemon] DVL driver started on %s @ %d\n",
                 dcfg.port.c_str(), dcfg.baud);
    return true;
}

// --------- 初始化 OnlineEstimator ---------

bool init_online_estimator(const app::NavDaemonConfig& cfg,
                           estimator::OnlineEstimator& est)
{
    if (!cfg.estimator.enable_online) {
        std::fprintf(stderr, "[nav_daemon] WARNING: estimator.enable_online=false, "
                             "online estimator will NOT be used\n");
        return false;
    }

    estimator::OnlineEstimatorConfig ecfg = cfg.estimator.online;
    est.setConfig(ecfg);

    // 初始位姿：全部 0，深度 0
    nav_core::Vec3d init_pos{0.0, 0.0, 0.0};
    nav_core::Vec3d init_vel{0.0, 0.0, 0.0};
    double init_yaw = 0.0;
    double init_depth = 0.0;

    est.reset(init_pos, init_vel, init_yaw, init_depth);

    std::fprintf(stderr, "[nav_daemon] OnlineEstimator initialized\n");
    return true;
}

// --------- 初始化 NavStatePublisher ---------

bool init_nav_state_publisher(const app::NavDaemonConfig& cfg,
                              io::NavStatePublisher&      pub)
{
    const app::PublisherSectionConfig& pcfg = cfg.publisher;

    io::NavStatePublisherConfig shm_cfg = pcfg.shm;
    if (!shm_cfg.enable) {
        std::fprintf(stderr, "[nav_daemon] NavStatePublisher disabled by config\n");
        return false;
    }

    if (!pub.init(shm_cfg)) {
        std::fprintf(stderr, "[nav_daemon] ERROR: NavStatePublisher::init failed "
                             "(shm_name=%s)\n",
                     shm_cfg.shm_name.c_str());
        return false;
    }

    std::fprintf(stderr, "[nav_daemon] NavStatePublisher initialized (shm=%s)\n",
                 shm_cfg.shm_name.c_str());
    return true;
}

// --------- 主循环 ---------

int run_main_loop(const app::NavDaemonConfig& cfg,
                  SharedSensorState&          shared_state,
                  drivers::ImuDriverWit&      imu_driver,
                  drivers::DvlDriver&         dvl_driver,
                  estimator::OnlineEstimator& est,
                  io::NavStatePublisher&      nav_pub)
{
    const double loop_hz = (cfg.loop.nav_loop_hz > 0.0) ? cfg.loop.nav_loop_hz : 20.0;
    const double loop_dt_s = 1.0 / loop_hz;

    std::fprintf(stderr, "[nav_daemon] main loop started (%.2f Hz)\n", loop_hz);

    auto next_wakeup = std::chrono::steady_clock::now();

    std::uint64_t print_counter = 0;

    while (!g_stop_requested.load(std::memory_order_relaxed)) {
        next_wakeup += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(loop_dt_s));

        const MonoTimeNs now_mono_ns = monotonic_now_ns();
        // const SysTimeNs  now_wall_ns = system_now_ns(); // 如需要可用于日志

        // 1) 从共享状态取最新 IMU / DVL
        std::optional<ImuFrame> imu_frame;
        std::optional<DvlFrame> dvl_frame;
        MonoTimeNs last_imu_ns = 0;
        MonoTimeNs last_dvl_ns = 0;

        {
            std::lock_guard<std::mutex> lock(shared_state.m);
            imu_frame   = shared_state.last_imu;
            dvl_frame   = shared_state.last_dvl;
            last_imu_ns = shared_state.last_imu_mono_ns;
            last_dvl_ns = shared_state.last_dvl_mono_ns;
            ++shared_state.loop_counter;
        }

        const double max_imu_age_s = cfg.loop.max_imu_age_s;
        const double max_dvl_age_s = cfg.loop.max_dvl_age_s;

        // 2) 喂给 OnlineEstimator
        if (imu_frame.has_value()) {
            // 计算 IMU age
            double imu_age_s = 0.0;
            if (last_imu_ns > 0) {
                imu_age_s = (now_mono_ns - last_imu_ns) * 1e-9;
            }
            if (imu_age_s <= max_imu_age_s || max_imu_age_s <= 0.0) {
                est.handleImuSample(*imu_frame);
            } else {
                // IMU 数据太旧：仅打印一次级别的 warning（节流打印）
                if ((print_counter % 100) == 0) {
                    std::fprintf(stderr,
                                 "[nav_daemon] WARNING: IMU age=%.3f s > max_imu_age_s=%.3f, "
                                 "skip this cycle\n",
                                 imu_age_s, max_imu_age_s);
                }
            }
        } else {
            if ((print_counter % 200) == 0) {
                std::fprintf(stderr,
                             "[nav_daemon] WARNING: no IMU frame received yet\n");
            }
        }

        if (dvl_frame.has_value()) {
            double dvl_age_s = 0.0;
            if (last_dvl_ns > 0) {
                dvl_age_s = (now_mono_ns - last_dvl_ns) * 1e-9;
            }
            if (dvl_age_s <= max_dvl_age_s || max_dvl_age_s <= 0.0) {
                est.handleDvlSample(*dvl_frame);
            } else {
                if ((print_counter % 200) == 0) {
                    std::fprintf(stderr,
                                 "[nav_daemon] WARNING: DVL age=%.3f s > max_dvl_age_s=%.3f, "
                                 "treat as DVL offline\n",
                                 dvl_age_s, max_dvl_age_s);
                }
            }
        }

        // 3) 从 OnlineEstimator 取 NavState
        smsg::NavState nav_state{};
        bool has_nav = est.lastNavState(nav_state);
        if (has_nav) {
            // 可选：在这里根据 IMU/DVL age 做最简单的 health 覆盖
            double imu_age_s = 0.0;
            if (last_imu_ns > 0) {
                imu_age_s = (now_mono_ns - last_imu_ns) * 1e-9;
            }
            double dvl_age_s = 0.0;
            if (last_dvl_ns > 0) {
                dvl_age_s = (now_mono_ns - last_dvl_ns) * 1e-9;
            }

            // 简单规则（v1）：IMU 断掉 => INVALID；DVL 断掉只影响 flags。
            if (imu_age_s > max_imu_age_s && max_imu_age_s > 0.0) {
                nav_state.health       = smsg::NavHealth::INVALID;
                nav_state.status_flags = smsg::NAV_FLAG_NONE;
            } else {
                // 由 OnlineEstimator 已经设置好的 health/status_flags 为主，
                // 这里只在 DVL 断掉时清掉 DVL_OK 标志（如你需要）。
                if (dvl_age_s > max_dvl_age_s && max_dvl_age_s > 0.0) {
                    if (smsg::nav_flag_has(nav_state.status_flags,
                                           smsg::NavStatusFlags::NAV_FLAG_DVL_OK))
                    {
                        smsg::nav_flag_clear(nav_state.status_flags,
                                             smsg::NavStatusFlags::NAV_FLAG_DVL_OK);
                    }
                    // health 在 OK / DEGRADED 之间你可以按需调整，这里保持原样。
                }
            }

            // 4) 写入 SHM
            if (nav_pub.ok()) {
                if (!nav_pub.publish(nav_state)) {
                    if ((print_counter % 100) == 0) {
                        std::fprintf(stderr,
                                     "[nav_daemon] ERROR: NavStatePublisher::publish failed\n");
                    }
                }
            }

            // 5) 简单调试输出（例如每 50 帧打印一次 xy/yaw）
            if ((print_counter % 50) == 0) {
                std::fprintf(stderr,
                             "[nav_daemon] NAV t_ns=%llu "
                             "pos(E,N)=(%.3f, %.3f) depth=%.2f yaw=%.3f rad "
                             "health=%d flags=0x%04X\n",
                             static_cast<unsigned long long>(nav_state.t_ns),
                             nav_state.pos[0], nav_state.pos[1],
                             nav_state.depth,
                             nav_state.rpy[2],
                             static_cast<int>(nav_state.health),
                             static_cast<unsigned>(nav_state.status_flags));
            }
        } else {
            if ((print_counter % 200) == 0) {
                std::fprintf(stderr,
                             "[nav_daemon] INFO: OnlineEstimator has no NavState yet\n");
            }
        }

        ++print_counter;

        // 6) 睡眠到下一个周期
        std::this_thread::sleep_until(next_wakeup);
    }

    std::fprintf(stderr, "[nav_daemon] main loop exiting (stop requested)\n");

    // 在退出前不主动对 DVL 发送 CZ 命令：
    //   - 当前策略是由外部工具显式管理 CS/CZ，避免“上电即操作”空气 ping；
    //   - 如需在 nav_daemon 内做安全停机，可以在此调用 dvl_driver.sendCommandCZ()。

    imu_driver.stop();
    dvl_driver.stop();

    return 0;
}

} // anonymous namespace

// --------- 程序入口 ---------

int main(int argc, char** argv)
{
    // 注册信号处理（Ctrl+C / kill）
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto opt = parse_cmdline(argc, argv);

    std::fprintf(stderr,
                 "[nav_daemon] starting with config: %s\n",
                 opt.config_path.c_str());

    // 1) 读取 YAML 配置
    app::NavDaemonConfig cfg{};
    std::string err;
    if (!app::load_nav_daemon_config_from_yaml(opt.config_path, cfg, &err)) {
        std::fprintf(stderr,
                     "[nav_daemon] FATAL: failed to load config from %s: %s\n",
                     opt.config_path.c_str(),
                     err.c_str());
        return 1;
    }

    // 2) 初始化共享状态 + 驱动 + 估计器 + 发布器
    SharedSensorState shared_state;

    drivers::ImuDriverWit imu_driver;
    drivers::DvlDriver    dvl_driver;
    estimator::OnlineEstimator estimator;
    io::NavStatePublisher  nav_pub;

    if (!cfg.estimator.enable_online) {
        std::fprintf(stderr,
                     "[nav_daemon] FATAL: estimator.enable_online=false, "
                     "nothing to do\n");
        return 1;
    }

    if (!init_imu(cfg, shared_state, imu_driver)) {
        std::fprintf(stderr, "[nav_daemon] FATAL: IMU init failed\n");
        return 1;
    }

    if (!init_dvl(cfg, shared_state, dvl_driver)) {
        std::fprintf(stderr, "[nav_daemon] FATAL: DVL init failed\n");
        imu_driver.stop();
        return 1;
    }

    if (!init_online_estimator(cfg, estimator)) {
        std::fprintf(stderr, "[nav_daemon] FATAL: OnlineEstimator init failed\n");
        imu_driver.stop();
        dvl_driver.stop();
        return 1;
    }

    bool pub_ok = init_nav_state_publisher(cfg, nav_pub);
    if (!pub_ok) {
        std::fprintf(stderr,
                     "[nav_daemon] WARNING: NavStatePublisher init failed or disabled, "
                     "SHM publishing will not work\n");
        // 不是致命错误：允许只在 stdout 上看结果
    }

    // 3) 进入主循环
    int rc = run_main_loop(cfg, shared_state,
                           imu_driver, dvl_driver,
                           estimator, nav_pub);

    std::fprintf(stderr, "[nav_daemon] exited with code %d\n", rc);
    return rc;
}
