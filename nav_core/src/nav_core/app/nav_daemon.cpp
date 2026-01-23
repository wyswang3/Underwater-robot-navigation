// src/nav_core/app/nav_daemon.cpp
//
// 角色：只做 wiring，不做业务细节。
//   - 解析命令行：--config nav_daemon.yaml, --eskf-config eskf.yaml
//   - 加载 NavDaemonConfig / EskfConfig
//   - 安装信号处理（SIGINT / SIGTERM）→ 设置 stop_flag
//   - 调用 app::run_nav_daemon(...)，其内部完成所有驱动/滤波/ESKF 管线搭建与主循环。

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "nav_core/app/nav_daemon_config.hpp"
#include "nav_core/estimator/eskf_config_yaml.hpp"

namespace app       = nav_core::app;
namespace estimator = nav_core::estimator;

// ========== 命令行解析 ==========

struct CmdlineOptions {
    std::string config_path{"config/nav_daemon.yaml"};
    std::string eskf_config_path{"config/eskf.yaml"};
};

CmdlineOptions parse_cmdline(int argc, char** argv)
{
    CmdlineOptions opt;
    for (int i = 1; i < argc; ++i) {
        const char* arg = argv[i];
        if ((std::strcmp(arg, "--config") == 0 || std::strcmp(arg, "-c") == 0) && i + 1 < argc) {
            opt.config_path = argv[++i];
        } else if (std::strcmp(arg, "--eskf-config") == 0 && i + 1 < argc) {
            opt.eskf_config_path = argv[++i];
        } else if (std::strcmp(arg, "--help") == 0 || std::strcmp(arg, "-h") == 0) {
            std::fprintf(stderr,
                         "Usage: %s [--config nav_daemon.yaml] [--eskf-config eskf.yaml]\n",
                         argv[0]);
            std::exit(0);
        } else {
            std::fprintf(stderr, "[nav_daemon] WARNING: unknown option: %s\n", arg);
        }
    }
    return opt;
}

// ========== 停止标志 & 信号处理 ==========

// main 中持有的停止标志，传给 run_nav_daemon / run_main_loop 使用
static std::atomic<bool> g_stop_flag{false};
// 给 signal handler 使用的指针（避免在信号处理里用复杂对象）
static std::atomic<bool>* g_stop_flag_ptr = &g_stop_flag;

void signal_handler(int sig)
{
    (void)sig;
    if (g_stop_flag_ptr) {
        g_stop_flag_ptr->store(true, std::memory_order_relaxed);
    }
}

// ========== 声明业务入口（实现在 nav_daemon_runner.cpp 中） ==========

namespace nav_core::app {

int run_nav_daemon(const NavDaemonConfig&      cfg,
                   const estimator::EskfConfig& eskf_cfg,
                   std::atomic<bool>&          stop_flag);

} // namespace nav_core::app

// ========== main：只做 wiring，不做业务细节 ==========

int main(int argc, char** argv)
{
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    auto opt = parse_cmdline(argc, argv);

    std::fprintf(stderr,
                 "[nav_daemon] starting with config=%s eskf_config=%s\n",
                 opt.config_path.c_str(), opt.eskf_config_path.c_str());

    // 1) 读取 nav_daemon 配置
    app::NavDaemonConfig cfg{};
    std::string err;
    if (!app::load_nav_daemon_config_from_yaml(opt.config_path, cfg, &err)) {
        std::fprintf(stderr,
                     "[nav_daemon] FATAL: failed to load %s: %s\n",
                     opt.config_path.c_str(), err.c_str());
        return 1;
    }
    if (!cfg.estimator.enable_online) {
        std::fprintf(stderr,
                     "[nav_daemon] FATAL: estimator.enable_online=false, nothing to do\n");
        return 1;
    }

    // 2) 读取 ESKF 配置
    estimator::EskfConfig eskf_cfg{};
    std::string eskf_err;
    if (!estimator::load_eskf_config_from_yaml_file(opt.eskf_config_path, eskf_cfg, &eskf_err)) {
        std::fprintf(stderr,
                     "[nav_daemon] FATAL: failed to load ESKF config %s: %s\n",
                     opt.eskf_config_path.c_str(), eskf_err.c_str());
        return 1;
    }

    // 3) 把配置 + 停止标志交给统一入口函数
    int rc = app::run_nav_daemon(cfg, eskf_cfg, g_stop_flag);
    std::fprintf(stderr, "[nav_daemon] exited with code %d\n", rc);
    return rc;
}
