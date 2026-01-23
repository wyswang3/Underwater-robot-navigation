#pragma once

// nav_core/include/nav_core/estimator/eskf_config_yaml.hpp
//
// @file  eskf_config_yaml.hpp
// @brief 从 YAML 文件加载 EskfConfig（与 Python 版 ESKF 2D 配置结构对齐）。
//
// 设计目标：
//   - 直接复用离线 ESKF 的 YAML 配置结构（time/init/imu/...），避免为 C++ 再造一套；
//   - 支持：
//       * 从文件路径加载：load_eskf_config_from_yaml_file(path, cfg, err);
//   - 未在 YAML 中出现的 3D ESKF 扩展字段（U、高度相关等）保持 EskfConfig 默认值。

#include <string>

#include "nav_core/estimator/eskf.hpp"

namespace nav_core::estimator {

/// @brief 从 YAML 文件加载 EskfConfig。
///
/// @param yaml_path  YAML 文件路径（结构参见 Python ESKF 2D 配置）
/// @param cfg        输出：填充好的 EskfConfig（在加载前不会被清零）
/// @param err_msg    若非空，出错时写入简要错误信息
///
/// @return true  加载成功
/// @return false 打开/解析/字段转换失败
bool load_eskf_config_from_yaml_file(const std::string& yaml_path,
                                     EskfConfig&        cfg,
                                     std::string*       err_msg = nullptr);

} // namespace nav_core::estimator
