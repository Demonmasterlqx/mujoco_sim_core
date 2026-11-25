#pragma once

#include <cstdint>
#include <string>

#include "wbr_lqr_generated_config.hpp"

#include <mujoco_sim/wbr_lqr_demo/config.hpp>
#include <mujoco_sim/wbr_lqr_demo/input_script.hpp>

namespace mujoco_sim::wbr_lqr_demo {

/**
 * @brief Selects the operator-input source used by the adapter process.
 *
 * Script mode preserves deterministic Controller-tick input. Cin mode reads
 * immediate actions from the process standard-input terminal and holds each
 * direction for the configured number of Controller updates.
 *
 * 选择 adapter 进程使用的操作输入源。
 *
 * Script 模式保留基于 Controller tick 的确定性输入；Cin 模式从进程标准输入终端读取
 * 即时动作，并让每个方向保持配置的 Controller update 数量。
 */
enum class InputMode {
  kScript,
  kCin,
};

/**
 * @brief Describes the selected adapter input source.
 *        描述 adapter 选用的输入源。
 */
struct InputConfig {
  InputMode mode = InputMode::kScript;
  /**
   * @brief Number of Controller updates for which each cin direction remains active.
   *        每次 cin 方向输入保持生效的 Controller update 数量。
   */
  std::uint32_t cin_hold_ticks = 200;
};

/**
 * @brief Stable application configuration produced from generated values.
 *        由生成配置转换得到的稳定应用配置。
 */
struct AppConfig {
  AdapterConfig adapter;
  PipelineConfig pipeline;
  InputConfig input;
  InputScript input_script;
  std::uint64_t log_every_ticks = 0;
};

/**
 * @brief Converts and cross-validates generated WBR adapter configuration.
 *
 * Conversion is transactional: `config` is unchanged on failure. Script mode
 * continues to validate the configured deterministic input intervals.
 *
 * 转换并交叉校验生成的 WBR adapter 配置。
 *
 * 转换具备事务性：失败时 `config` 保持不变。Script 模式仍会校验配置的确定性输入区间。
 *
 * @param generated_config Parsed generated configuration value.
 *                         已解析的生成配置值。
 * @param config Receives the converted configuration and must not be null.
 *               接收转换结果，且不得为空。
 * @param error_message Receives a contextual diagnostic and must not be null.
 *                      接收上下文诊断，且不得为空。
 * @return true on success; otherwise false.
 *         成功返回 true，否则返回 false。
 */
bool BuildAppConfig(const generated::WbrConfig& generated_config, AppConfig* config,
                    std::string* error_message);

}  // namespace mujoco_sim::wbr_lqr_demo
