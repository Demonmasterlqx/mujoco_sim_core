#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <mujoco_sim/adapter_sdk/sync_adapter_client.hpp>
#include <mujoco_sim/wbr_lqr_demo/tick_controller.hpp>

#include "app_config.hpp"
#include "wbr_lqr_generated_config.hpp"

namespace {

const char* FsmName(control::chassis_state state) {
  switch (state) {
    case control::chassis_state::relax: return "relax";
    case control::chassis_state::recover: return "recover";
    case control::chassis_state::flatten: return "flatten";
    case control::chassis_state::neutral: return "neutral";
    case control::chassis_state::normal: return "normal";
    case control::chassis_state::offground: return "offground";
    case control::chassis_state::spin: return "spin";
    case control::chassis_state::gostair: return "gostair";
    case control::chassis_state::jump: return "jump";
  }
  return "unknown";
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<const char*> arguments;
  arguments.reserve(static_cast<std::size_t>(argc));
  for (int i = 0; i < argc; ++i) arguments.push_back(argv[i]);

  mujoco_sim::wbr_lqr_demo::generated::WbrConfig generated_config;
  std::string error;
  if (!mujoco_sim::wbr_lqr_demo::generated::Parse(
          argc, arguments.data(), &generated_config, &error)) {
    if (error == mujoco_sim::wbr_lqr_demo::generated::Usage()) {
      std::cout << error;
      return 0;
    }
    std::cerr << error << '\n';
    return 1;
  }

  mujoco_sim::wbr_lqr_demo::AppConfig config;
  if (!mujoco_sim::wbr_lqr_demo::BuildAppConfig(generated_config, &config,
                                                &error)) {
    std::cerr << error << '\n';
    return 1;
  }

  mujoco_sim::adapter_sdk::SyncAdapterClient client(
      mujoco_sim::wbr_lqr_demo::MakeAdapterOptions(config.adapter));
  if (!client.Start(&error)) {
    std::cerr << error << '\n';
    return 1;
  }

  mujoco_sim::wbr_lqr_demo::TickController controller(
      client.registration().layout(), config.adapter.interfaces,
      config.pipeline, config.input_script);
  std::string handler_error;
  std::uint64_t handled_ticks = 0;
  std::uint64_t nonzero_ticks = 0;
  std::uint64_t clamp_count = 0;
  std::uint64_t space_effects = 0;
  std::uint64_t e_effects = 0;
  std::uint64_t q_effects = 0;

  int run_result = 0;
  if (!client.Activate(&error)) {
    handler_error = error;
    run_result = 1;
  }
  while (run_result == 0 &&
         (config.adapter.max_ticks == 0 ||
          handled_ticks < config.adapter.max_ticks)) {
    mujoco_sim::v1::Tick tick;
    const auto wait_result = client.WaitForTick(
        std::chrono::milliseconds(100), &tick, &error);
    if (wait_result == mujoco_sim::adapter_sdk::TickWaitResult::kTimeout) {
      continue;
    }
    if (wait_result != mujoco_sim::adapter_sdk::TickWaitResult::kReceived) {
      handler_error = error.empty() ? "adapter stopped while waiting for Tick"
                                    : error;
      run_result = 1;
      break;
    }

    try {
      auto commands = controller.HandleTick(tick);
      const auto& input = controller.last_input();
      if (!client.Commit(tick, commands, &error)) {
        handler_error = "failed to Commit Tick: " + error;
        run_result = 1;
        break;
      }
      if (tick.controller_state() == mujoco_sim::v1::ACTIVE) {
        ++handled_ticks;
      }
      clamp_count += controller.last_clamp_count();
      if (tick.controller_state() == mujoco_sim::v1::ACTIVE &&
          std::any_of(commands.begin(), commands.end(),
                      [](const auto& command) {
                        return std::fabs(command.value) > 1e-8;
                      })) {
        ++nonzero_ticks;
      }
      if (tick.controller_state() == mujoco_sim::v1::ACTIVE &&
          config.log_every_ticks > 0 &&
          handled_ticks % config.log_every_ticks == 0 &&
          controller.last_output().has_value()) {
        std::cout << "wbr-lqr tick=" << handled_ticks
                  << " fsm=" << FsmName(controller.last_output()->fsm_state)
                  << " clamps=" << clamp_count << '\n';
      }
      if (tick.controller_state() == mujoco_sim::v1::ACTIVE &&
          controller.last_output().has_value()) {
        const auto& command = controller.last_output()->command;
        if (input.space && command.move) ++space_effects;
        if (input.e &&
            std::fabs(command.len - config.pipeline.chassis.lmid) < 1e-6F) {
          ++e_effects;
        }
        if (input.q &&
            std::fabs(command.len - config.pipeline.chassis.lmin) < 1e-6F) {
          ++q_effects;
        }
      }
    } catch (const std::exception& exception) {
      handler_error = exception.what();
      run_result = 1;
      client.Stop();
      break;
    }
  }

  std::string unregister_error;
  if (!client.Unregister(&unregister_error)) {
    std::cerr << "adapter unregister failed: " << unregister_error << '\n';
  }

  if (!client.lifecycle_warning().empty()) {
    std::cerr << "adapter lifecycle warning: " << client.lifecycle_warning()
              << '\n';
  }

  if (!handler_error.empty()) {
    std::cerr << handler_error << '\n';
    return 1;
  }
  const auto final_state = controller.last_output().has_value()
                               ? FsmName(controller.last_output()->fsm_state)
                               : "none";
  std::cout << "wbr-lqr summary ticks=" << handled_ticks
            << " nonzero_ticks=" << nonzero_ticks
            << " clamps=" << clamp_count
            << " space_effects=" << space_effects
            << " e_effects=" << e_effects << " q_effects=" << q_effects
            << " final_fsm=" << final_state
            << '\n';
  return run_result;
}
