#include "adapter_app.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <mujoco_sim/adapter_sdk/controller_client.hpp>
#include <mujoco_sim/wbr_lqr_demo/controller.hpp>

#include "app_config.hpp"
#include "terminal_keyboard_input.hpp"
#include "process_signal.hpp"
#include "wbr_lqr_generated_config.hpp"

namespace {

const char* FsmName(control::chassis_state state) {
  switch (state) {
    case control::chassis_state::relax:
      return "relax";
    case control::chassis_state::recover:
      return "recover";
    case control::chassis_state::flatten:
      return "flatten";
    case control::chassis_state::neutral:
      return "neutral";
    case control::chassis_state::normal:
      return "normal";
    case control::chassis_state::offground:
      return "offground";
    case control::chassis_state::spin:
      return "spin";
    case control::chassis_state::gostair:
      return "gostair";
    case control::chassis_state::jump:
      return "jump";
  }
  return "unknown";
}

bool CheckInputTermination(mujoco_sim::wbr_lqr_demo::TerminalKeyboardInput* keyboard,
                           std::string* handler_error) {
  if (mujoco_sim::wbr_lqr_demo::ProcessSignalGuard::StopRequested()) return true;
  if (keyboard == nullptr) return false;
  if (keyboard->ExitRequested()) return true;
  std::string keyboard_error;
  if (keyboard->TakeError(&keyboard_error)) {
    *handler_error = std::move(keyboard_error);
    return true;
  }
  return false;
}

}  // namespace

namespace mujoco_sim::wbr_lqr_demo {

int RunAdapter(int argc, char** argv, KeyboardInputFactory keyboard_factory,
               const AdapterTestHooks* test_hooks) {
  std::vector<const char*> arguments;
  arguments.reserve(static_cast<std::size_t>(argc));
  for (int i = 0; i < argc; ++i)
    arguments.push_back(argv[i]);

  mujoco_sim::wbr_lqr_demo::generated::WbrConfig generated_config;
  std::string error;
  if (!mujoco_sim::wbr_lqr_demo::generated::Parse(argc, arguments.data(), &generated_config,
                                                  &error)) {
    if (error == mujoco_sim::wbr_lqr_demo::generated::Usage()) {
      std::cout << error;
      return 0;
    }
    std::cerr << error << '\n';
    return 1;
  }

  mujoco_sim::wbr_lqr_demo::AppConfig config;
  if (!mujoco_sim::wbr_lqr_demo::BuildAppConfig(generated_config, &config, &error)) {
    std::cerr << error << '\n';
    return 1;
  }

  auto signal_guard = mujoco_sim::wbr_lqr_demo::ProcessSignalGuard::Install(&error);
  if (!signal_guard) {
    std::cerr << error << '\n';
    return 1;
  }

  std::unique_ptr<mujoco_sim::wbr_lqr_demo::TerminalKeyboardInput> keyboard;
  if (config.input.mode == mujoco_sim::wbr_lqr_demo::InputMode::kCin) {
    if (keyboard_factory == nullptr) {
      std::cerr << "terminal keyboard factory is not configured\n";
      return 1;
    }
    keyboard = keyboard_factory(config.input.cin_hold_ticks, &error);
    if (!keyboard) {
      std::cerr << error << '\n';
      return 1;
    }
    std::cout << "wbr-lqr terminal input='" << keyboard->source_name()
              << "' direction_hold_ticks=" << config.input.cin_hold_ticks << '\n'
              << "keys: W/A/S/D timed motion, Q/E/F leg length, Space enable, X stop, Esc exit\n"
              << "terminal input is active only while this foreground TTY receives keys\n";
  }

  mujoco_sim::adapter_sdk::ControllerClient client(
      mujoco_sim::wbr_lqr_demo::MakeControllerOptions(config.adapter));
  if (!client.Start(&error)) {
    std::cerr << error << '\n';
    return 1;
  }

  mujoco_sim::wbr_lqr_demo::WbrController controller(client.layout(), config.adapter.interfaces,
                                                     config.pipeline, config.input_script);
  std::string handler_error;
  std::uint64_t handled_ticks = 0;
  std::uint64_t nonzero_ticks = 0;
  std::uint64_t clamp_count = 0;
  std::uint64_t space_effects = 0;
  std::uint64_t forward_effects = 0;
  std::uint64_t e_effects = 0;
  std::uint64_t q_effects = 0;

  int run_result = 0;
  if (!client.Activate(&error)) {
    handler_error = error;
    run_result = 1;
  }
  if (run_result == 0 && keyboard && !keyboard->Start(&error)) {
    handler_error = error;
    run_result = 1;
  }
  if (run_result == 0 && keyboard && test_hooks != nullptr &&
      test_hooks->after_keyboard_start != nullptr &&
      !test_hooks->after_keyboard_start(&handler_error)) {
    run_result = 1;
  }
  while (run_result == 0 &&
         (config.adapter.max_ticks == 0 || handled_ticks < config.adapter.max_ticks)) {
    if (CheckInputTermination(keyboard.get(), &handler_error)) {
      if (!handler_error.empty()) run_result = 1;
      break;
    }
    mujoco_sim::adapter_sdk::StateLease tick;
    const auto wait_result = client.WaitForNewState(std::chrono::milliseconds(100), &tick, &error);
    if (wait_result == mujoco_sim::adapter_sdk::StateReadResult::kTimeout) {
      continue;
    }
    if (wait_result != mujoco_sim::adapter_sdk::StateReadResult::kReceived) {
      handler_error = error.empty() ? "adapter stopped while waiting for State" : error;
      run_result = 1;
      break;
    }
    if (CheckInputTermination(keyboard.get(), &handler_error)) {
      if (!handler_error.empty()) run_result = 1;
      break;
    }

    try {
      std::vector<double> commands;
      if (keyboard) {
        const control::input_snapshot_t input =
            handled_ticks == 0 ? control::input_snapshot_t{} : keyboard->Consume();
        commands = controller.ComputeCommand(tick, input);
      } else {
        commands = controller.ComputeCommand(tick);
      }
      const auto& input = controller.last_input();
      if (!client.SetCommand(commands, &error)) {
        handler_error = "failed to set command: " + error;
        run_result = 1;
        break;
      }
      ++handled_ticks;
      clamp_count += controller.last_clamp_count();
      if (std::any_of(commands.begin(), commands.end(),
                      [](double command) { return std::fabs(command) > 1e-8; })) {
        ++nonzero_ticks;
      }
      if (config.log_every_ticks > 0 && handled_ticks % config.log_every_ticks == 0 &&
          controller.last_output().has_value()) {
        std::cout << "wbr-lqr tick=" << handled_ticks
                  << " fsm=" << FsmName(controller.last_output()->fsm_state)
                  << " clamps=" << clamp_count << '\n';
      }
      if (controller.last_output().has_value()) {
        const auto& command = controller.last_output()->command;
        if (input.space && command.move) ++space_effects;
        if (input.w && !input.s && std::fabs(command.v - 2.0F) < 1e-6F) {
          ++forward_effects;
        }
        if (input.e && std::fabs(command.len - config.pipeline.chassis.lmid) < 1e-6F) {
          ++e_effects;
        }
        if (input.q && std::fabs(command.len - config.pipeline.chassis.lmin) < 1e-6F) {
          ++q_effects;
        }
      }
    } catch (const std::exception& exception) {
      handler_error = exception.what();
      run_result = 1;
      break;
    }
  }

  if (keyboard && test_hooks != nullptr && test_hooks->before_keyboard_stop != nullptr &&
      !test_hooks->before_keyboard_stop(&error)) {
    if (handler_error.empty()) handler_error = error;
    run_result = 1;
  }

  if (keyboard) {
    keyboard->RequestStop();
    keyboard->Join();
    std::string final_keyboard_error;
    if (keyboard->TakeError(&final_keyboard_error) && handler_error.empty()) {
      handler_error = std::move(final_keyboard_error);
      run_result = 1;
    }
    keyboard->Clear();
  }

  std::string unregister_error;
  if (!client.Unregister(&unregister_error)) {
    std::cerr << "adapter unregister failed: " << unregister_error << '\n';
    run_result = 1;
  }

  keyboard.reset();
  std::string signal_error;
  if (!signal_guard->Restore(&signal_error)) {
    std::cerr << signal_error << '\n';
    run_result = 1;
  }

  if (!handler_error.empty()) {
    std::cerr << handler_error << '\n';
    return 1;
  }
  const auto final_state =
      controller.last_output().has_value() ? FsmName(controller.last_output()->fsm_state) : "none";
  std::cout << "wbr-lqr summary ticks=" << handled_ticks << " nonzero_ticks=" << nonzero_ticks
            << " clamps=" << clamp_count << " space_effects=" << space_effects
            << " forward_effects=" << forward_effects << " e_effects=" << e_effects
            << " q_effects=" << q_effects << " final_fsm=" << final_state << '\n';
  return run_result;
}

}  // namespace mujoco_sim::wbr_lqr_demo
