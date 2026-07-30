#include "adapter_app.hpp"

#include "terminal_keyboard_input.hpp"

int main(int argc, char** argv) {
  return mujoco_sim::wbr_lqr_demo::RunAdapter(
      argc, argv, &mujoco_sim::wbr_lqr_demo::TerminalKeyboardInput::OpenStandardInput);
}
