#include <iostream>
#include <string>

#include "status_tool.hpp"

int main(int argc, char** argv) {
  mujoco_sim::server::status_tool::StatusOptions options;
  std::string message;
  const auto result = mujoco_sim::server::status_tool::ParseCommandLine(
      argc, argv, &options, &message);
  if (result ==
      mujoco_sim::server::status_tool::CommandLineResult::kHelp) {
    std::cout << message;
    return 0;
  }
  if (result ==
      mujoco_sim::server::status_tool::CommandLineResult::kError) {
    std::cerr << message << '\n';
    return 1;
  }
  return mujoco_sim::server::status_tool::RunStatusTool(
      options, &std::cout, &std::cerr);
}
