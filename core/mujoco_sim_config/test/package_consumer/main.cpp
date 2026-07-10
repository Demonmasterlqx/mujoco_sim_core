#include "consumer_config.hpp"

#include <string>
#include <vector>

int main() {
  const char* argv[] = {"config-consumer", "--model", "robot.xml",
                        "--headless", "--label", "one", "two"};
  install_consumer::ConsumerConfig config;
  std::string error;
  if (!install_consumer::Parse(7, argv, &config, &error)) {
    return 1;
  }
  if (config.model != "robot.xml" || !config.headless ||
      config.labels != std::vector<std::string>{"one", "two"}) {
    return 2;
  }
  return install_consumer::Usage().find("--gui") == std::string::npos ? 3 : 0;
}
