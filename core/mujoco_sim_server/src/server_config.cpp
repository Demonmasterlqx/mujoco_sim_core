#include <mujoco_sim/server/server_config.hpp>

#include <cstdint>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <CLI/CLI.hpp>
#include <yaml-cpp/yaml.h>

namespace mujoco_sim::server {
namespace {

constexpr const char* kYamlIntegerTag = "tag:yaml.org,2002:int";
constexpr const char* kYamlStringTag = "tag:yaml.org,2002:str";
constexpr const char* kYamlBooleanTag = "tag:yaml.org,2002:bool";

struct CommandLineOverrides {
  std::optional<std::string> config_path;
  std::optional<std::string> model_path;
  std::optional<std::string> server_name;
  std::optional<std::uint64_t> max_ticks;
  std::optional<std::uint32_t> tick_timeout_ms;
  std::optional<std::uint32_t> startup_timeout_ms;
  bool headless = false;
  bool gui = false;
};

std::string ConfigLocation(const std::string& path, const YAML::Mark& mark) {
  std::ostringstream output;
  output << path;
  if (!mark.is_null()) {
    output << ':' << mark.line + 1 << ':' << mark.column + 1;
  }
  return output.str();
}

bool ConfigError(const std::string& path, const YAML::Mark& mark,
                 const std::string& message, std::string* error_message) {
  *error_message = ConfigLocation(path, mark) + ": " + message;
  return false;
}

bool IsImplicitNonString(const YAML::Node& node) {
  bool boolean_value = false;
  long long signed_value = 0;
  unsigned long long unsigned_value = 0;
  long double floating_value = 0;
  return YAML::convert<bool>::decode(node, boolean_value) ||
         YAML::convert<long long>::decode(node, signed_value) ||
         YAML::convert<unsigned long long>::decode(node, unsigned_value) ||
         YAML::convert<long double>::decode(node, floating_value);
}

bool ParseStringField(const std::string& path, const std::string& key,
                      const YAML::Node& node, std::string* value,
                      std::string* error_message) {
  if (!node.IsScalar() ||
      (node.Tag() != "!" && node.Tag() != kYamlStringTag &&
       (node.Tag() != "?" || IsImplicitNonString(node)))) {
    return ConfigError(path, node.Mark(), key + " must be a string",
                       error_message);
  }
  *value = node.as<std::string>();
  return true;
}

bool ParseBooleanField(const std::string& path, const std::string& key,
                       const YAML::Node& node, bool* value,
                       std::string* error_message) {
  const std::string tag = node.Tag();
  if (!node.IsScalar() || (tag != "?" && tag != kYamlBooleanTag) ||
      !YAML::convert<bool>::decode(node, *value)) {
    return ConfigError(path, node.Mark(), key + " must be a boolean",
                       error_message);
  }
  return true;
}

template <typename T>
bool ParseUnsignedField(const std::string& path, const std::string& key,
                        const YAML::Node& node, T* value,
                        std::string* error_message) {
  const std::string tag = node.Tag();
  if (!node.IsScalar() ||
      (tag != "?" && tag != kYamlIntegerTag)) {
    return ConfigError(path, node.Mark(),
                       key + " must be an unsigned integer", error_message);
  }

  try {
    *value = node.as<T>();
    return true;
  } catch (const YAML::Exception&) {
    return ConfigError(path, node.Mark(),
                       key + " must be an unsigned integer in range",
                       error_message);
  }
}

bool ApplyYamlField(const std::string& path, const std::string& key,
                    const YAML::Node& value, ServerConfig* config,
                    std::string* error_message) {
  if (key == "server_name") {
    return ParseStringField(path, key, value, &config->server_name,
                            error_message);
  }
  if (key == "model_path") {
    return ParseStringField(path, key, value, &config->model_path,
                            error_message);
  }
  if (key == "max_ticks") {
    return ParseUnsignedField(path, key, value, &config->max_ticks,
                              error_message);
  }
  if (key == "tick_timeout_ms") {
    return ParseUnsignedField(path, key, value, &config->tick_timeout_ms,
                              error_message);
  }
  if (key == "startup_timeout_ms") {
    return ParseUnsignedField(path, key, value, &config->startup_timeout_ms,
                              error_message);
  }
  if (key == "status_period_ms") {
    return ParseUnsignedField(path, key, value, &config->status_period_ms,
                              error_message);
  }
  if (key == "headless") {
    return ParseBooleanField(path, key, value, &config->headless,
                             error_message);
  }
  return ConfigError(path, value.Mark(), "unknown field: " + key,
                     error_message);
}

void ConfigureCommandLine(CLI::App* app, CommandLineOverrides* overrides) {
  app->set_help_flag("-h,--help", "Show this help message and exit");
  app->add_option("--config", overrides->config_path,
                  "YAML configuration file")
      ->check(CLI::ExistingFile)
      ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
  app->add_option("--model", overrides->model_path,
                  "MuJoCo MJCF model path")
      ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
  app->add_option("--server-name", overrides->server_name,
                  "Server namespace")
      ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
  app->add_option("--max-ticks", overrides->max_ticks,
                  "Stop successfully after this many ticks; zero is unlimited")
      ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
  app->add_option("--tick-timeout-ms", overrides->tick_timeout_ms,
                  "Commit barrier timeout in milliseconds")
      ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
  app->add_option("--startup-timeout-ms", overrides->startup_timeout_ms,
                  "Controller startup timeout in milliseconds")
      ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
  CLI::Option* headless =
      app->add_flag("--headless", overrides->headless,
                    "Run without creating a graphical viewer")
      ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
  CLI::Option* gui =
      app->add_flag("--gui", overrides->gui,
                    "Run with the graphical viewer")
          ->multi_option_policy(CLI::MultiOptionPolicy::Throw);
  headless->excludes(gui);
  gui->excludes(headless);
}

}  // namespace

bool LoadConfigFile(const std::string& path, ServerConfig* config,
                    std::string* error_message) {
  error_message->clear();
  ServerConfig parsed = *config;

  try {
    const std::vector<YAML::Node> documents = YAML::LoadAllFromFile(path);
    if (documents.size() > 1) {
      return ConfigError(path, documents[1].Mark(),
                         "multiple YAML documents are not supported",
                         error_message);
    }
    if (documents.empty() || documents.front().IsNull()) {
      return true;
    }

    const YAML::Node& root = documents.front();
    if (!root.IsMap()) {
      return ConfigError(path, root.Mark(),
                         "configuration root must be a mapping",
                         error_message);
    }

    std::unordered_set<std::string> seen_keys;
    for (const auto& field : root) {
      if (!field.first.IsScalar()) {
        return ConfigError(path, field.first.Mark(),
                           "configuration keys must be strings",
                           error_message);
      }
      const std::string key = field.first.as<std::string>();
      if (!seen_keys.insert(key).second) {
        return ConfigError(path, field.first.Mark(),
                           "duplicate field: " + key, error_message);
      }
      if (!ApplyYamlField(path, key, field.second, &parsed, error_message)) {
        return false;
      }
    }
  } catch (const YAML::Exception& error) {
    return ConfigError(path, error.mark, error.msg, error_message);
  }

  *config = std::move(parsed);
  return true;
}

bool ParseCommandLine(int argc, char** argv, ServerConfig* config,
                      std::string* error_message) {
  error_message->clear();
  CommandLineOverrides overrides;
  CLI::App app{"Run the MuJoCo lockstep simulation server",
               "mujoco-sim-server"};
  ConfigureCommandLine(&app, &overrides);

  try {
    app.parse(argc, argv);
  } catch (const CLI::CallForHelp&) {
    *error_message = app.help();
    return false;
  } catch (const CLI::ParseError& error) {
    std::ostringstream standard_output;
    std::ostringstream standard_error;
    app.exit(error, standard_output, standard_error);
    *error_message = standard_error.str();
    if (error_message->empty()) {
      *error_message = error.what();
    }
    return false;
  }

  ServerConfig parsed = *config;
  if (overrides.config_path.has_value() &&
      !LoadConfigFile(*overrides.config_path, &parsed, error_message)) {
    return false;
  }

  if (overrides.model_path.has_value()) {
    parsed.model_path = *overrides.model_path;
  }
  if (overrides.server_name.has_value()) {
    parsed.server_name = *overrides.server_name;
  }
  if (overrides.max_ticks.has_value()) {
    parsed.max_ticks = *overrides.max_ticks;
  }
  if (overrides.tick_timeout_ms.has_value()) {
    parsed.tick_timeout_ms = *overrides.tick_timeout_ms;
  }
  if (overrides.startup_timeout_ms.has_value()) {
    parsed.startup_timeout_ms = *overrides.startup_timeout_ms;
  }
  if (overrides.headless) {
    parsed.headless = true;
  } else if (overrides.gui) {
    parsed.headless = false;
  }

  if (parsed.model_path.empty()) {
    *error_message = "model path is required";
    return false;
  }
  if (parsed.server_name.empty()) {
    *error_message = "server name is required";
    return false;
  }

  *config = std::move(parsed);
  return true;
}

std::string Usage() {
  CommandLineOverrides overrides;
  CLI::App app{"Run the MuJoCo lockstep simulation server",
               "mujoco-sim-server"};
  ConfigureCommandLine(&app, &overrides);
  return app.help();
}

}  // namespace mujoco_sim::server
