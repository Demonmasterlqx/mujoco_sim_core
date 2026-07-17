#include "runtime_test_config.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

namespace {

using mujoco_sim::config::test::RuntimeConfig;

int failures = 0;

void Check(bool condition, const char* expression, const char* file, int line) {
  if (!condition) {
    std::cerr << file << ':' << line << ": check failed: " << expression << '\n';
    ++failures;
  }
}

#define CHECK(expression) Check((expression), #expression, __FILE__, __LINE__)

class ConfigFile {
 public:
  ConfigFile(std::string label, const std::string& contents) {
    static std::uint64_t sequence = 0;
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
            ("mujoco_sim_config_" + std::move(label) + "_" + std::to_string(stamp) + "_" +
             std::to_string(sequence++) + ".yaml");
    std::ofstream stream(path_);
    stream << contents;
  }

  ~ConfigFile() {
    std::error_code error;
    std::filesystem::remove(path_, error);
  }

  const std::filesystem::path& path() const { return path_; }

 private:
  std::filesystem::path path_;
};

bool SameSentinel(const RuntimeConfig& config) {
  return config.name == "sentinel" && config.model == "sentinel.xml" && config.retries == 5 &&
         config.cycles == 99 && !config.enabled;
}

RuntimeConfig Sentinel() {
  RuntimeConfig config;
  config.name = "sentinel";
  config.model = "sentinel.xml";
  config.retries = 5;
  config.cycles = 99;
  config.enabled = false;
  return config;
}

bool ParseArgs(std::vector<std::string> arguments, RuntimeConfig* config,
               std::string* error_message) {
  std::vector<const char*> argv;
  argv.reserve(arguments.size());
  for (const std::string& argument : arguments) {
    argv.push_back(argument.c_str());
  }
  return mujoco_sim::config::test::Parse(static_cast<int>(argv.size()), argv.data(), config,
                                         error_message);
}

std::string RequiredYaml() {
  return "model: robot.xml\n"
         "workers:\n"
         "  - id: 7\n"
         "    label: worker\n"
         "    axes: [1, 2]\n";
}

void TestDefaultsAndAllTypes() {
  ConfigFile minimal("minimal", RequiredYaml());
  RuntimeConfig defaults;
  std::string error;
  CHECK(mujoco_sim::config::test::Load(minimal.path(), &defaults, &error));
  CHECK(error.empty());
  CHECK(defaults.name == "simulator");
  CHECK(defaults.model == minimal.path().parent_path() / "robot.xml");
  CHECK(defaults.enabled);
  CHECK(defaults.retries == 3);
  CHECK(defaults.cycles == 10);
  CHECK(defaults.gain == 0.5);
  CHECK(defaults.code == "sim");
  CHECK(defaults.tags == std::vector<std::string>{"base"});
  CHECK(defaults.gravity[2] == -9.81);
  CHECK(defaults.mode == RuntimeConfig::Mode::kLockstep);
  CHECK(defaults.workers.size() == 1);
  CHECK(defaults.workers[0].policy == RuntimeConfig::WorkersItem::Policy::kBackup);

  ConfigFile all("all",
                 "name: yaml\n"
                 "model: /tmp/model.xml\n"
                 "enabled: false\n"
                 "retries: 4\n"
                 "cycles: 18446744073709551615\n"
                 "gain: 0.75\n"
                 "code: abc\n"
                 "tags: [one, two]\n"
                 "gravity: [1, 2.5, -3]\n"
                 "mode: real-time\n"
                 "network:\n"
                 "  host: localhost\n"
                 "  port: 9000\n"
                 "workers:\n"
                 "  - id: 42\n"
                 "    label: main\n"
                 "    axes: [3, 4]\n"
                 "    policy: primary\n");
  RuntimeConfig config;
  CHECK(mujoco_sim::config::test::Load(all.path(), &config, &error));
  CHECK(error.empty());
  CHECK(config.name == "yaml");
  CHECK(config.model == "/tmp/model.xml");
  CHECK(!config.enabled);
  CHECK(config.retries == 4);
  CHECK(config.cycles == UINT64_MAX);
  CHECK(config.gain == 0.75);
  CHECK(config.tags == (std::vector<std::string>{"one", "two"}));
  CHECK(config.gravity[0] == 1.0 && config.gravity[1] == 2.5 && config.gravity[2] == -3.0);
  CHECK(config.mode == RuntimeConfig::Mode::kRealTime);
  CHECK(config.network.host == "localhost");
  CHECK(config.network.port == 9000);
  CHECK(config.workers.size() == 1);
  CHECK(config.workers[0].id == 42);
  CHECK(config.workers[0].axes[1] == 4);
  CHECK(config.workers[0].policy == RuntimeConfig::WorkersItem::Policy::kPrimary);

  RuntimeConfig cli_path;
  CHECK(ParseArgs(
      {"runtime-config-test", "--model", "cli-relative.xml", "--config", minimal.path().string()},
      &cli_path, &error));
  CHECK(cli_path.model == "cli-relative.xml");
}

void ExpectYamlFailure(const std::string& label, const std::string& contents,
                       const std::string& expected, bool expect_location = true) {
  ConfigFile file(label, contents);
  RuntimeConfig config = Sentinel();
  std::string error;
  CHECK(!mujoco_sim::config::test::Load(file.path(), &config, &error));
  if (error.find(expected) == std::string::npos) {
    std::cerr << "case " << label << ": expected '" << expected << "' in '" << error << "'\n";
  }
  CHECK(error.find(expected) != std::string::npos);
  CHECK(SameSentinel(config));
  if (expect_location) {
    CHECK(error.find(file.path().string() + ":") != std::string::npos);
  }
}

void TestStrictYaml() {
  ExpectYamlFailure("root", "- model\n- robot.xml\n", "root must be a mapping");
  ExpectYamlFailure("documents", RequiredYaml() + "---\n" + RequiredYaml(),
                    "multiple YAML documents");
  ExpectYamlFailure("syntax", "model: [\n", "end of sequence");
  ExpectYamlFailure("unknown", RequiredYaml() + "unknown: value\n", "unknown field: unknown");
  ExpectYamlFailure("duplicate", "model: one.xml\nmodel: two.xml\nworkers: []\n",
                    "duplicate field: model");
  ExpectYamlFailure("key", "1: value\n" + RequiredYaml(), "keys must be strings");
  ExpectYamlFailure("string_type", RequiredYaml() + "name: 12\n", "name must be a string");
  ExpectYamlFailure("bool_type", RequiredYaml() + "enabled: \"true\"\n",
                    "enabled must be a boolean");
  ExpectYamlFailure("uint_type", RequiredYaml() + "retries: \"2\"\n", "unsigned 32-bit integer");
  ExpectYamlFailure("uint32_overflow", RequiredYaml() + "retries: 4294967296\n", "in range");
  ExpectYamlFailure("uint64_overflow", RequiredYaml() + "cycles: 18446744073709551616\n",
                    "in range");
  ExpectYamlFailure("double_type", RequiredYaml() + "gain: \"0.5\"\n", "gain must be a number");
  ExpectYamlFailure("vector_type", RequiredYaml() + "tags: value\n", "tags must be a sequence");
  ExpectYamlFailure("vector_item", RequiredYaml() + "tags: [ok, 2]\n", "tags[1] must be a string");
  ExpectYamlFailure("array_size", RequiredYaml() + "gravity: [1, 2]\n", "exactly 3 items");
  ExpectYamlFailure("object_type", RequiredYaml() + "network: host\n", "network must be a mapping");
  ExpectYamlFailure("nested_unknown", RequiredYaml() + "network:\n  host: localhost\n  typo: 1\n",
                    "unknown field: network.typo");
  ExpectYamlFailure("sequence_type", "model: x\nworkers: value\n", "workers must be a sequence");
  ExpectYamlFailure("enum", RequiredYaml() + "mode: unknown\n", "mode must be one of");
}

void TestConstraintsAndRequired() {
  ExpectYamlFailure("empty_path",
                    "model: \"\"\nworkers:\n"
                    "  - {id: 1, label: w, axes: [1, 2]}\n",
                    "model must be non-empty");
  ExpectYamlFailure("required_model", "workers:\n  - id: 1\n    label: w\n    axes: [1, 2]\n",
                    "model is required");
  ExpectYamlFailure("required_workers", "model: robot.xml\n", "workers is required");
  ExpectYamlFailure("required_item", "model: robot.xml\nworkers:\n  - label: w\n    axes: [1, 2]\n",
                    "workers[0].id is required");
  ExpectYamlFailure("nested_required", RequiredYaml() + "network:\n  port: 7000\n",
                    "network.host is required");
  ExpectYamlFailure("non_empty", RequiredYaml() + "name: \"\"\n", "name must be non-empty");
  ExpectYamlFailure("minimum", RequiredYaml() + "retries: 0\n", "retries must be >= 1");
  ExpectYamlFailure("maximum", RequiredYaml() + "gain: 1.5\n", "gain must be <= 1");
  ExpectYamlFailure("finite", RequiredYaml() + "gain: .inf\n", "gain must be finite");
  ExpectYamlFailure("size", RequiredYaml() + "code: long\n", "code must contain exactly 3 items");
  ExpectYamlFailure("unique", RequiredYaml() + "tags: [same, same]\n",
                    "tags must contain unique items");
  ExpectYamlFailure("array_unique",
                    "model: robot.xml\nworkers:\n  - id: 1\n    label: w\n"
                    "    axes: [2, 2]\n",
                    "workers[0].axes must contain unique items");
  ExpectYamlFailure("sequence_non_empty", "model: robot.xml\nworkers: []\n",
                    "workers must be non-empty");
}

void TestCliPrecedenceAndOrder() {
  ConfigFile file("precedence",
                  "name: yaml\n"
                  "model: yaml.xml\n"
                  "enabled: true\n"
                  "retries: 2\n"
                  "cycles: 20\n"
                  "gain: 0.25\n"
                  "code: yml\n"
                  "tags: [yaml]\n"
                  "gravity: [0, 0, -1]\n"
                  "mode: real-time\n"
                  "network: {host: yaml-host, port: 7001}\n"
                  "workers:\n"
                  "  - {id: 1, label: yaml-worker, axes: [1, 2]}\n");
  const std::vector<std::vector<std::string>> orders = {
      {"runtime-config-test",
       "--config",
       file.path().string(),
       "--name",
       "cli",
       "--model",
       "cli.xml",
       "--disabled",
       "--retries",
       "4",
       "--cycles",
       "99",
       "--gain",
       "0.8",
       "--code",
       "cli",
       "--tag",
       "one",
       "two",
       "--gravity",
       "1",
       "2",
       "3",
       "--mode",
       "lockstep",
       "--host",
       "cli-host",
       "--port",
       "8000"},
      {"runtime-config-test",
       "--name",
       "cli",
       "--model",
       "cli.xml",
       "--disabled",
       "--retries",
       "4",
       "--cycles",
       "99",
       "--gain",
       "0.8",
       "--code",
       "cli",
       "--tag",
       "one",
       "two",
       "--gravity",
       "1",
       "2",
       "3",
       "--mode",
       "lockstep",
       "--host",
       "cli-host",
       "--port",
       "8000",
       "--config",
       file.path().string()},
      {"runtime-config-test",
       "--config",
       file.path().string(),
       "--name",
       "cli",
       "--model",
       "cli.xml",
       "--disabled",
       "--retries",
       "4",
       "--cycles",
       "99",
       "--gain",
       "0.8",
       "--code",
       "cli",
       "--tag",
       "one,two",
       "--gravity",
       "1",
       "2",
       "3",
       "--mode",
       "lockstep",
       "--host",
       "cli-host",
       "--port",
       "8000"},
  };

  for (const auto& arguments : orders) {
    RuntimeConfig config;
    std::string error;
    CHECK(ParseArgs(arguments, &config, &error));
    CHECK(error.empty());
    CHECK(config.name == "cli");
    CHECK(config.model == "cli.xml");
    CHECK(!config.enabled);
    CHECK(config.retries == 4);
    CHECK(config.cycles == 99);
    CHECK(config.gain == 0.8);
    CHECK(config.code == "cli");
    CHECK(config.tags == (std::vector<std::string>{"one", "two"}));
    CHECK(config.gravity == (std::array<double, 3>{1, 2, 3}));
    CHECK(config.mode == RuntimeConfig::Mode::kLockstep);
    CHECK(config.network.host == "cli-host");
    CHECK(config.network.port == 8000);
    CHECK(config.workers.size() == 1);
    CHECK(config.workers[0].label == "yaml-worker");
  }
}

void TestCliFailuresHelpAndTransactions() {
  RuntimeConfig help_config = Sentinel();
  std::string error;
  CHECK(!ParseArgs({"runtime-config-test", "--help"}, &help_config, &error));
  CHECK(error == mujoco_sim::config::test::Usage());
  CHECK(error.find("--config") != std::string::npos);
  CHECK(error.find("--disabled") != std::string::npos);
  CHECK(error.find("--gravity") != std::string::npos);
  CHECK(SameSentinel(help_config));

  const std::vector<std::vector<std::string>> invalid = {
      {"runtime-config-test", "--unknown"},
      {"runtime-config-test", "--retries", "4294967296"},
      {"runtime-config-test", "--cycles", "18446744073709551616"},
      {"runtime-config-test", "--gain", "nan"},
      {"runtime-config-test", "--enabled", "--disabled"},
      {"runtime-config-test", "--gravity", "1", "2"},
      {"runtime-config-test", "--model", "robot.xml"},
  };
  for (const auto& arguments : invalid) {
    RuntimeConfig config = Sentinel();
    error.clear();
    CHECK(!ParseArgs(arguments, &config, &error));
    CHECK(!error.empty());
    CHECK(SameSentinel(config));
  }
}

}  // namespace

int main() {
  TestDefaultsAndAllTypes();
  TestStrictYaml();
  TestConstraintsAndRequired();
  TestCliPrecedenceAndOrder();
  TestCliFailuresHelpAndTransactions();
  if (failures != 0) {
    std::cerr << failures << " test checks failed\n";
    return 1;
  }
  return 0;
}
