#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <unordered_set>

#include <CLI/CLI.hpp>
#include <yaml-cpp/yaml.h>

namespace mujoco_sim::config::runtime {

std::string Location(const std::filesystem::path& source,
                     const YAML::Mark& mark);

bool ErrorAt(const std::filesystem::path& source, const YAML::Mark& mark,
             std::string_view message, std::string* error_message);

bool LoadSingleMapping(const std::filesystem::path& source, YAML::Node* root,
                       std::string* error_message);

bool RequireMapping(const std::filesystem::path& source,
                    const YAML::Node& node, std::string_view field,
                    std::string* error_message);

bool RequireSequence(const std::filesystem::path& source,
                     const YAML::Node& node, std::string_view field,
                     std::string* error_message);

bool ReadMappingKey(const std::filesystem::path& source,
                    const YAML::Node& node, std::string* key,
                    std::string* error_message);

bool RegisterKey(const std::filesystem::path& source,
                 const YAML::Node& node, const std::string& key,
                 std::unordered_set<std::string>* seen,
                 std::string* error_message);

bool ParseString(const std::filesystem::path& source, const YAML::Node& node,
                 std::string_view field, std::string* value,
                 std::string* error_message);

bool ParsePath(const std::filesystem::path& source, const YAML::Node& node,
               std::string_view field, std::filesystem::path* value,
               std::string* error_message);

bool ParseBool(const std::filesystem::path& source, const YAML::Node& node,
               std::string_view field, bool* value,
               std::string* error_message);

bool ParseUInt32(const std::filesystem::path& source, const YAML::Node& node,
                 std::string_view field, std::uint32_t* value,
                 std::string* error_message);

bool ParseUInt64(const std::filesystem::path& source, const YAML::Node& node,
                 std::string_view field, std::uint64_t* value,
                 std::string* error_message);

bool ParseDouble(const std::filesystem::path& source, const YAML::Node& node,
                 std::string_view field, double* value,
                 std::string* error_message);

bool ParseCliUInt32(std::string_view text, std::string_view option,
                    std::uint32_t* value, std::string* error_message);

bool ParseCliUInt64(std::string_view text, std::string_view option,
                    std::uint64_t* value, std::string* error_message);

bool ParseCliDouble(std::string_view text, std::string_view option,
                    double* value, std::string* error_message);

bool ParseCliBool(std::string_view text, std::string_view option, bool* value,
                  std::string* error_message);

std::string FormatCliError(const CLI::App& app,
                           const CLI::ParseError& error);

}  // namespace mujoco_sim::config::runtime
