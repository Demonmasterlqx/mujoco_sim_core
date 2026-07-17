#include <mujoco_sim/config/runtime.hpp>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <limits>
#include <sstream>
#include <system_error>
#include <utility>
#include <vector>

namespace mujoco_sim::config::runtime {
namespace {

constexpr std::string_view kYamlStringTag = "tag:yaml.org,2002:str";
constexpr std::string_view kYamlBooleanTag = "tag:yaml.org,2002:bool";
constexpr std::string_view kYamlIntegerTag = "tag:yaml.org,2002:int";
constexpr std::string_view kYamlFloatTag = "tag:yaml.org,2002:float";

std::string Lower(std::string_view text) {
  std::string result{text};
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char value) {
    if (value >= 'A' && value <= 'Z') {
      return static_cast<char>(value - 'A' + 'a');
    }
    return static_cast<char>(value);
  });
  return result;
}

bool IsStringNode(const YAML::Node& node) {
  if (!node.IsScalar()) {
    return false;
  }
  const std::string& tag = node.Tag();
  if (tag == "!" || tag == kYamlStringTag) {
    return true;
  }
  if (tag != "?") {
    return false;
  }

  const std::string text = node.Scalar();
  const std::string lower = Lower(text);
  if (text.empty() || lower == "null" || text == "~" || lower == "true" || lower == "false" ||
      lower == ".nan" || lower == ".inf" || lower == "+.inf" || lower == "-.inf") {
    return false;
  }

  std::int64_t signed_value = 0;
  const auto integer = std::from_chars(text.data(), text.data() + text.size(), signed_value, 10);
  if (integer.ec == std::errc{} && integer.ptr == text.data() + text.size()) {
    return false;
  }

  double floating_value = 0.0;
  const auto floating = std::from_chars(text.data(), text.data() + text.size(), floating_value,
                                        std::chars_format::general);
  return !(floating.ec == std::errc{} && floating.ptr == text.data() + text.size());
}

enum class UnsignedResult {
  kSuccess,
  kInvalid,
  kOutOfRange
};

template <typename T>
UnsignedResult ParseUnsignedText(std::string_view text, T* value) {
  if (text.empty() || !std::all_of(text.begin(), text.end(), [](unsigned char character) {
        return character >= '0' && character <= '9';
      })) {
    return UnsignedResult::kInvalid;
  }

  T parsed = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed, 10);
  if (result.ec == std::errc::result_out_of_range) {
    return UnsignedResult::kOutOfRange;
  }
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
    return UnsignedResult::kInvalid;
  }
  *value = parsed;
  return UnsignedResult::kSuccess;
}

bool ParseDoubleText(std::string_view text, double* value) {
  const std::string lower = Lower(text);
  if (lower == ".inf" || lower == "+.inf" || lower == "inf" || lower == "+inf") {
    *value = std::numeric_limits<double>::infinity();
    return true;
  }
  if (lower == "-.inf" || lower == "-inf") {
    *value = -std::numeric_limits<double>::infinity();
    return true;
  }
  if (lower == ".nan" || lower == "nan") {
    *value = std::numeric_limits<double>::quiet_NaN();
    return true;
  }

  double parsed = 0.0;
  const auto result =
      std::from_chars(text.data(), text.data() + text.size(), parsed, std::chars_format::general);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
    return false;
  }
  *value = parsed;
  return true;
}

bool CliError(std::string_view option, std::string_view requirement, std::string* error_message) {
  *error_message = std::string{option} + " " + std::string{requirement};
  return false;
}

template <typename T>
bool ParseYamlUnsigned(const std::filesystem::path& source, const YAML::Node& node,
                       std::string_view field, std::string_view type_name, T* value,
                       std::string* error_message) {
  const std::string& tag = node.Tag();
  if (!node.IsScalar() || (tag != "?" && tag != kYamlIntegerTag)) {
    return ErrorAt(source, node.Mark(), std::string{field} + " must be " + std::string{type_name},
                   error_message);
  }

  const UnsignedResult result = ParseUnsignedText(node.Scalar(), value);
  if (result != UnsignedResult::kSuccess) {
    return ErrorAt(source, node.Mark(),
                   std::string{field} + " must be " + std::string{type_name} + " in range",
                   error_message);
  }
  return true;
}

template <typename T>
bool ParseCliUnsigned(std::string_view text, std::string_view option, std::string_view type_name,
                      T* value, std::string* error_message) {
  if (ParseUnsignedText(text, value) != UnsignedResult::kSuccess) {
    return CliError(option, std::string{"must be "} + std::string{type_name} + " in range",
                    error_message);
  }
  return true;
}

}  // namespace

std::string Location(const std::filesystem::path& source, const YAML::Mark& mark) {
  std::ostringstream output;
  output << source.string();
  if (!mark.is_null()) {
    output << ':' << mark.line + 1 << ':' << mark.column + 1;
  }
  return output.str();
}

bool ErrorAt(const std::filesystem::path& source, const YAML::Mark& mark, std::string_view message,
             std::string* error_message) {
  *error_message = Location(source, mark) + ": " + std::string{message};
  return false;
}

bool LoadSingleMapping(const std::filesystem::path& source, YAML::Node* root,
                       std::string* error_message) {
  try {
    const std::vector<YAML::Node> documents = YAML::LoadAllFromFile(source.string());
    if (documents.size() > 1) {
      return ErrorAt(source, documents[1].Mark(), "multiple YAML documents are not supported",
                     error_message);
    }
    if (documents.empty() || documents.front().IsNull()) {
      *root = YAML::Node{YAML::NodeType::Map};
      return true;
    }
    if (!documents.front().IsMap()) {
      return ErrorAt(source, documents.front().Mark(), "configuration root must be a mapping",
                     error_message);
    }
    *root = documents.front();
    return true;
  } catch (const YAML::Exception& error) {
    return ErrorAt(source, error.mark, error.msg, error_message);
  }
}

bool RequireMapping(const std::filesystem::path& source, const YAML::Node& node,
                    std::string_view field, std::string* error_message) {
  if (!node.IsMap()) {
    return ErrorAt(source, node.Mark(), std::string{field} + " must be a mapping", error_message);
  }
  return true;
}

bool RequireSequence(const std::filesystem::path& source, const YAML::Node& node,
                     std::string_view field, std::string* error_message) {
  if (!node.IsSequence()) {
    return ErrorAt(source, node.Mark(), std::string{field} + " must be a sequence", error_message);
  }
  return true;
}

bool ReadMappingKey(const std::filesystem::path& source, const YAML::Node& node, std::string* key,
                    std::string* error_message) {
  if (!IsStringNode(node)) {
    return ErrorAt(source, node.Mark(), "configuration keys must be strings", error_message);
  }
  *key = node.Scalar();
  return true;
}

bool RegisterKey(const std::filesystem::path& source, const YAML::Node& node,
                 const std::string& key, std::unordered_set<std::string>* seen,
                 std::string* error_message) {
  if (!seen->insert(key).second) {
    return ErrorAt(source, node.Mark(), "duplicate field: " + key, error_message);
  }
  return true;
}

bool ParseString(const std::filesystem::path& source, const YAML::Node& node,
                 std::string_view field, std::string* value, std::string* error_message) {
  if (!IsStringNode(node)) {
    return ErrorAt(source, node.Mark(), std::string{field} + " must be a string", error_message);
  }
  *value = node.Scalar();
  return true;
}

bool ParsePath(const std::filesystem::path& source, const YAML::Node& node, std::string_view field,
               std::filesystem::path* value, std::string* error_message) {
  std::string parsed;
  if (!ParseString(source, node, field, &parsed, error_message)) {
    return false;
  }
  *value = std::filesystem::path{std::move(parsed)};
  return true;
}

bool ParseBool(const std::filesystem::path& source, const YAML::Node& node, std::string_view field,
               bool* value, std::string* error_message) {
  const std::string& tag = node.Tag();
  if (!node.IsScalar() || (tag != "?" && tag != kYamlBooleanTag)) {
    return ErrorAt(source, node.Mark(), std::string{field} + " must be a boolean", error_message);
  }
  const std::string lower = Lower(node.Scalar());
  if (lower == "true") {
    *value = true;
    return true;
  }
  if (lower == "false") {
    *value = false;
    return true;
  }
  return ErrorAt(source, node.Mark(), std::string{field} + " must be a boolean", error_message);
}

bool ParseUInt32(const std::filesystem::path& source, const YAML::Node& node,
                 std::string_view field, std::uint32_t* value, std::string* error_message) {
  return ParseYamlUnsigned(source, node, field, "an unsigned 32-bit integer", value, error_message);
}

bool ParseUInt64(const std::filesystem::path& source, const YAML::Node& node,
                 std::string_view field, std::uint64_t* value, std::string* error_message) {
  return ParseYamlUnsigned(source, node, field, "an unsigned 64-bit integer", value, error_message);
}

bool ParseDouble(const std::filesystem::path& source, const YAML::Node& node,
                 std::string_view field, double* value, std::string* error_message) {
  const std::string& tag = node.Tag();
  if (!node.IsScalar() || (tag != "?" && tag != kYamlFloatTag && tag != kYamlIntegerTag) ||
      !ParseDoubleText(node.Scalar(), value)) {
    return ErrorAt(source, node.Mark(), std::string{field} + " must be a number", error_message);
  }
  return true;
}

bool ParseCliUInt32(std::string_view text, std::string_view option, std::uint32_t* value,
                    std::string* error_message) {
  return ParseCliUnsigned(text, option, "an unsigned 32-bit integer", value, error_message);
}

bool ParseCliUInt64(std::string_view text, std::string_view option, std::uint64_t* value,
                    std::string* error_message) {
  return ParseCliUnsigned(text, option, "an unsigned 64-bit integer", value, error_message);
}

bool ParseCliDouble(std::string_view text, std::string_view option, double* value,
                    std::string* error_message) {
  if (!ParseDoubleText(text, value)) {
    return CliError(option, "must be a number", error_message);
  }
  return true;
}

bool ParseCliBool(std::string_view text, std::string_view option, bool* value,
                  std::string* error_message) {
  const std::string lower = Lower(text);
  if (lower == "true") {
    *value = true;
    return true;
  }
  if (lower == "false") {
    *value = false;
    return true;
  }
  return CliError(option, "must be true or false", error_message);
}

std::string FormatCliError(const CLI::App& app, const CLI::ParseError& error) {
  std::ostringstream standard_output;
  std::ostringstream standard_error;
  app.exit(error, standard_output, standard_error);
  if (!standard_error.str().empty()) {
    return standard_error.str();
  }
  return error.what();
}

}  // namespace mujoco_sim::config::runtime
