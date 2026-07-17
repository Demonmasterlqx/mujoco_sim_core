#include <mujoco_sim/protocol/identifier.hpp>

namespace mujoco_sim::protocol {
namespace {

const char* KindName(IdentifierKind kind) {
  switch (kind) {
    case IdentifierKind::kServer:
      return "server_name";
    case IdentifierKind::kAdapter:
      return "adapter_id";
    case IdentifierKind::kStream:
      return "stream_name";
  }
  return "identifier";
}

bool IsAlphanumeric(unsigned char value) {
  return (value >= 'A' && value <= 'Z') || (value >= 'a' && value <= 'z') ||
         (value >= '0' && value <= '9');
}

}  // namespace

bool ValidateIdentifier(std::string_view value, IdentifierKind kind, std::string* error_message) {
  const auto fail = [&](const char* reason) {
    if (error_message != nullptr) {
      *error_message = std::string(KindName(kind)) + " " + reason;
    }
    return false;
  };
  if (value.empty()) return fail("must not be empty");
  if (value.size() > 64) return fail("must not exceed 64 characters");
  if (!IsAlphanumeric(static_cast<unsigned char>(value.front()))) {
    return fail("must start with an ASCII letter or digit");
  }
  for (const unsigned char character : value) {
    if (!IsAlphanumeric(character) && character != '_' && character != '-') {
      return fail("may contain only ASCII letters, digits, '_' and '-'");
    }
  }
  if (error_message != nullptr) error_message->clear();
  return true;
}

}  // namespace mujoco_sim::protocol
