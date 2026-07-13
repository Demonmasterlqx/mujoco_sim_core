#pragma once

#include <string>
#include <string_view>

namespace mujoco_sim::protocol {

enum class IdentifierKind {
  kServer,
  kAdapter,
  kStream,
};

bool ValidateIdentifier(std::string_view value, IdentifierKind kind,
                        std::string* error_message);

}  // namespace mujoco_sim::protocol
