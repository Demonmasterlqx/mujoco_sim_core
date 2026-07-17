#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <span>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mujoco_sim/core/domain.hpp>

namespace mujoco_sim::server {

class ControllerCommandInbox {
 public:
  class RegistrationReservation {
   public:
    RegistrationReservation() = default;
    RegistrationReservation(RegistrationReservation&& other) noexcept;
    RegistrationReservation& operator=(RegistrationReservation&& other) noexcept;
    RegistrationReservation(const RegistrationReservation&) = delete;
    RegistrationReservation& operator=(const RegistrationReservation&) = delete;
    ~RegistrationReservation();

    explicit operator bool() const noexcept { return owner_ != nullptr; }
    bool Commit(std::uint64_t session_id, std::uint64_t epoch,
                std::span<const std::uint64_t> removed_sessions = {});

   private:
    friend class ControllerCommandInbox;
    RegistrationReservation(ControllerCommandInbox* owner, std::unique_lock<std::mutex> lock,
                            bool controller_slot);
    void Reset() noexcept;

    ControllerCommandInbox* owner_ = nullptr;
    std::unique_lock<std::mutex> lock_;
    bool controller_slot_ = false;
  };

  RegistrationReservation PrepareRegistration(bool controller_slot);
  bool Register(std::uint64_t session_id, std::uint64_t epoch = 1);
  void Remove(std::uint64_t session_id);
  bool Publish(mujoco_sim::core::ControllerCommand command);
  bool PublishFault(mujoco_sim::core::ControllerFault fault);
  std::vector<mujoco_sim::core::ControllerCommand> Drain();
  std::vector<mujoco_sim::core::ControllerFault> DrainFaults();
  void Close();

 private:
  struct Slot {
    std::uint64_t epoch = 0;
    bool pending = false;
    mujoco_sim::core::ControllerCommand command;
    std::optional<mujoco_sim::core::ControllerFault> fault;
  };
  static constexpr std::uint64_t kReservedSessionId = 0;

  std::mutex mutex_;
  std::unordered_map<std::uint64_t, Slot> slots_;
  bool closed_ = false;
};

}  // namespace mujoco_sim::server
