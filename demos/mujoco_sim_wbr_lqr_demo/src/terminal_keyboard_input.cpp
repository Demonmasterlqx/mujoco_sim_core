#include "terminal_keyboard_input.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <exception>
#include <string>
#include <utility>

#include <fcntl.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <unistd.h>

namespace mujoco_sim::wbr_lqr_demo {
namespace {

std::string ErrnoMessage(int error_number) {
  return std::strerror(error_number);
}

class StandardInputSource final : public KeyboardByteSource {
 public:
  int PollFileDescriptor() const noexcept override { return STDIN_FILENO; }

  KeyboardReadResult Read(char* key, std::string* error_message) override {
    const ssize_t count = read(STDIN_FILENO, key, 1);
    if (count == 1) return KeyboardReadResult::kKey;
    if (count == 0) return KeyboardReadResult::kDisconnected;
    if (errno == EINTR) return KeyboardReadResult::kWouldBlock;
    if (errno == EAGAIN || errno == EWOULDBLOCK) return KeyboardReadResult::kWouldBlock;
    *error_message = "failed to read terminal standard input: " + ErrnoMessage(errno);
    return KeyboardReadResult::kError;
  }

  std::string_view Name() const noexcept override { return "standard input"; }
};

void DrainEventFd(int fd) noexcept {
  std::uint64_t value = 0;
  while (read(fd, &value, sizeof(value)) < 0 && errno == EINTR) {
  }
}

}  // namespace

TerminalInputMailbox::TerminalInputMailbox(std::uint32_t direction_hold_ticks)
    : direction_hold_ticks_(std::max(direction_hold_ticks, 1U)) {}

void TerminalInputMailbox::HandleKey(char key) {
  const unsigned char byte = static_cast<unsigned char>(key);
  const char normalized = static_cast<char>(std::tolower(byte));
  std::lock_guard lock(mutex_);
  switch (normalized) {
    case 'w':
      direction_ticks_[static_cast<std::size_t>(Direction::kW)] = direction_hold_ticks_;
      break;
    case 's':
      direction_ticks_[static_cast<std::size_t>(Direction::kS)] = direction_hold_ticks_;
      break;
    case 'a':
      direction_ticks_[static_cast<std::size_t>(Direction::kA)] = direction_hold_ticks_;
      break;
    case 'd':
      direction_ticks_[static_cast<std::size_t>(Direction::kD)] = direction_hold_ticks_;
      break;
    case 'q':
      pending_one_shots_.q = true;
      pending_one_shots_.e = false;
      pending_one_shots_.f = false;
      break;
    case 'e':
      pending_one_shots_.q = false;
      pending_one_shots_.e = true;
      pending_one_shots_.f = false;
      break;
    case 'f':
      pending_one_shots_.q = false;
      pending_one_shots_.e = false;
      pending_one_shots_.f = true;
      break;
    case ' ':
      ClearDirectionsLocked();
      pending_one_shots_.space = !pending_one_shots_.space;
      break;
    case 'x':
      ClearDirectionsLocked();
      break;
    case '\x1b':
      ClearDirectionsLocked();
      pending_one_shots_ = {};
      exit_requested_ = true;
      break;
    default:
      break;
  }
}

control::input_snapshot_t TerminalInputMailbox::Consume() {
  std::lock_guard lock(mutex_);
  control::input_snapshot_t input = pending_one_shots_;
  input.w = direction_ticks_[static_cast<std::size_t>(Direction::kW)] > 0;
  input.s = direction_ticks_[static_cast<std::size_t>(Direction::kS)] > 0;
  input.a = direction_ticks_[static_cast<std::size_t>(Direction::kA)] > 0;
  input.d = direction_ticks_[static_cast<std::size_t>(Direction::kD)] > 0;
  for (auto& remaining_ticks : direction_ticks_) {
    if (remaining_ticks > 0) --remaining_ticks;
  }
  pending_one_shots_ = {};
  return input;
}

void TerminalInputMailbox::Clear() {
  std::lock_guard lock(mutex_);
  ClearDirectionsLocked();
  pending_one_shots_ = {};
}

void TerminalInputMailbox::ClearDirectionsLocked() noexcept {
  direction_ticks_.fill(0);
}

bool TerminalInputMailbox::ExitRequested() const {
  std::lock_guard lock(mutex_);
  return exit_requested_;
}

std::unique_ptr<TerminalKeyboardInput> TerminalKeyboardInput::OpenStandardInput(
    std::uint32_t direction_hold_ticks, std::string* error_message) {
  if (error_message == nullptr) return nullptr;
  error_message->clear();
  if (isatty(STDIN_FILENO) == 0) {
    *error_message = "terminal input mode requires standard input to be an interactive TTY";
    return nullptr;
  }
  termios original{};
  if (tcgetattr(STDIN_FILENO, &original) != 0) {
    *error_message = "failed to read standard input terminal settings: " + ErrnoMessage(errno);
    return nullptr;
  }

  const int stop_fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
  if (stop_fd < 0) {
    *error_message = "failed to create terminal reader stop eventfd: " + ErrnoMessage(errno);
    return nullptr;
  }
  return std::unique_ptr<TerminalKeyboardInput>(new TerminalKeyboardInput(
      std::make_unique<StandardInputSource>(), stop_fd, direction_hold_ticks, &original));
}

std::unique_ptr<TerminalKeyboardInput> TerminalKeyboardInput::CreateForTesting(
    std::unique_ptr<KeyboardByteSource> source, std::uint32_t direction_hold_ticks,
    std::string* error_message) {
  if (error_message == nullptr) return nullptr;
  error_message->clear();
  if (!source || source->PollFileDescriptor() < 0) {
    *error_message = "terminal keyboard byte source is invalid";
    return nullptr;
  }
  const int stop_fd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
  if (stop_fd < 0) {
    *error_message = "failed to create terminal reader stop eventfd: " + ErrnoMessage(errno);
    return nullptr;
  }
  return std::unique_ptr<TerminalKeyboardInput>(
      new TerminalKeyboardInput(std::move(source), stop_fd, direction_hold_ticks, nullptr));
}

TerminalKeyboardInput::TerminalKeyboardInput(std::unique_ptr<KeyboardByteSource> source,
                                             int stop_fd, std::uint32_t direction_hold_ticks,
                                             const termios* original_terminal)
    : source_(std::move(source)), stop_fd_(stop_fd), mailbox_(direction_hold_ticks) {
  if (original_terminal != nullptr) {
    original_terminal_ = *original_terminal;
    manages_terminal_ = true;
  }
}

TerminalKeyboardInput::~TerminalKeyboardInput() noexcept {
  RequestStop();
  Join();
  if (stop_fd_ >= 0) close(stop_fd_);
}

void TerminalKeyboardInput::InjectKeyForTesting(char key) {
  mailbox_.HandleKey(key);
}

bool TerminalKeyboardInput::EnableImmediateMode(std::string* error_message) {
  if (!manages_terminal_ || terminal_active_) return true;
  termios immediate = original_terminal_;
  immediate.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  immediate.c_cc[VMIN] = 0;
  immediate.c_cc[VTIME] = 0;
  if (tcsetattr(STDIN_FILENO, TCSANOW, &immediate) != 0) {
    *error_message = "failed to enable immediate terminal input: " + ErrnoMessage(errno);
    return false;
  }
  terminal_active_ = true;
  return true;
}

bool TerminalKeyboardInput::Start(std::string* error_message) {
  if (error_message == nullptr) return false;
  error_message->clear();
  if (reader_.joinable()) {
    *error_message = "terminal keyboard reader is already running";
    return false;
  }
  if (!EnableImmediateMode(error_message)) return false;
  DrainEventFd(stop_fd_);
  stop_requested_ = false;
  try {
    reader_ = std::jthread([this] { Run(); });
  } catch (const std::exception& error) {
    *error_message = "failed to start terminal keyboard reader: " + std::string(error.what());
    RestoreTerminal();
    return false;
  } catch (...) {
    *error_message = "failed to start terminal keyboard reader: unknown exception";
    RestoreTerminal();
    return false;
  }
  return true;
}

void TerminalKeyboardInput::RequestStop() noexcept {
  stop_requested_ = true;
  if (stop_fd_ < 0) return;
  const std::uint64_t value = 1;
  while (write(stop_fd_, &value, sizeof(value)) < 0 && errno == EINTR) {
  }
}

void TerminalKeyboardInput::Join() noexcept {
  if (reader_.joinable() && reader_.get_id() != std::this_thread::get_id()) {
    try {
      reader_.join();
    } catch (const std::exception& error) {
      RecordError(error.what());
    } catch (...) {
      RecordError("terminal keyboard reader join failed: unknown exception");
    }
  }
  RestoreTerminal();
}

void TerminalKeyboardInput::RestoreTerminal() noexcept {
  if (!terminal_active_) return;
  if (tcsetattr(STDIN_FILENO, TCSANOW, &original_terminal_) != 0) {
    const int error_number = errno;
    terminal_active_ = false;
    try {
      RecordError("failed to restore standard input terminal settings: " +
                  ErrnoMessage(error_number));
    } catch (...) {
      RecordError("failed to restore standard input terminal settings");
    }
    return;
  }
  terminal_active_ = false;
}

control::input_snapshot_t TerminalKeyboardInput::Consume() {
  return mailbox_.Consume();
}

void TerminalKeyboardInput::Clear() {
  mailbox_.Clear();
}

bool TerminalKeyboardInput::ExitRequested() const {
  return mailbox_.ExitRequested();
}

bool TerminalKeyboardInput::TakeError(std::string* error_message) {
  if (error_message == nullptr) return false;
  if (!reader_failed_.exchange(false)) return false;
  std::lock_guard lock(error_mutex_);
  *error_message = terminal_error_.empty() ? "terminal reader failed without a diagnostic"
                                           : std::move(terminal_error_);
  terminal_error_.clear();
  return true;
}

std::string_view TerminalKeyboardInput::source_name() const noexcept {
  return source_->Name();
}

void TerminalKeyboardInput::Run() noexcept {
  try {
    const int input_fd = source_->PollFileDescriptor();
    std::array<pollfd, 2> descriptors = {{{input_fd, POLLIN, 0}, {stop_fd_, POLLIN, 0}}};
    while (!stop_requested_) {
      const int result = poll(descriptors.data(), descriptors.size(), -1);
      if (result < 0) {
        if (errno == EINTR) continue;
        RecordError("terminal input poll failed for '" + std::string(source_->Name()) +
                    "': " + ErrnoMessage(errno));
        return;
      }
      if ((descriptors[1].revents & POLLIN) != 0) {
        DrainEventFd(stop_fd_);
        return;
      }
      if ((descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        RecordError("terminal input disconnected: " + std::string(source_->Name()));
        return;
      }
      if ((descriptors[0].revents & POLLIN) != 0 && !DrainKeys()) return;
      descriptors[0].revents = 0;
      descriptors[1].revents = 0;
    }
  } catch (const std::exception& error) {
    RecordError(error.what());
  } catch (...) {
    RecordError("terminal keyboard reader failed: unknown exception");
  }
}

bool TerminalKeyboardInput::DrainKeys() {
  while (!stop_requested_) {
    char key = 0;
    std::string error;
    switch (source_->Read(&key, &error)) {
      case KeyboardReadResult::kKey:
        mailbox_.HandleKey(key);
        return true;
      case KeyboardReadResult::kWouldBlock:
        return true;
      case KeyboardReadResult::kDisconnected:
        RecordError("terminal input disconnected: " + std::string(source_->Name()));
        return false;
      case KeyboardReadResult::kError:
        RecordError(error.empty() ? "terminal input read failed: " + std::string(source_->Name())
                                  : std::move(error));
        return false;
    }
  }
  return false;
}

void TerminalKeyboardInput::RecordError(std::string_view message) noexcept {
  try {
    mailbox_.Clear();
    std::lock_guard lock(error_mutex_);
    if (terminal_error_.empty()) terminal_error_.assign(message);
  } catch (...) {
  }
  reader_failed_ = true;
}

}  // namespace mujoco_sim::wbr_lqr_demo
