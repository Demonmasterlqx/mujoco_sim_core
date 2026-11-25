#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <string_view>
#include <thread>

#include <fcntl.h>
#include <unistd.h>

#include "adapter_app.hpp"
#include "terminal_keyboard_input.hpp"

namespace demo = mujoco_sim::wbr_lqr_demo;

namespace {

using namespace std::chrono_literals;

std::atomic<bool> disconnect_mode = false;
std::atomic<bool> disconnect_read_entered = false;
std::atomic<bool> release_disconnect = false;

class FakeTerminalSource final : public demo::KeyboardByteSource {
 public:
  explicit FakeTerminalSource(bool disconnect) : disconnect_(disconnect) {
    int descriptors[2] = {-1, -1};
    if (pipe2(descriptors, O_CLOEXEC | O_NONBLOCK) != 0) return;
    read_fd_ = descriptors[0];
    write_fd_ = descriptors[1];
    if (disconnect_) {
      const char token = 1;
      if (write(write_fd_, &token, sizeof(token)) == 1) return;
      close(read_fd_);
      close(write_fd_);
      read_fd_ = -1;
      write_fd_ = -1;
    }
  }

  ~FakeTerminalSource() override {
    if (read_fd_ >= 0) close(read_fd_);
    if (write_fd_ >= 0) close(write_fd_);
  }

  int PollFileDescriptor() const noexcept override { return read_fd_; }

  demo::KeyboardReadResult Read(char*, std::string*) override {
    if (!disconnect_ || delivered_) return demo::KeyboardReadResult::kWouldBlock;
    char token = 0;
    while (read(read_fd_, &token, sizeof(token)) < 0 && errno == EINTR) {
    }
    disconnect_read_entered = true;
    while (!release_disconnect) {
      std::this_thread::yield();
    }
    delivered_ = true;
    return demo::KeyboardReadResult::kDisconnected;
  }

  std::string_view Name() const noexcept override { return "fake-terminal-e2e"; }

 private:
  bool disconnect_ = false;
  int read_fd_ = -1;
  int write_fd_ = -1;
  bool delivered_ = false;
};

std::unique_ptr<demo::TerminalKeyboardInput> CreateFakeKeyboard(std::uint32_t direction_hold_ticks,
                                                                std::string* error_message) {
  const bool disconnect = disconnect_mode;
  auto keyboard = demo::TerminalKeyboardInput::CreateForTesting(
      std::make_unique<FakeTerminalSource>(disconnect), direction_hold_ticks, error_message);
  if (keyboard && !disconnect) {
    keyboard->InjectKeyForTesting(' ');
    keyboard->InjectKeyForTesting('w');
  }
  return keyboard;
}

bool WaitForDisconnectRead(std::string* error_message) {
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!disconnect_read_entered && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  if (disconnect_read_entered) return true;
  *error_message = "fake terminal reader did not reach Read()";
  return false;
}

bool ReleaseDisconnect(std::string*) {
  release_disconnect = true;
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  const char* scenario = std::getenv("WBR_FAKE_TERMINAL_CASE");
  disconnect_mode = scenario != nullptr && std::string_view(scenario) == "disconnect";
  disconnect_read_entered = false;
  release_disconnect = false;
  const demo::AdapterTestHooks hooks = {
      .after_keyboard_start = &WaitForDisconnectRead,
      .before_keyboard_stop = &ReleaseDisconnect,
  };
  return demo::RunAdapter(argc, argv, &CreateFakeKeyboard, disconnect_mode ? &hooks : nullptr);
}
