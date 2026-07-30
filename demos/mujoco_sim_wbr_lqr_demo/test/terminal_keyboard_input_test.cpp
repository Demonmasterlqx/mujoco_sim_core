#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "process_signal.hpp"
#include "terminal_keyboard_input.hpp"

namespace demo = mujoco_sim::wbr_lqr_demo;

namespace {

using namespace std::chrono_literals;

volatile std::sig_atomic_t previous_sigterm_called = 0;

void PreviousSigtermHandler(int) {
  previous_sigterm_called = 1;
}

class FakeByteSource final : public demo::KeyboardByteSource {
 public:
  FakeByteSource() {
    int descriptors[2] = {-1, -1};
    if (pipe2(descriptors, O_CLOEXEC | O_NONBLOCK) != 0) return;
    read_fd_ = descriptors[0];
    write_fd_ = descriptors[1];
  }

  ~FakeByteSource() override {
    if (read_fd_ >= 0) close(read_fd_);
    if (write_fd_ >= 0) close(write_fd_);
  }

  int PollFileDescriptor() const noexcept override { return read_fd_; }

  demo::KeyboardReadResult Read(char* key, std::string* error_message) override {
    std::lock_guard lock(mutex_);
    if (items_.empty()) return demo::KeyboardReadResult::kWouldBlock;
    char token = 0;
    static_cast<void>(read(read_fd_, &token, sizeof(token)));
    Item item = std::move(items_.front());
    items_.pop_front();
    if (item.result == demo::KeyboardReadResult::kKey) *key = item.key;
    if (item.result == demo::KeyboardReadResult::kError) *error_message = item.error;
    return item.result;
  }

  std::string_view Name() const noexcept override { return "fake-terminal"; }

  void Push(char key) { PushItem({demo::KeyboardReadResult::kKey, key, {}}); }

  void PushResult(demo::KeyboardReadResult result, std::string error = {}) {
    PushItem({result, 0, std::move(error)});
  }

 private:
  struct Item {
    demo::KeyboardReadResult result = demo::KeyboardReadResult::kWouldBlock;
    char key = 0;
    std::string error;
  };

  void PushItem(Item item) {
    {
      std::lock_guard lock(mutex_);
      items_.push_back(std::move(item));
    }
    const char token = 1;
    ASSERT_EQ(write(write_fd_, &token, sizeof(token)), 1);
  }

  int read_fd_ = -1;
  int write_fd_ = -1;
  std::mutex mutex_;
  std::deque<Item> items_;
};

class ScopedStdinRedirect {
 public:
  explicit ScopedStdinRedirect(int replacement_fd) : saved_fd_(dup(STDIN_FILENO)) {
    if (saved_fd_ >= 0 && dup2(replacement_fd, STDIN_FILENO) != STDIN_FILENO) {
      close(saved_fd_);
      saved_fd_ = -1;
    }
  }

  ~ScopedStdinRedirect() {
    if (saved_fd_ < 0) return;
    static_cast<void>(dup2(saved_fd_, STDIN_FILENO));
    close(saved_fd_);
  }

  bool valid() const { return saved_fd_ >= 0; }

 private:
  int saved_fd_ = -1;
};

template <typename Predicate>
bool WaitUntil(Predicate predicate) {
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) return true;
    std::this_thread::yield();
  }
  return predicate();
}

void ExpectReleased(const control::input_snapshot_t& input) {
  EXPECT_FALSE(input.w);
  EXPECT_FALSE(input.s);
  EXPECT_FALSE(input.a);
  EXPECT_FALSE(input.d);
  EXPECT_FALSE(input.q);
  EXPECT_FALSE(input.e);
  EXPECT_FALSE(input.f);
  EXPECT_FALSE(input.space);
}

}  // namespace

TEST(TerminalInputMailboxTest, WasdActionsRemainActiveForConfiguredTicks) {
  using DirectionMember = bool control::input_snapshot_t::*;
  constexpr std::array<std::pair<char, DirectionMember>, 4> directions = {{
      {'w', &control::input_snapshot_t::w},
      {'S', &control::input_snapshot_t::s},
      {'a', &control::input_snapshot_t::a},
      {'D', &control::input_snapshot_t::d},
  }};

  constexpr std::uint32_t kHoldTicks = 3;
  demo::TerminalInputMailbox mailbox(kHoldTicks);
  for (const auto& [key, member] : directions) {
    mailbox.HandleKey(key);
    for (std::uint32_t tick = 0; tick < kHoldTicks; ++tick) {
      EXPECT_TRUE(mailbox.Consume().*member) << key << " tick=" << tick;
    }
    EXPECT_FALSE(mailbox.Consume().*member);
  }
}

TEST(TerminalInputMailboxTest, NoNewKeyProducesReleasedSnapshot) {
  demo::TerminalInputMailbox mailbox(1);
  ExpectReleased(mailbox.Consume());
  mailbox.HandleKey('w');
  EXPECT_TRUE(mailbox.Consume().w);
  ExpectReleased(mailbox.Consume());
}

TEST(TerminalInputMailboxTest, OpposingKeysRemainVisibleInSameFrame) {
  demo::TerminalInputMailbox mailbox(1);
  mailbox.HandleKey('w');
  mailbox.HandleKey('s');
  mailbox.HandleKey('a');
  mailbox.HandleKey('d');
  const auto input = mailbox.Consume();
  EXPECT_TRUE(input.w);
  EXPECT_TRUE(input.s);
  EXPECT_TRUE(input.a);
  EXPECT_TRUE(input.d);
  ExpectReleased(mailbox.Consume());
}

TEST(TerminalInputMailboxTest, RepeatedDirectionRefreshesConfiguredLifetime) {
  demo::TerminalInputMailbox mailbox(3);
  mailbox.HandleKey('w');
  EXPECT_TRUE(mailbox.Consume().w);
  EXPECT_TRUE(mailbox.Consume().w);
  mailbox.HandleKey('W');
  for (int tick = 0; tick < 3; ++tick) {
    EXPECT_TRUE(mailbox.Consume().w) << tick;
  }
  ExpectReleased(mailbox.Consume());
}

TEST(TerminalInputMailboxTest, LegTargetIsLastWinsAndSpaceUsesParity) {
  demo::TerminalInputMailbox mailbox;
  mailbox.HandleKey('q');
  mailbox.HandleKey('E');
  auto input = mailbox.Consume();
  EXPECT_FALSE(input.q);
  EXPECT_TRUE(input.e);
  EXPECT_FALSE(input.f);
  ExpectReleased(mailbox.Consume());

  mailbox.HandleKey(' ');
  EXPECT_TRUE(mailbox.Consume().space);
  mailbox.HandleKey(' ');
  mailbox.HandleKey(' ');
  EXPECT_FALSE(mailbox.Consume().space);
}

TEST(TerminalInputMailboxTest, SpaceXEscapeAndUnknownKeysProvideSafeState) {
  demo::TerminalInputMailbox mailbox;
  mailbox.HandleKey('w');
  mailbox.HandleKey('d');
  mailbox.HandleKey(' ');
  auto input = mailbox.Consume();
  EXPECT_FALSE(input.w);
  EXPECT_FALSE(input.d);
  EXPECT_TRUE(input.space);

  mailbox.HandleKey('s');
  mailbox.HandleKey('a');
  mailbox.HandleKey('x');
  ExpectReleased(mailbox.Consume());
  mailbox.HandleKey('r');
  ExpectReleased(mailbox.Consume());

  mailbox.HandleKey('q');
  mailbox.HandleKey('\x1b');
  EXPECT_TRUE(mailbox.ExitRequested());
  ExpectReleased(mailbox.Consume());
}

TEST(TerminalInputMailboxTest, PreservesPendingSpaceWhenFirstTickUsesExplicitRelease) {
  demo::TerminalInputMailbox mailbox;
  mailbox.HandleKey(' ');
  const control::input_snapshot_t first_tick{};
  EXPECT_FALSE(first_tick.space);
  EXPECT_TRUE(mailbox.Consume().space);
}

TEST(TerminalInputMailboxTest, SupportsConcurrentProducerAndConsumerWithoutDuplicateActions) {
  demo::TerminalInputMailbox mailbox(1);
  std::atomic<bool> producer_done = false;
  std::atomic<int> consumed_actions = 0;
  std::jthread producer([&] {
    for (int index = 0; index < 20000; ++index) {
      mailbox.HandleKey('w');
      mailbox.HandleKey('q');
    }
    producer_done = true;
  });
  while (!producer_done) {
    if (mailbox.Consume().q) ++consumed_actions;
  }
  producer.join();
  if (mailbox.Consume().q) ++consumed_actions;
  EXPECT_LE(consumed_actions, 20000);
  ExpectReleased(mailbox.Consume());
}

TEST(TerminalKeyboardInputTest, ReaderConsumesKeysAndStopsWithoutInputTimeout) {
  auto source = std::make_unique<FakeByteSource>();
  FakeByteSource* source_ptr = source.get();
  std::string error;
  auto keyboard = demo::TerminalKeyboardInput::CreateForTesting(std::move(source), 1, &error);
  ASSERT_NE(keyboard, nullptr) << error;
  ASSERT_TRUE(keyboard->Start(&error)) << error;

  source_ptr->Push('w');
  ASSERT_TRUE(WaitUntil([&] { return keyboard->Consume().w; }));
  ExpectReleased(keyboard->Consume());

  const auto start = std::chrono::steady_clock::now();
  keyboard->RequestStop();
  keyboard->Join();
  EXPECT_LT(std::chrono::steady_clock::now() - start, 500ms);
  EXPECT_FALSE(keyboard->TakeError(&error));
}

TEST(TerminalKeyboardInputTest, ReadsPseudoTerminalAndRestoresSettings) {
  const int master_fd = posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC);
  ASSERT_GE(master_fd, 0);
  ASSERT_EQ(grantpt(master_fd), 0);
  ASSERT_EQ(unlockpt(master_fd), 0);
  const char* slave_name = ptsname(master_fd);
  ASSERT_NE(slave_name, nullptr);
  const int slave_fd = open(slave_name, O_RDWR | O_NOCTTY | O_CLOEXEC);
  ASSERT_GE(slave_fd, 0);

  termios original{};
  ASSERT_EQ(tcgetattr(slave_fd, &original), 0);
  {
    ScopedStdinRedirect redirect(slave_fd);
    ASSERT_TRUE(redirect.valid());
    std::string error;
    auto keyboard = demo::TerminalKeyboardInput::OpenStandardInput(1, &error);
    ASSERT_NE(keyboard, nullptr) << error;
    ASSERT_TRUE(keyboard->Start(&error)) << error;

    termios immediate{};
    ASSERT_EQ(tcgetattr(STDIN_FILENO, &immediate), 0);
    EXPECT_EQ(immediate.c_lflag & (ICANON | ECHO), 0U);
    const char key = 'w';
    ASSERT_EQ(write(master_fd, &key, sizeof(key)), 1);
    ASSERT_TRUE(WaitUntil([&] { return keyboard->Consume().w; }));
    ExpectReleased(keyboard->Consume());

    keyboard->RequestStop();
    keyboard->Join();
    termios restored{};
    ASSERT_EQ(tcgetattr(STDIN_FILENO, &restored), 0);
    EXPECT_EQ(restored.c_lflag, original.c_lflag);
    EXPECT_EQ(restored.c_cc[VMIN], original.c_cc[VMIN]);
    EXPECT_EQ(restored.c_cc[VTIME], original.c_cc[VTIME]);
  }
  close(slave_fd);
  close(master_fd);
}

TEST(TerminalKeyboardInputTest, ReportsDisconnectAndInjectedReadError) {
  auto source = std::make_unique<FakeByteSource>();
  FakeByteSource* source_ptr = source.get();
  std::string error;
  auto keyboard = demo::TerminalKeyboardInput::CreateForTesting(std::move(source), 1, &error);
  ASSERT_NE(keyboard, nullptr) << error;
  ASSERT_TRUE(keyboard->Start(&error)) << error;
  source_ptr->Push('d');
  ASSERT_TRUE(WaitUntil([&] { return keyboard->Consume().d; }));
  source_ptr->PushResult(demo::KeyboardReadResult::kDisconnected);
  ASSERT_TRUE(WaitUntil([&] { return keyboard->TakeError(&error); }));
  EXPECT_NE(error.find("disconnected"), std::string::npos);
  ExpectReleased(keyboard->Consume());
  keyboard->RequestStop();
  keyboard->Join();

  auto error_source = std::make_unique<FakeByteSource>();
  FakeByteSource* error_source_ptr = error_source.get();
  keyboard = demo::TerminalKeyboardInput::CreateForTesting(std::move(error_source), 1, &error);
  ASSERT_NE(keyboard, nullptr) << error;
  ASSERT_TRUE(keyboard->Start(&error)) << error;
  error_source_ptr->PushResult(demo::KeyboardReadResult::kError, "injected terminal failure");
  ASSERT_TRUE(WaitUntil([&] { return keyboard->TakeError(&error); }));
  EXPECT_EQ(error, "injected terminal failure");
}

TEST(TerminalKeyboardInputTest, SupportsRepeatedStartStopAndRejectsInvalidSource) {
  std::string error;
  EXPECT_EQ(demo::TerminalKeyboardInput::CreateForTesting(nullptr, 1, &error), nullptr);
  EXPECT_NE(error.find("invalid"), std::string::npos);

  auto keyboard =
      demo::TerminalKeyboardInput::CreateForTesting(std::make_unique<FakeByteSource>(), 1, &error);
  ASSERT_NE(keyboard, nullptr) << error;
  ASSERT_TRUE(keyboard->Start(&error)) << error;
  keyboard->RequestStop();
  keyboard->Join();
  ASSERT_TRUE(keyboard->Start(&error)) << error;
  keyboard->RequestStop();
  keyboard->Join();
}

TEST(ProcessSignalGuardTest, ObservesAndRestoresTerminationSignal) {
  std::string error;
  auto guard = demo::ProcessSignalGuard::Install(&error);
  ASSERT_NE(guard, nullptr) << error;
  EXPECT_FALSE(demo::ProcessSignalGuard::StopRequested());
  ASSERT_EQ(raise(SIGINT), 0);
  EXPECT_TRUE(demo::ProcessSignalGuard::StopRequested());
  EXPECT_TRUE(guard->Restore(&error)) << error;
  EXPECT_TRUE(guard->Restore(&error)) << error;
}

TEST(ProcessSignalGuardTest, RestoresPreviousSigtermHandler) {
  previous_sigterm_called = 0;
  const auto previous_handler = std::signal(SIGTERM, PreviousSigtermHandler);
  ASSERT_NE(previous_handler, SIG_ERR);
  std::string error;
  auto guard = demo::ProcessSignalGuard::Install(&error);
  ASSERT_NE(guard, nullptr) << error;
  ASSERT_EQ(raise(SIGTERM), 0);
  EXPECT_TRUE(demo::ProcessSignalGuard::StopRequested());
  ASSERT_TRUE(guard->Restore(&error)) << error;
  ASSERT_EQ(raise(SIGTERM), 0);
  EXPECT_EQ(previous_sigterm_called, 1);
  EXPECT_NE(std::signal(SIGTERM, previous_handler), SIG_ERR);
}
