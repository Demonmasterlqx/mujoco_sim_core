#pragma once

#include <cassert>
#include <cstdint>

namespace mujoco_sim::sensor_demo {

enum class MapperScanOutcome {
  kAccepted,
  kRejected,
};

// A processed scan is one successfully decoded LiDAR sample. Each processed
// scan must receive exactly one accepted or rejected outcome.
class MapperScanAccounting {
 public:
  explicit MapperScanAccounting(std::uint64_t max_scans)
      : max_scans_(max_scans) {}

  [[nodiscard]] bool CanStartScan() const noexcept {
    return !scan_pending_ &&
           (max_scans_ == 0 || processed_scans_ < max_scans_);
  }

  std::uint64_t BeginDecodedScan() noexcept {
    assert(CanStartScan());
    scan_pending_ = true;
    return ++processed_scans_;
  }

  void FinishScan(MapperScanOutcome outcome) noexcept {
    assert(scan_pending_);
    if (outcome == MapperScanOutcome::kAccepted) {
      ++accepted_scans_;
    } else {
      ++rejected_scans_;
    }
    scan_pending_ = false;
  }

  [[nodiscard]] std::uint64_t processed_scans() const noexcept {
    return processed_scans_;
  }
  [[nodiscard]] std::uint64_t accepted_scans() const noexcept {
    return accepted_scans_;
  }
  [[nodiscard]] std::uint64_t rejected_scans() const noexcept {
    return rejected_scans_;
  }

 private:
  std::uint64_t max_scans_ = 0;
  std::uint64_t processed_scans_ = 0;
  std::uint64_t accepted_scans_ = 0;
  std::uint64_t rejected_scans_ = 0;
  bool scan_pending_ = false;
};

class MapperScanScope {
 public:
  explicit MapperScanScope(MapperScanAccounting& accounting) noexcept
      : accounting_(&accounting), scan_number_(accounting.BeginDecodedScan()) {}

  ~MapperScanScope() {
    if (accounting_ != nullptr) {
      accounting_->FinishScan(MapperScanOutcome::kRejected);
    }
  }

  MapperScanScope(const MapperScanScope&) = delete;
  MapperScanScope& operator=(const MapperScanScope&) = delete;
  MapperScanScope(MapperScanScope&&) = delete;
  MapperScanScope& operator=(MapperScanScope&&) = delete;

  [[nodiscard]] std::uint64_t scan_number() const noexcept {
    return scan_number_;
  }

  void Finish(MapperScanOutcome outcome) noexcept {
    assert(accounting_ != nullptr);
    accounting_->FinishScan(outcome);
    accounting_ = nullptr;
  }

 private:
  MapperScanAccounting* accounting_;
  std::uint64_t scan_number_;
};

}  // namespace mujoco_sim::sensor_demo
