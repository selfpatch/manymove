// Copyright 2026 Selfpatch.ai
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Selfpatch.ai nor the names of its contributors
//      may be used to endorse or promote products derived from this software
//      without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef FAKE_FAULT_MANAGER_HPP_
#define FAKE_FAULT_MANAGER_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ros2_medkit_msgs/srv/report_fault.hpp>

namespace manymove_cpp_trees::test
{

// Test fixture that stands in for the medkit FaultManager during gtest runs.
// It hosts the /fault_manager/report_fault service and records every accepted
// request so individual tests can assert which faults a BT node emitted.
//
// Mirrors the existing FakeXxxServer pattern in this test directory so tests
// stay readable for anyone already familiar with the codebase.
class FakeFaultManager
{
public:
  using ReportFault = ros2_medkit_msgs::srv::ReportFault;

  explicit FakeFaultManager(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    service_ = node_->create_service<ReportFault>(
      "/fault_manager/report_fault",
      [this](
        const std::shared_ptr<ReportFault::Request> request,
        std::shared_ptr<ReportFault::Response> response) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          received_.push_back(*request);
        }
        response->accepted = true;
      });
  }

  std::vector<ReportFault::Request> received() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return received_;
  }

  size_t count_of(const std::string & fault_code) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t n = 0;
    for (const auto & r : received_) {
      if (r.fault_code == fault_code) {
        ++n;
      }
    }
    return n;
  }

  // Spin-poll util: returns true once any received entry matches `fault_code`.
  bool wait_for(
    rclcpp::executors::SingleThreadedExecutor & exec, const std::string & fault_code,
    std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (count_of(fault_code) > 0) {
        return true;
      }
      exec.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    received_.clear();
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<ReportFault>::SharedPtr service_;
  mutable std::mutex mutex_;
  std::vector<ReportFault::Request> received_;
};

}  // namespace manymove_cpp_trees::test

#endif  // FAKE_FAULT_MANAGER_HPP_
