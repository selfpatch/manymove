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

#include <gtest/gtest.h>
#include <behaviortree_cpp_v3/blackboard.h>

#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <ros2_medkit_msgs/msg/fault.hpp>

#include "manymove_cpp_trees/fault_codes.hpp"
#include "manymove_cpp_trees/fault_reporting.hpp"
#include "manymove_cpp_trees/main_imports_helper.hpp"

#include "fake_fault_manager.hpp"

using manymove_cpp_trees::FaultReporting;
using manymove_cpp_trees::installFaultReporter;
using manymove_cpp_trees::test::FakeFaultManager;
using namespace std::chrono_literals;

// Minimal subclass that exposes the protected reportFault helpers so tests
// can drive the mixin without spinning up a full BT.CPP factory.
class TestableReportingClass : public FaultReporting
{
public:
  explicit TestableReportingClass(const BT::Blackboard::Ptr & bb)
  : FaultReporting(bb) {}

  using FaultReporting::reportFault;
  using FaultReporting::reportFaultPassed;
};

class FaultReportingFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    fault_manager_node_ = rclcpp::Node::make_shared("fake_fault_manager");
    fake_fm_ = std::make_unique<FakeFaultManager>(fault_manager_node_);

    bt_node_ = rclcpp::Node::make_shared("test_bt_client");
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("node", bt_node_);
    installFaultReporter(blackboard_, bt_node_);

    exec_.add_node(fault_manager_node_);
    exec_.add_node(bt_node_);
  }

  void TearDown() override
  {
    exec_.remove_node(bt_node_);
    exec_.remove_node(fault_manager_node_);
    fake_fm_.reset();
    bt_node_.reset();
    fault_manager_node_.reset();
    blackboard_.reset();
  }

  rclcpp::Node::SharedPtr fault_manager_node_;
  rclcpp::Node::SharedPtr bt_node_;
  std::unique_ptr<FakeFaultManager> fake_fm_;
  BT::Blackboard::Ptr blackboard_;
  rclcpp::executors::SingleThreadedExecutor exec_;
};

TEST_F(FaultReportingFixture, ReporterIsInstalledOnBlackboard)
{
  std::shared_ptr<ros2_medkit_fault_reporter::FaultReporter> reporter;
  ASSERT_TRUE(blackboard_->get(manymove_cpp_trees::kFaultReporterBlackboardKey, reporter));
  ASSERT_NE(reporter, nullptr);
}

TEST_F(FaultReportingFixture, MixinForwardsErrorReportToFakeManager)
{
  TestableReportingClass node(blackboard_);

  node.reportFault(
    manymove_cpp_trees::fault_codes::kPlannerCollisionDetected,
    manymove_cpp_trees::kSeverityError,
    "test collision");

  ASSERT_TRUE(
    fake_fm_->wait_for(
      exec_, manymove_cpp_trees::fault_codes::kPlannerCollisionDetected, 2s));

  const auto received = fake_fm_->received();
  ASSERT_EQ(received.size(), 1u);
  EXPECT_EQ(
    received.front().fault_code,
    manymove_cpp_trees::fault_codes::kPlannerCollisionDetected);
  EXPECT_EQ(received.front().severity, ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR);
  EXPECT_EQ(received.front().description, "test collision");
  EXPECT_EQ(received.front().source_id, std::string(bt_node_->get_fully_qualified_name()));
}

TEST_F(FaultReportingFixture, ReportFaultPassedDeliversPassedEvent)
{
  TestableReportingClass node(blackboard_);

  // Send one FAILED first so the LocalFilter has something to clear.
  node.reportFault(
    manymove_cpp_trees::fault_codes::kPlannerRetryAttempt,
    manymove_cpp_trees::kSeverityError,
    "failure");
  ASSERT_TRUE(
    fake_fm_->wait_for(
      exec_, manymove_cpp_trees::fault_codes::kPlannerRetryAttempt, 2s));
  fake_fm_->clear();

  node.reportFaultPassed(manymove_cpp_trees::fault_codes::kPlannerRetryAttempt);
  ASSERT_TRUE(
    fake_fm_->wait_for(
      exec_, manymove_cpp_trees::fault_codes::kPlannerRetryAttempt, 2s));

  const auto received = fake_fm_->received();
  ASSERT_EQ(received.size(), 1u);
  EXPECT_EQ(received.front().event_type, ros2_medkit_msgs::srv::ReportFault::Request::EVENT_PASSED);
}

TEST_F(FaultReportingFixture, MixinIsNoOpWhenReporterAbsentFromBlackboard)
{
  // Build a blackboard WITHOUT installFaultReporter -> reporter_ should be null
  // and the mixin must silently no-op rather than crash.
  auto empty_bb = BT::Blackboard::create();
  TestableReportingClass node(empty_bb);

  ASSERT_NO_THROW(
    node.reportFault(
      manymove_cpp_trees::fault_codes::kRobotNotReady,
      manymove_cpp_trees::kSeverityCritical, "no reporter"));
  ASSERT_NO_THROW(
    node.reportFaultPassed(manymove_cpp_trees::fault_codes::kRobotNotReady));

  // Nothing should reach the fake manager because the report was never sent.
  exec_.spin_some();
  std::this_thread::sleep_for(200ms);
  exec_.spin_some();
  EXPECT_EQ(fake_fm_->received().size(), 0u);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
