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

#ifndef MANYMOVE_CPP_TREES__FAULT_REPORTING_HPP_
#define MANYMOVE_CPP_TREES__FAULT_REPORTING_HPP_

#include <behaviortree_cpp_v3/blackboard.h>

#include <memory>
#include <string>

#include <ros2_medkit_fault_reporter/fault_reporter.hpp>
#include <ros2_medkit_msgs/msg/fault.hpp>

namespace manymove_cpp_trees
{

// Blackboard key under which bt_client_*.cpp installs the shared FaultReporter
// for every BT node in the tree to consume. See installFaultReporter() in
// main_imports_helper.hpp.
inline constexpr char kFaultReporterBlackboardKey[] = "fault_reporter";

// Severity aliases that mirror ros2_medkit_msgs::msg::Fault::SEVERITY_*. Kept
// here as a header-local convenience so call sites do not have to spell out
// the verbose qualified constants on every reportFault() invocation.
inline constexpr uint8_t kSeverityInfo = ros2_medkit_msgs::msg::Fault::SEVERITY_INFO;
inline constexpr uint8_t kSeverityWarn = ros2_medkit_msgs::msg::Fault::SEVERITY_WARN;
inline constexpr uint8_t kSeverityError = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
inline constexpr uint8_t kSeverityCritical = ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL;

// Capability class that gives a BT action node one-line access to the
// process-wide FaultReporter installed on the blackboard. Inherited as a
// second base alongside BT::SyncActionNode / BT::StatefulActionNode by every
// BT node that reports faults; see fault_codes.hpp for the catalogue and
// docs/FAULT_CODES.md for which sites use which codes.
//
// Condition / decorator nodes (BT::ConditionNode, BT::DecoratorNode) MUST NOT
// inherit this: their FAILURE return is normal control flow, not a fault,
// and instrumenting them would flood the FaultManager.
//
// Lifetime: keeps a shared_ptr to the reporter so it is safe to use from any
// callback running on the same node executor. The reporter itself is owned
// by the bt_client_*.cpp main(); destruction order is fine because the
// blackboard outlives every BT node that pulled the pointer in its ctor.
class FaultReporting
{
public:
  // No virtual functions intentionally: this is a non-polymorphic capability
  // class and we do not want any cost from a vtable in BT node base classes.
  FaultReporting() = default;

protected:
  explicit FaultReporting(const BT::Blackboard::Ptr & blackboard)
  {
    if (blackboard) {
      blackboard->get(kFaultReporterBlackboardKey, reporter_);
    }
  }

  // Report a FAILED event for the given fault_code. No-op when the reporter
  // is not installed on the blackboard (e.g. unit tests that build a tree
  // without bt_client_*.cpp scaffolding).
  void reportFault(const char * fault_code, uint8_t severity, const std::string & description) const
  {
    if (reporter_) {
      reporter_->report(fault_code, severity, description);
    }
  }

  // Report a PASSED event for healing/threshold reset. Pair with reportFault()
  // when a previously reported transient condition (e.g. retry attempt) has
  // cleared in the success branch of the same BT node.
  void reportFaultPassed(const char * fault_code) const
  {
    if (reporter_) {
      reporter_->report_passed(fault_code);
    }
  }

  std::shared_ptr<ros2_medkit_fault_reporter::FaultReporter> reporter_;
};

}  // namespace manymove_cpp_trees

#endif  // MANYMOVE_CPP_TREES__FAULT_REPORTING_HPP_
