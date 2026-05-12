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

#ifndef MANYMOVE_CPP_TREES__FAULT_CODES_HPP_
#define MANYMOVE_CPP_TREES__FAULT_CODES_HPP_

// Single source of truth for fault codes emitted by manymove_cpp_trees BT
// nodes via ros2_medkit_fault_reporter. Convention:
//   MANYMOVE_<SUBSYSTEM>_<CONDITION>
// Allowed character set (per ros2_medkit_msgs ReportFault.srv): [A-Z0-9_].
// Maximum length: 64 characters.
//
// Each code is reported by exactly the failure path documented in the comment
// above it. See docs/FAULT_CODES.md for severity, recovery semantics, and
// trigger conditions.

namespace manymove_cpp_trees::fault_codes
{

// -- Planner ----------------------------------------------------------------
// Collision detected when entering a move (action_nodes_planner.cpp onStart).
inline constexpr char kPlannerCollisionDetected[] = "MANYMOVE_PLANNER_COLLISION_DETECTED";
// One retry of a move sequence failed; the move may still succeed.
inline constexpr char kPlannerRetryAttempt[] = "MANYMOVE_PLANNER_RETRY_ATTEMPT";
// All retries exhausted, motion aborted.
inline constexpr char kPlannerRetriesExhausted[] = "MANYMOVE_PLANNER_RETRIES_EXHAUSTED";
// External e-stop / stop_execution flag triggered while motion was running.
inline constexpr char kPlannerEstopTriggered[] = "MANYMOVE_PLANNER_ESTOP_TRIGGERED";

// -- Object manager ---------------------------------------------------------
// AddCollisionObject action returned failure (planning scene reject).
inline constexpr char kObjectAddFailed[] = "MANYMOVE_OBJECT_ADD_FAILED";
// RemoveCollisionObject action returned failure.
inline constexpr char kObjectRemoveFailed[] = "MANYMOVE_OBJECT_REMOVE_FAILED";
// AttachDetachObject action returned failure (link not found, attach reject).
inline constexpr char kObjectAttachFailed[] = "MANYMOVE_OBJECT_ATTACH_FAILED";
// GetObjectPose action returned failure or the object is not in the scene.
inline constexpr char kObjectGetPoseFailed[] = "MANYMOVE_OBJECT_GET_POSE_FAILED";
// WaitForObject elapsed without the expected object presence/absence.
inline constexpr char kObjectWaitTimeout[] = "MANYMOVE_OBJECT_WAIT_TIMEOUT";

// -- Signals (gripper, IO, robot state) -------------------------------------
// SetOutputAction returned a failed result from the action server.
inline constexpr char kSignalSetOutputFailed[] = "MANYMOVE_SIGNAL_SET_OUTPUT_FAILED";
// GetInputAction returned a failed result from the action server.
inline constexpr char kSignalGetInputFailed[] = "MANYMOVE_SIGNAL_GET_INPUT_FAILED";
// WaitForInputAction elapsed without observing the desired input value.
inline constexpr char kSignalWaitInputTimeout[] = "MANYMOVE_SIGNAL_WAIT_INPUT_TIMEOUT";
// CheckRobotStateAction reported the robot is not ready (err / mode mismatch).
inline constexpr char kRobotNotReady[] = "MANYMOVE_ROBOT_NOT_READY";
// ResetRobotStateAction failed (controller unload / state reset / load step).
inline constexpr char kRobotResetFailed[] = "MANYMOVE_ROBOT_RESET_FAILED";

// -- Logic / TF (action-side, not condition checks) -------------------------
// GetLinkPoseAction tf2 lookup raised TransformException.
inline constexpr char kTfLookupFailed[] = "MANYMOVE_TF_LOOKUP_FAILED";
// WaitForKeyBool elapsed without observing the expected blackboard value.
inline constexpr char kWaitKeyTimeout[] = "MANYMOVE_WAIT_KEY_TIMEOUT";

// -- Gripper ----------------------------------------------------------------
// GripperCommandAction goal aborted, canceled, or rejected.
inline constexpr char kGripperCommandFailed[] = "MANYMOVE_GRIPPER_COMMAND_FAILED";
// GripperTrajAction trajectory execution failed.
inline constexpr char kGripperTrajFailed[] = "MANYMOVE_GRIPPER_TRAJ_FAILED";

// -- Isaac Sim integration --------------------------------------------------
// FoundationPoseAlignmentNode could not lock onto a valid pose in time.
inline constexpr char kIsaacFoundationPoseFailed[] = "MANYMOVE_ISAAC_FOUNDATION_POSE_FAILED";

}  // namespace manymove_cpp_trees::fault_codes

#endif  // MANYMOVE_CPP_TREES__FAULT_CODES_HPP_
