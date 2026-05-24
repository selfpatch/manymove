# manymove fault codes

This is the catalogue of fault codes emitted by `manymove_cpp_trees` BT action
nodes via [`ros2_medkit_fault_reporter`](https://github.com/selfpatch/ros2_medkit)
when built with `MANYMOVE_WITH_MEDKIT=ON` (see the package README for the
build instructions). With the option OFF the same `reportFault()` call sites
compile to no-ops and nothing is published.
Each entry below maps to one or more `reportFault()` call sites in the action
node sources; the source-of-truth string constants live in
`manymove_cpp_trees/include/manymove_cpp_trees/fault_codes.hpp`.

## Conventions

- Format: `MANYMOVE_<SUBSYSTEM>_<CONDITION>`
- Character set: `[A-Z0-9_]`, max 64 characters (medkit `ReportFault.srv` limit)
- Severity uses `ros2_medkit_msgs::msg::Fault::SEVERITY_*`:
  - `INFO` (0), `WARN` (1), `ERROR` (2), `CRITICAL` (3)
- Configuration / programmer-error FAILUREs (missing input ports, bad
  blackboard keys) are intentionally NOT instrumented; only operational
  faults are reported.
- `BT::ConditionNode` and `BT::DecoratorNode` returns of FAILURE are normal
  control flow and never produce fault reports.

## Planner (`action_nodes_planner.cpp`)

| Code | Severity | Trigger | Pair |
|------|----------|---------|------|
| `MANYMOVE_PLANNER_COLLISION_DETECTED` | ERROR | `MoveManipulatorAction::onStart` sees `collision_detected=true` before sending the goal. | - |
| `MANYMOVE_PLANNER_ESTOP_TRIGGERED` | CRITICAL | `MoveManipulatorAction::onStart` sees `stop_execution=true`. | - |
| `MANYMOVE_PLANNER_RETRY_ATTEMPT` | WARN | One attempt of `MoveManipulatorAction` failed; the move may still succeed on retry. Throttled locally by `LocalFilter`. | `reportFaultPassed` in the success branch |
| `MANYMOVE_PLANNER_RETRIES_EXHAUSTED` | ERROR | `current_try_ >= max_tries_`; motion aborted. | - |

## Object manager (`action_nodes_objects.cpp`)

| Code | Severity | Trigger | Pair |
|------|----------|---------|------|
| `MANYMOVE_OBJECT_ADD_FAILED` | ERROR | `AddCollisionObjectAction` action result `success=false`. | - |
| `MANYMOVE_OBJECT_REMOVE_FAILED` | ERROR | `RemoveCollisionObjectAction` action result `success=false`. | - |
| `MANYMOVE_OBJECT_ATTACH_FAILED` | ERROR | `AttachDetachObjectAction` action result `success=false`. | - |
| `MANYMOVE_OBJECT_GET_POSE_FAILED` | ERROR | `GetObjectPoseAction` action result `success=false`. | - |
| `MANYMOVE_OBJECT_WAIT_TIMEOUT` | WARN | `WaitForObjectAction` elapsed without observing the expected presence/absence. | `reportFaultPassed` when the wait condition is met or on `onHalted` (subtree abandoned the wait). |

`CheckObjectExistsAction` returning FAILURE for "object missing" is intentional
control flow used by callers as a condition; it is not instrumented.

## Signals (`action_nodes_signals.cpp`)

| Code | Severity | Trigger | Pair |
|------|----------|---------|------|
| `MANYMOVE_SIGNAL_SET_OUTPUT_FAILED` | ERROR | `SetOutputAction` action result reports failure. | - |
| `MANYMOVE_SIGNAL_GET_INPUT_FAILED` | ERROR | `GetInputAction` action result reports failure, or `WaitForInputAction` cannot connect to the action server within 5s. | - |
| `MANYMOVE_SIGNAL_WAIT_INPUT_TIMEOUT` | WARN | `WaitForInputAction` timeout elapsed without observing the desired value. | `reportFaultPassed` on success or on `onHalted` (subtree abandoned the wait). |
| `MANYMOVE_ROBOT_NOT_READY` | CRITICAL | `CheckRobotStateAction` reports `ready=false`. | `reportFaultPassed` once the robot reports `ready=true`. |
| `MANYMOVE_ROBOT_RESET_FAILED` | ERROR | `ResetRobotStateAction` fails any of its three steps (unload trajectory controller, reset robot state, load trajectory controller); raised on goal rejection or non-SUCCEEDED result. | - |

## Logic / TF (`action_nodes_logic.cpp`)

| Code | Severity | Trigger | Pair |
|------|----------|---------|------|
| `MANYMOVE_TF_LOOKUP_FAILED` | WARN | `GetLinkPoseAction` tf2 `lookupTransform` raised `tf2::TransformException`. | - |
| `MANYMOVE_WAIT_KEY_TIMEOUT` | WARN | `WaitForKeyBool` elapsed without the expected blackboard value. | `reportFaultPassed` once the key matches. |

## Gripper (`action_nodes_gripper.cpp`)

| Code | Severity | Trigger | Pair |
|------|----------|---------|------|
| `MANYMOVE_GRIPPER_COMMAND_FAILED` | ERROR | `GripperCommandAction` action server unavailable, or goal reported `reached_goal=false` and `stalled=false`. | - |
| `MANYMOVE_GRIPPER_TRAJ_FAILED` | ERROR | `GripperTrajAction` action server unavailable or trajectory result code != `SUCCEEDED`. | - |

## Isaac Sim (`action_nodes_isaac.cpp`)

| Code | Severity | Trigger | Pair |
|------|----------|---------|------|
| `MANYMOVE_ISAAC_FOUNDATION_POSE_FAILED` | ERROR | `FoundationPoseAlignmentNode` timed out waiting for detections, no detection passed the `target_id`/`min_score` filter within the configured timeout, or TF transform of the detection pose to the alignment / planning frame timed out. | - |

`GetEntityPoseNode`/`SetEntityPoseNode` service errors and Isaac TF timeouts
are not yet instrumented; they will gain dedicated codes when the
`manymove_industrial` demo wires them up.

## Local filtering

`ros2_medkit_fault_reporter::LocalFilter` debounces repeated reports of the
same `fault_code` so soft-fault burst patterns (e.g. several
`MANYMOVE_PLANNER_RETRY_ATTEMPT` in quick succession) do not flood the
`FaultManager`. Defaults: `default_threshold=3`, `default_window_sec=10.0`,
`bypass_severity=ERROR`. Configure per-bt_client via standard ROS parameters:

```yaml
bt_client_xarm7:
  ros__parameters:
    fault_reporter:
      local_filtering:
        enabled: true
        default_threshold: 3
        default_window_sec: 10.0
        bypass_severity: 2
```

Pair `reportFaultPassed` with the matching `reportFault` on the success branch
of any throttled code so the filter resets cleanly between cycles.
