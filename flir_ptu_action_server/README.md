flir_ptu_action_server
======================

Flir D46 and E46 compatible implementation of the `ptz_action_server_msgs/Ptz.action`

Requires the underlying `flir_ptu` driver to be running.  See https://github.com/ros-drivers/flir_ptu

Because the D46 and E46 is a simple pan/tilt motor setup the `zoom` parameter is ignored.


Parameters
-----------

The `flir_ptu_action_server_node` takes the following parameters:
- `cmd_topic` -- the name of the `JointState` topic provided by the `flir_ptu` package to move the unit. Default: `/ptu_driver/cmd`
- `act_ns` -- the namespace of the `Ptz.action` actions the node provides. Default: `/ptu_driver/ptz_cmd`
- `pan_joint` -- the name of the pan joint as published in `/joint_states`. Default: `ptu_pan`
- `tilt_joint` -- the name of the tilt joint as published in `/joint_states`. Default: `ptu_tilt`

Subscriptions
--------------

- `/joint_states` -- the current state of the robot's joints

Published Topics
-----------------

The node provides the `move_ptz` action with its corresponding `goal`, `feedback`, `result`, and `cancel` topics
in the namespace defined by `act_ns` as described above.

The current position of the PTU is published on `{act_ns}/ptz_state` using the `ptz_action_server_msgs/PtzPosition` message type.
