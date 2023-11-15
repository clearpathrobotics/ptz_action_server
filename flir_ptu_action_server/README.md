flir_ptu_action_server
======================

Flir D46 and E46 compatible implementation of the `ptz_action_server_msgs/Ptz.action`

Requires the underlying `flir_ptu` driver to be running.  See https://github.com/ros-drivers/flir_ptu

Because the D46 and E46 is a simple pan/tilt motor setup the `zoom` parameter is ignored.


Parameters
-----------

The `flir_ptu_action_server_node` takes the following parameters:
- `cmd_topic` -- the name of the `JointState` topic provided by the `flir_ptu` package to move the unit. Default: `/ptu/cmd`
- `act_ns` -- the namespace of the `Ptz.action` actions the node provides. Default: `/ptu`
- `pan_joint` -- the name of the pan joint as published in `/joint_states`. Default: `ptu_pan`
- `tilt_joint` -- the name of the tilt joint as published in `/joint_states`. Default: `ptu_tilt`

Subscriptions
--------------

- `/joint_states` -- the current state of the robot's joints

Published Topics
-----------------

The node provides two actions within the `act_ns` namespace:
- `move_ptz/position_abs`: send an absolute pan/tilt position (zoom is ignored)
- `move_ptz/position_rel`: send a goal position relative to the current position (zoom is ignored)

The current position of the PTU is published on `{act_ns}/ptz_state` using the `ptz_action_server_msgs/PtzPosition` message type.
