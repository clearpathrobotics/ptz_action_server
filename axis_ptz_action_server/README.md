axis_ptz_action_server
===================

This package provides `ptz_action_server_msgs`-compatible actions for the Axis PTZ cameras

Requires `axis_camera` (https://github.com/ros-drivers/axis_camera) to be running to provide base-level
hardware control.


Parameters
-----------

The `axis_ptz_action_server_node` takes the following parameters:
- `cmd_topic` -- the name of the `Axis.msg` topic provided by the `axis_camera` package to move the unit. Default: `/axis/cmd`
- `status_topic` -- the name of the `Axis.msg` topic provided by the `flir_ptu` package to report the camera's state. Default: `/axis/state`
- `act_ns` -- the name of the `Ptz.action` service the node provides. Default: `/ptu_driver/ptz_cmd`


Published Topics
-----------------

The node provides the `move_ptz` action with its corresponding `goal`, `feedback`, `result`, and `cancel` topics
in the namespace defined by `act_ns` as described above.

The node subscribes to the `status_topic` and republishes the PTZ state in `{act_ns}/ptz_state` using the
`ptz_action_server_msgs/PtzPosition` message type.
