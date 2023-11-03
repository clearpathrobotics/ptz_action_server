axis_ptz_action_server
===================

This package provides `ptz_action_server_msgs`-compatible actions for the Axis PTZ cameras

Requires `axis_camera` (https://github.com/ros-drivers/axis_camera) to be running to provide base-level
hardware control.


Parameters
-----------

The `axis_ptz_action_server_node` takes the following parameters:
- `cmd_ns` -- the namespace for the `axis_camera` node's commands. Default: `/axis/cmd`
- `status_topic` -- the topic for the `axis_camera` node's position status topic. Default: `/axis/state/position`
- `act_ns` -- the namespace to publish the PTZ actions in. Default: `/axis`.
- `publish_joint_states`: if true, the action server will publish the pan & tilt angles to `/joint_states`. Default: `true`
- `pan_joint`: the name of the pan joint in the URDF. Default: `$(camera_name)_pan_joint`
- `tilt_joint`: the name of the tilt joint in the URDF. Default: `$(camera_name)_tilt_joint`


Usage
------

The `axis_ptz_action_server_node` publishes 3 actionlib servers:
- `$(act_ns)/move_ptz/position_abs`: send an absolute pan/tilt/zoom position to the camera
- `$(act_ns)/move_ptz/position_rel`: send a pan/tilt/zoom position relative to the camera's current position
- `$(act_ns)/move_ptz/velocity`: send velocity control commands to the camera

Pan and tilt values are expressed in radians (or rad/s for velocity control).  Positive pan is to the left relative
to the camera's base link (anticlockwise), and positive tilt is up.

Position-based zoom levels are in the range `[1, X]` where X is the magnification level of the camera. e.g. a camera with
a 5x zoom will accept values in the range `[1, 5]` and a 24x zoom will accept values in the range `[1, 24]`.

Velocity-based zoom levels are in the range `[-100, 100]` where the value is the percentage of the camera's maximum
zoom speed. i.e. `100` is equivalent to zooming in at `100%` of maximum speed and `-25` is equivalent to zooming out
at `25%` of maximum speed.

These limits can be reconfigured. See `config/limits_dome.yaml` and `config/limits_q62` for examples.


Published Topics
-----------------

The node provides the `move_ptz` action with its corresponding `goal`, `feedback`, `result`, and `cancel` topics
in the namespace defined by `act_ns` as described above.

The node subscribes to the `status_topic` and republishes the PTZ state in `{act_ns}/ptz_state` using the
`ptz_action_server_msgs/PtzPosition` message type.
