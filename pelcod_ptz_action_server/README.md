pelcod_ptz_action_server
======================

Pelco-D compatible implementation of the `ptz_action_server_msgs/Ptz.action` for the Proton PTZ camera and other compatible
devices.

Requires the underlying `pelco_d_controls` driver to be running.
See https://gitlab.clearpathrobotics.com/research/pelco_d_controls


Parameters
-----------

The `pelcod_ptz_action_server` takes the following parameters:
- `cmd_srv` -- the name of the service the Pelco-D driver provides for commanding the device. Default: `/pelcod/ptz_server`
- `position_topic` -- the name of the topic the Pelco-D driver publishes `JointState` messages on. Default: `/pelcod/ptz_joint_state`
- `zoom_topic` -- the name of the topic the Pelco-D driver publishes the zoom level on. Default: `/pelcod/ptz_zoom_state`
- `act_ns` -- namespace for the `Ptz.action` server this node provides. Default: `/pelcod`
- `pan_joint` -- the name of the pan joint as published in `/joint_states`. Default: `pan`
- `tilt_joint` -- the name of the tilt joint as published in `/joint_states`. Default: `tilt`

PTZ/PTU limits are defined by these parameters:
- `min_zoom` -- the minimum zoom level of the camera. Should be `0` if zoom is not supported
- `max_zoom` -- the maximum zoom level of the camera. Should be `0` if zoom is not supported
- `min_pan` -- the minimum pan angle of the camera in radians. Default: `-2.356194490192345` (-135 degrees)
- `max_pan` -- the maximum pan angle of the camera in radians. Default: `2.356194490192345` (+135 degrees)
- `min_tilt` -- the minimum tilt angle of the camera in radians. Default: `-0.7853981633974483` (-45 degrees)
- `max_tilt` -- the maximum tilt angle of the camera in radians. Default: `1.5707963267948966` (+90 degrees)


Subscriptions
--------------

- `{position_topic}` -- the current state of the robot's joints, including the ones named by the `pan_joint` and
  `tilt_joint` parameters described above
- `{zoom_topic}` -- the current zoom level of the camera. If the PTU does not support zoom, this subscription can be
  left disconnected without error.


Published Topics
-----------------

The node provides the `move_ptz` action with its corresponding `goal`, `feedback`, `result`, and `cancel` topics
in the namespace defined by `act_ns` as described above.

The current position of the PTU is published on `{act_ns}/ptz_state` using the `ptz_action_server_msgs/PtzPosition` message type.
