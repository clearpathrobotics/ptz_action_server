PTZ Action Server
===================

This repo contains both generic action messages for controlling PTZ and PTU devices, as well as
middle-ware ROS packages designed to wrap hardware/vendor-specific drivers and provide a more universal
interface


ptz_action_server_msgs
-------------------

This package contains the core ActionLib compatible messages that compatible ROS nodes should use.  To allow easy
inter-operability, the action server should be implemented to support these conditions, regardless of the underlying
hardware:

- pan and tilt are expressed in radians
- pan has a maximum allowable range of -pi to +pi, though cameras with a restricted field of motion may support
  a subset of that range
- tilt has a maximum allowable range of -pi/2 to +pi/2, though cameras with a restricted field of motion may suport
  a subset of that range
- positive pan indicates counter-clockwise rotation, negative pan indicates clockwise rotation when the camera is viewed
  from above
- positive tilt will pitch the camera upwards, negative tilt will pitch the camera downwards
- zoom is the X-factor of the camera's zoom range. For example, a camera with a 1-24x zoom shall accept zoom values
  in the range of 1-24.
- if the camera does not support zoom (e.g. the Flir D46 PTU), the zoom field is ignored
- while the action is ongoing feedback shall be published at no less than 1Hz, underlying hardware permitting
- preemption should be allowed if possible, but may not be supported by the underlying hardware.
- the action server will publish the state of the PTZ camera in the above logical angles/ranges at a rate of no less
  than 1Hz.

The main action server uses the following message format:
```
# Ptz.action
float32 pan
float32 tilt
float32 zoom
---
bool success
---
float32 pan_remaining
float32 tilt_remaining
float32 zoom_remaining
```

The current state of the camera is reported using the following message format:
```
# PtzPosition.msg
float32 pan
float32 tilt
float32 zoom

```


Supported Implementations
--------------------------

This repository contains action server implementations that communicate with underlying ROS hardware drivers:
- axiz_ptz_action_server: interacts with the axis_camera driver, supporting most Axis PTZ cameras that use ethernet
- pelcod_ptz_action_server: interacts with any camera that uses the pelco_d_controls package, including the Infinity Optics
  Proton PTZ camera
- flir_ptu_action_server: interacts with the Flir D46 and E46 PTUs, supported by the flir_ptu package, either over serial
  or ethernet.
