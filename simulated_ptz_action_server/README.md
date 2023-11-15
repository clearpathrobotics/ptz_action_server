simulated_ptz_action_server
============================

This package contains a simple ROS node to simulate the action server interface for a PTZ camera.

The node interacts with PTZ controllers to move the camera inside Gazebo, and implements a basic digital zoom
with OpenCV to allow zooming.  (Gazebo cameras have fixed fields of view and cannot normally zoom at all)

Usage
------

At its most basic, this node can be used to control an already-configured URDF by simply running

```bash
roslaunch simulated_ptz_action_server simulated_ptz_action_server.launch
```

The digitally-zoomed camera data will be available on `/sensor/camera_0/image_raw` and the PTZ actions are
available in `/sensors/camera_0/move_ptz`.

The node provides two `actionlib` servers:
- `move_ptz/position_abs`: sends an absolute pan/tilt/zoom destination for the camera to move to
- `move_ptz/position_rel`: sends a desired pan/tilt/zoom destination relative to the camera's current position

To control the camera, simply connect your `actionlib` client to the appropriate absolute or relative action server
above and send goals.


URDF Configuration & PID Controllers
-------------------------------------

To use the node you must configure your URDF to include a compatible PTZ camera.  At minimum you must define revolute
or continuous joints for the pan & tilt actuators, and include a Gazebo camera plugin to provide the image data.

Below is an example of how this could be configured. Note that for Gazebo to recognize the camera joints both
links must have physical properties.

```xml
<!--
  The main body of the camera, mounted to the robot.
-->
<link name="ptz_camera_base_link">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <mass value="1" />
    <inertia
      ixx="1.0" ixy="0.0" ixz="0.0"
      iyy="1.0" iyz="0.0"
      izz="1.0" />
  </inertial>
  <visual>
    <geometry>
      <mesh filename="package://my_camera_description/meshes/body.dae" scale="0.001 0.001 0.001" />
    </geometry>
    <material name="black" />
    <origin xyz="0 0 0" rpy="0 0 ${pi/2}" />
  </visual>
  <collision>
    <geometry>
      <mesh filename="package://my_camera_description/meshes/body.stl" scale="0.001 0.001 0.001" />
    </geometry>
    <origin xyz="0 0 0" rpy="0 0 ${pi/2}" />
  </collision>
</link>
<joint name="ptz_camera_base_joint" type="fixed">
  <parent link="base_link" />
  <child link="ptz_camera_base_link" />
  <origin xyz="0 0 0.2" rpy="0 0 0" />
</joint>

<!--
  The pan joint. This can be continuous or revolute with angle limits
-->
<link name="ptz_camera_pan_link">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <mass value="1" />
    <inertia
      ixx="1.0" ixy="0.0" ixz="0.0"
      iyy="1.0" iyz="0.0"
      izz="1.0" />
  </inertial>
</link>
<joint name="ptz_camera_pan_joint" type="continuous">
  <axis xyz="0 0 1" />
  <parent link="ptz_camera_base_link" />
  <child link="ptz_camera_pan_link" />
  <origin xyz="0 0 0.15" rpy="0 0 0" />
</joint>
<transmission name="ptz_camera_pan_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="ptz_camera_pan_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="ptz_camera_pan_actuator">
      <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>

<!--
  The camera's tilting mechanism
  Typically this joint will be revolute with angle limits, but it could be continuous
  if your camera allows full 360-degree tilting
-->
<link name="ptz_camera_tilt_link">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <mass value="1" />
    <inertia
      ixx="1.0" ixy="0.0" ixz="0.0"
      iyy="1.0" iyz="0.0"
      izz="1.0" />
  </inertial>
  <visual>
    <geometry>
      <mesh filename="package://my_camera_description/meshes/head.dae" scale="0.001 0.001 0.001" />
    </geometry>
    <material name="black" />
    <origin xyz="0 0 0" rpy="0 0 ${pi/2}" />
  </visual>
  <collision>
    <geometry>
      <mesh filename="package://my_camera_description/meshes/head.stl" scale="0.001 0.001 0.001" />
    </geometry>
    <origin xyz="0 0 0" rpy="0 0 ${pi/2}" />
  </collision>
</link>
<joint name="ptz_camera_tilt_joint" type="revolute">
  <axis xyz="0 1 0" />
  <limit lower="-1.5707963267948966" upper="0.5235987755982988" effort="1" velocity="1" />
  <parent link="ptz_camera_pan_link" />
  <child link="ptz_camera_tilt_link" />
  <origin xyz="0 0 0" rpy="${pi} 0 0" />
</joint>
<transmission name="ptz_camera_tilt_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="ptz_camera_tilt_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="ptz_camera_tilt_actuator">
      <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>

<!--
  The camera lens link
  This is the frame that the camera plugin uses to determine the camera's FoV
-->
<link name="ptz_camera_link" />
<joint name="ptz_camera_joint" type="fixed">
  <parent link="ptz_camera_tilt_link" />
  <child link="ptz_camera_link" />
  <origin xyz="0.07 0 0" rpy="-${pi} 0 0" />
</joint>
<gazebo reference="ptz_camera_link">
  <sensor type="camera" name="ptz_camera">
    <update_rate>15</update_rate>
    <camera>
      <horizontal_fov>1.5184351666666667</horizontal_fov>
      <vertical_fov>1.0122901111111111</vertical_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>500.0</far>
      </clip>
    </camera>
    <plugin name="ptz_camera_controller" filename="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <cameraName>/sensors/camera_0</cameraName>
      <!--
        We use the image_raw_nozoom topic because the PTZ action server node will publish image_raw
        with a digital zoom applied
      -->
      <imageTopicName>image_raw_nozoom</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>ptz_camera_link</frameName>
    </plugin>
  </sensor>
</gazebo>
```

You must launch the PID controllers to control the pan & tilt joints as well:

```xml
<launch>
  <rosparam command="load" file="$(find simulated_ptz_action_server)/config/ptz_controllers.yaml" />
  <node name="ptz_action_server_spawner" pkg="controller_manager" type="spawner"
        args="pan_position_controller tilt_position_controller"/>
</launch>
```

where the `ptz_controllers.yaml` file contains the following:

```yaml
pan_position_controller:
  type: "velocity_controllers/JointPositionController"
  joint: "ptz_camera_pan_joint"
  pid:
    p: 1.0
    i: 0.01
    d: 0.1

tilt_position_controller:
  type: "velocity_controllers/JointPositionController"
  joint: "ptz_camera_tilt_joint"
  pid:
    p: 5.0
    i: 0.01
    d: 1.0
```

If you have multiple PTZ cameras, each one must have its own controllers
