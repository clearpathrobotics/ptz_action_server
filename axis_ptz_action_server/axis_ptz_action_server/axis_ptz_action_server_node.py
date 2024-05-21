#!/usr/bin/env python3

# Copyright 2024 Clearpath Robotics Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Clearpath Robotics Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import math
from math import radians as deg2rad
from threading import Lock, Thread
import uuid

from action_msgs.msg import GoalStatus
from axis_msgs.msg import Ptz
from geometry_msgs.msg import TransformStamped
from ptz_action_server_msgs.action import PtzFrame, PtzMove, PtzRelTo
from ptz_action_server_msgs.msg import PtzState
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros
import transforms3d


class AxisPtzControlNode(Node):
    """Provide a PTZ.action interface for axis_camera driver."""

    # How long before we consider the last message sent on the status topic to be stale?
    STATE_TIMEOUT = rclpy.duration.Duration(10)

    def __init__(self, name):
        super().__init__(name)
        self.cmd_ns = self.declare_parameter('cmd_ns', '/axis/cmd').value
        if self.cmd_ns[0] != '/':
            self.cmd_ns = f'/{self.cmd_ns}'
        self.cmd_pos_topic = f'{self.cmd_ns}/position'
        self.cmd_vel_topic = f'{self.cmd_ns}/velocity'
        self.status_topic = self.declare_parameter('status_topic', '/axis/state/position').value
        self.act_ns = self.declare_parameter('act_ns', '/axis').value
        self.invert_tilt = self.declare_parameter('invert_tilt', False).value
        self.invert_pan = self.declare_parameter('invert_pan', False).value

        self.min_pan = self.declare_parameter('min_pan', deg2rad(-170)).value
        self.max_pan = self.declare_parameter('max_pan', deg2rad(170)).value
        self.min_tilt = self.declare_parameter('min_tilt', 0).value
        self.max_tilt = self.declare_parameter('max_tilt', deg2rad(90)).value
        self.min_hw_zoom = self.declare_parameter('min_zoom', 1).value
        self.max_hw_zoom = self.declare_parameter('max_zoom', 9999).value

        self.camera_base_link = self.declare_parameter('camera_base_link', 'base_link').value

        self.publish_joint_states = self.declare_parameter('publish_joint_states', True).value
        self.pan_joint = self.declare_parameter('pan_joint', 'pan_joint').value
        self.tilt_joint = self.declare_parameter('tilt_joint', 'tilt_joint').value

        self.min_logical_zoom = self.declare_parameter('min_logical_zoom', 1.0).value
        self.max_logical_zoom = self.declare_parameter('max_logical_zoom', 24.0).value

        # allow 1 degree of error when tracking the position feedback
        # this is a little coarse, but the camera isn't a high-precision piece of kit anyway
        self.position_tolerance = deg2rad(1.0)

        # allow some tolerance in the zoom. This is a big range
        self.zoom_tolerance = 10.0

        self.goal_pan = 0.0
        self.goal_tilt = 0.0
        self.goal_zoom = 1.0

        # the camera's current state as reported by the underlying driver
        # initially None so we don't accidentally overwrite e.g. focus or brightness
        self.last_axis_state = None

        # The current logical PTZ state
        self.ptz_state = PtzState()
        self.ptz_state.pan = 0
        self.ptz_state.tilt = 0
        self.ptz_state.zoom = 1

        # is the camera currently moving?
        self.is_moving = False
        self.is_moving_lock = Lock()

        # Action Servers

        self._goal_handle = None
        self._goal_handle_ind = None
        self._goal_lock = Lock()
        self._goal_lock_ind = Lock()
        self.ptz_abs_action = ActionServer(
            self,
            PtzMove,
            f'{self.act_ns}/move_ptz/position_abs',
            execute_callback=self.ptz_abs_actionHandler,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback_direct,
        )
        self.ptz_rel_action = ActionServer(
            self,
            PtzMove,
            f'{self.act_ns}/move_ptz/position_rel',
            execute_callback=self.ptz_rel_actionHandler,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback_indirect,
        )
        self.vel_action = ActionServer(
            self,
            PtzMove,
            f'{self.act_ns}/move_ptz/velocity',
            execute_callback=self.vel_actionHandler,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback_direct,
        )
        self.tf_ptz_abs_action = ActionServer(
            self,
            PtzRelTo,
            f'{self.act_ns}/move_ptz/position_abs_tf',
            execute_callback=self.tf_ptz_abs_actionHandler,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback_indirect,
        )
        self.frame_ptz_action = ActionServer(
            self,
            PtzFrame,
            'move_ptz/frame',
            execute_callback=self.frame_ptz_actionHandler,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback_indirect,
        )

        self.last_state_at = self.get_clock().now()
        self.last_command_at = self.get_clock().now()

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.camera_vector_frame = f"camera_vector_frame_{str(uuid.uuid1()).replace('-', '_')}"

        self.start()

    def destroy_node(self):
        self.ptz_abs_action.destroy()
        self.ptz_rel_action.destroy()
        self.vel_action.destroy()
        self.tf_ptz_abs_action.destroy()
        self.frame_ptz_action.destroy()
        super().destroy_node()

    def start(self):
        """Start the subscribers, publishers, and service handlers."""
        self.cmd_pos_pub = self.create_publisher(Ptz, self.cmd_pos_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Ptz, self.cmd_vel_topic, 10)
        self.status_sub = self.create_subscription(Ptz, self.status_topic, self.state_callback, 10)
        self.status_pub = self.create_publisher(PtzState, f'{self.act_ns}/ptz_state', 10)

        if self.publish_joint_states:
            self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
            self.js_thread = Thread(target=self.joint_state_pub_thread)
            self.js_thread.start()

    def linear_rescale(self, x, old_min, old_max, new_min, new_max):
        return (x - old_min) / (old_max - old_min) * (new_max - new_min) + new_min

    def clamp(self, x, min, max, metavar):
        if x < min:
            self.get_logger().warning(f'Requested {metavar}={x} is too small. Clamping to {min}')
            return min
        if x > max:
            self.get_logger().warning(f'Requested {metavar}={x} is too large. Clamping to {max}')
            return max
        return x

    def state_callback(self, data):
        """
        Monitor the position of the camera to determine if it haa reached its goal position.

        Also saves the current state to self.last_axis_state

        @param data  The Ptz.msg information received on the topic
        """
        pan = data.pan
        tilt = data.tilt
        zoom = self.linear_rescale(
            data.zoom,
            self.min_hw_zoom,
            self.max_hw_zoom,
            self.min_logical_zoom,
            self.max_logical_zoom,
        )

        # invert pan & tilt if necessary
        if self.invert_tilt:
            tilt = -tilt
        if self.invert_pan:
            pan = -pan

        if self.is_moving:
            if self.ptz_state.mode == PtzState.MODE_POSITION:
                pan_remaining = abs(self.goal_pan - pan)
                tilt_remaining = abs(self.goal_tilt - tilt)
                zoom_remaining = abs(self.goal_zoom - zoom)

                # check if we've reached the goal
                if (
                    pan_remaining <= self.position_tolerance
                    and tilt_remaining <= self.position_tolerance
                    and zoom_remaining <= self.zoom_tolerance
                ):
                    self.get_logger().info('PTZ reached goal!')
                    self.is_moving = False
            elif self.ptz_state.mode == PtzState.MODE_VELOCITY:
                dpan = self.last_axis_state.pan - data.pan
                dtilt = self.last_axis_state.tilt - data.tilt
                dzoom = self.last_axis_state.zoom - data.zoom

                if dpan == 0 and dtilt == 0 and dzoom == 0:
                    self.get_logger().info('PTZ reached end-of-travel!')
                    self.is_moving = False

        self.last_axis_state = data
        self.last_state_at = self.get_clock().now()

        # publish the current PTZ state
        self.ptz_state.pan = pan
        self.ptz_state.tilt = tilt
        self.ptz_state.zoom = zoom
        self.status_pub.publish(self.ptz_state)

    def joint_state_pub_thread(self):
        js = JointState()

        rate = self.create_rate(10)
        while not rclpy.ok():
            js.header.stamp = self.get_clock().now()
            js.name = [self.pan_joint, self.tilt_joint]
            js.position = [self.ptz_state.pan, self.ptz_state.tilt]
            js.velocity = [0, 0]
            js.effort = [0, 0]
            self.joint_state_pub.publish(js)
            rate.sleep()

    # We need a seperate accepted goal callback for "indirect" actions:
    # (position_rel, position_abs_tf, and frame_ptz) since these call the "direct" action
    # position_abs having a seperate goal handle for each means we can have
    # an indirect and direct at the same time but not two in either category
    def handle_accepted_callback_direct(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def handle_accepted_callback_indirect(self, goal_handle):
        with self._goal_lock_ind:
            # This server only allows one goal at a time
            if self._goal_handle_ind is not None and self._goal_handle_ind.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle_ind.abort()
            self._goal_handle_ind = goal_handle
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def ptz_abs_actionHandler(self, goal_handle):
        """
        Handle PTZ.action requests to set absolute pan/tilt/zoom positions.

        Converts the PTZ goal into an Axis.msg we write to the axis_camera driver
        and then wait until the feedback topic reports that the PTU has finished moving

        @param goal_handle  The PtzGoal object to process
        """
        if self.last_axis_state is None:
            # We have never contacted the camera; block until we hear _something_
            self.get_logger().warning(
                'Still waiting for initial state from the camera. Cannot move yet'
            )
            goal_handle.abort()
            ptz_resp = PtzMove.Result()
            ptz_resp.success = False
            ptz_resp.message = 'Still waiting for initial state from the camera. Cannot move yet'
            return ptz_resp

        if self.get_clock().now() - self.last_state_at > self.STATE_TIMEOUT:
            # We haven't heard from the camera in a while; wait until the camera updates its status
            self.get_logger().warning(
                'Status of Axis camera is stale. Waiting for the camera to come back online'
            )
            goal_handle.abort()
            ptz_resp = PtzMove.Result()
            ptz_resp.success = False
            ptz_resp.message = (
                'Status of Axis camera is stale. Waiting for the camera to come back online'
            )
            return ptz_resp

        # The camera is alive and ready to move
        self.goal_pan = self.clamp(goal_handle.request.pan, self.min_pan, self.max_pan, 'pan')
        self.goal_tilt = self.clamp(
            goal_handle.request.tilt, self.min_tilt, self.max_tilt, 'tilt'
        )
        self.goal_zoom = self.clamp(
            goal_handle.request.zoom, self.min_logical_zoom, self.max_logical_zoom, 'zoom'
        )

        # send the command to the camera
        self.is_moving = True
        cmd = Ptz()
        cmd.pan = self.goal_pan
        cmd.tilt = self.goal_tilt
        cmd.zoom = self.linear_rescale(
            self.goal_zoom,
            self.min_logical_zoom,
            self.max_logical_zoom,
            self.min_hw_zoom,
            self.max_hw_zoom,
        )
        self.cmd_pos_pub.publish(cmd)
        self.last_command_at = self.get_clock().now()
        self.ptz_state.mode = PtzState.MODE_POSITION

        # sleep until joint_states reports that the pan and tilt joints have reached the
        # desired position OR have stopped moving
        rate = self.create_rate(2)
        timed_out = False
        while (
            self.is_moving
            and not goal_handle.is_cancel_requested
            and goal_handle.is_active
            and not timed_out
        ):
            feedback = PtzMove.Feedback()
            feedback.zoom_remaining = self.goal_zoom - self.ptz_state.zoom
            feedback.pan_remaining = self.goal_pan - self.ptz_state.pan
            feedback.tilt_remaining = self.goal_tilt - self.ptz_state.tilt
            goal_handle.publish_feedback(feedback)

            if self.get_clock().now() - self.last_state_at > self.STATE_TIMEOUT:
                timed_out = True
            else:
                self.get_logger().debug(
                    f'PTZ is still moving ({feedback.pan_remaining}, {feedback.tilt_remaining}, '
                    f'{feedback.zoom_remaining}) to go'
                )
                rate.sleep()

        if not goal_handle.is_active:
            self.get_logger().info('PTZ action aborted')
            self.is_moving = False
            return PtzMove.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.is_moving = False
            self.get_logger().info('PTZ action canceled')
            return PtzMove.Result()
        if timed_out:
            self.get_logger().warning('Timed out waiting for axis camera state')
            self.is_moving = False
            ptz_resp = PtzMove.Result()
            ptz_resp.success = False
            ptz_resp.message = (
                'Timed out waiting for axis camera state; did the camera go offline?'
            )
            self.ptz_state.mode = PtzState.MODE_IDLE
            goal_handle.abort()
            return ptz_resp
        self.get_logger().debug('PTZ action completed successfully')
        ptz_resp = PtzMove.Result()
        ptz_resp.success = True
        self.ptz_state.mode = PtzState.MODE_IDLE
        goal_handle.succeed()
        return ptz_resp

    def ptz_rel_actionHandler(self, goal_handle):
        """
        Handle moving the camera to a relative pan/tilt/zoom from current.

        Converts the relative pan/tilt/zoom to absolute positions and uses the position_abs action
        """
        abs_goal = PtzMove.Goal(
            self.ptz_state.pan + goal_handle.request.pan,
            self.ptz_state.tilt + goal_handle.request.tilt,
            self.ptz_state.zoom + goal_handle.request.zoom,
        )

        ptz_client = ActionClient(self, PtzMove, 'move_ptz/position_abs')
        ptz_client.wait_for_server()

        def feedback_cb(feedback):
            goal_handle.publish_feedback(PtzMove.Feedback(feedback.feedback))

        abs_goal_future = ptz_client.send_goal_async(
            abs_goal,
            feedback_cb=feedback_cb,
        )

        rclpy.spin_until_future_complete(self, abs_goal_future)
        abs_goal_handle = abs_goal_future.result()
        # Check if goal accepted (it always is here)
        get_result_future = abs_goal_handle.get_result_async()

        while (
            not get_result_future.done()
            and not goal_handle.is_cancel_requested
            and goal_handle.is_active
        ):
            rclpy.spin_once(self, timeout_sec=1)
        if not goal_handle.is_active:
            self.get_logger().info('PTZ action aborted')
            return PtzMove.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('PTZ action canceled')
            return PtzMove.Result()

        result = get_result_future.result().result
        status = get_result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
            return result
        if status == GoalStatus.STATUS_ABORTED:
            goal_handle.abort()
            return result
        goal_handle.succeed()
        return result

    def vel_actionHandler(self, goal_handle):
        """
        Handle PTZ.action requests for velocity control.

        Converts the PTZ goal into an Axis.msg we write to the axis_camera driver
        and then wait until the feedback topic reports that the PTU has finished moving

        @param goal_handle  The PtzGoal object to process
        """
        if self.last_axis_state is None:
            # We have never contacted the camera; block until we hear _something_
            self.get_logger().warning(
                'Still waiting for initial state from the camera. Cannot move yet'
            )
            goal_handle.abort()
            ptz_resp = PtzMove.Result()
            ptz_resp.success = False
            ptz_resp.message = 'Still waiting for initial state from the camera. Cannot move yet'
            return ptz_resp
        if self.get_clock().now() - self.last_state_at > self.STATE_TIMEOUT:
            # We haven't heard from the camera in a while; block until the camera
            # updates its status again
            self.get_logger().warning(
                'Status of Axis camera is stale. Waiting for the camera to come back online'
            )
            goal_handle.abort()
            ptz_resp = PtzMove.Result()
            ptz_resp.success = False
            ptz_resp.message = (
                'Status of Axis camera is stale. Waiting for the camera to come back online'
            )
            return ptz_resp
        # The camera is alive and ready to move
        cmd = Ptz()
        cmd.pan = goal_handle.request.pan
        cmd.tilt = goal_handle.request.tilt
        cmd.zoom = goal_handle.request.zoom
        self.cmd_vel_pub.publish(cmd)
        if cmd.pan != 0 or cmd.tilt != 0 or cmd.zoom != 0:
            self.is_moving = True
            self.ptz_state.mode = PtzState.MODE_VELOCITY

            rate = self.create_rate(2)
            timed_out = False
            while (
                self.is_moving
                and goal_handle.is_active
                and not goal_handle.is_cancel_requested
                and not timed_out
            ):
                feedback = PtzMove.Feedback()
                feedback.zoom_remaining = math.copysign(1, goal_handle.zoom)
                feedback.pan_remaining = math.copysign(1, goal_handle.pan)
                feedback.tilt_remaining = math.copysign(1, goal_handle.tilt)
                self.vel_action.publish_feedback(feedback)

                if self.get_clock().now() - self.last_state_at > self.STATE_TIMEOUT:
                    timed_out = True
                else:
                    self.get_logger().debug(
                        f'PTZ is still moving ({feedback.pan_remaining}, {feedback.tilt_remaining}'
                        f', {feedback.zoom_remaining}) to go'
                    )
                    rate.sleep()

            if not goal_handle.is_active:
                self.get_logger().info('Velocity action aborted')
                self.is_moving = False
                self.ptz_state.mode = PtzState.MODE_IDLE
                return PtzMove.Result()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.is_moving = False
                self.vel_state = PtzState.MODE_IDLE
                self.get_logger().info('Velocity action canceled')
                return PtzMove.Result()
            if timed_out:
                self.get_logger().warning('Timed out waiting for axis camera state')
                self.is_moving = False
                ptz_resp = PtzMove.Result()
                ptz_resp.success = False
                self.vel_state = PtzState.MODE_IDLE
                ptz_resp.message = (
                    'Timed out waiting for axis camera state; did the camera go offline?'
                )
                goal_handle.abort()
                return ptz_resp
            self.get_logger().debug('Velocity action completed successfully')
            ptz_resp = PtzMove.Result()
            ptz_resp.success = True
            self.ptz_state.mode = PtzState.MODE_IDLE
            goal_handle.succeed()
            return ptz_resp
        self.get_logger().debug('Velocity action completed successfully')
        ptz_resp = PtzMove.Result()
        ptz_resp.success = True
        self.is_moving = False
        self.ptz_state.mode = PtzState.MODE_IDLE
        goal_handle.succeed()
        return ptz_resp

    def tf_ptz_abs_actionHandler(self, goal_handle):
        """
        Handle a TF-aware absolute PTZ action.

        We calculate the TF between the camera's base link & the link provided in the request,
        and then use the normal absolute PTZ action internally to move the camera to the
        corresponding position
        """
        # calculate the initial absolute goal as if there is no TF to apply
        abs_goal = PtzMove.Goal(
            goal_handle.request.pan, goal_handle.request.tilt, goal_handle.request.zoom
        )

        if goal_handle.request.frame_id:
            try:
                # add a temporary frame to represent the desired camera vector
                camera_vector_q = transforms3d.euler.euler2quat(
                    0, -goal_handle.request.tilt, goal_handle.request.pan
                )
                static_tf = TransformStamped()
                static_tf.header.stamp = self.get_clock().now()
                static_tf.header.frame_id = goal_handle.request.frame_id
                static_tf.child_frame_id = self.camera_vector_frame
                static_tf.transform.rotation.x = camera_vector_q[1]
                static_tf.transform.rotation.y = camera_vector_q[2]
                static_tf.transform.rotation.z = camera_vector_q[3]
                static_tf.transform.rotation.w = camera_vector_q[0]
                self.tf_broadcaster.sendTransform(static_tf)

                # The the RPY transformation between the camera base link & the reference frame
                tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(100.0))
                tf2_ros.TransformListener(tf_buffer)
                tf_stamped = tf_buffer.lookup_transform(
                    self.camera_base_link,
                    self.camera_vector_frame,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(5.0),
                )
                q = tf_stamped.transform.rotation
                (_roll, pitch, yaw) = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])

                # yaw & pitch are right-handed, but pan & tilt are left-handed, so invert them
                abs_goal.pan = -yaw
                abs_goal.tilt = -pitch

                # clamp the adjusted pan to lie within [-pi, pi]
                while abs_goal.pan < -math.pi:
                    abs_goal.pan += math.pi
                while abs_goal.pan > math.pi:
                    abs_goal.pan -= math.pi

                # clamp the adjusted tilt to lie within [-pi/2, pi/2]
                while abs_goal.tilt < -math.pi / 2:
                    abs_goal.tilt += math.pi
                while abs_goal.tilt > math.pi / 2:
                    abs_goal.tilt -= math.pi

            except Exception as err:
                self.get_logger().error(
                    'Failed to look up transform between '
                    f'{self.camera_base_link} and {goal_handle.request.frame_id}: {err}'
                )

        ptz_client = ActionClient(self, PtzMove, 'move_ptz/position_abs')
        ptz_client.wait_for_server()

        def feedback_cb(feedback):
            goal_handle.publish_feedback(PtzRelTo.Feedback(feedback.feedback))

        abs_goal_future = ptz_client.send_goal_async(
            abs_goal,
            feedback_cb=feedback_cb,
        )

        rclpy.spin_until_future_complete(self, abs_goal_future)
        abs_goal_handle = abs_goal_future.result()
        # Check if goal accepted (it always is here)
        get_result_future = abs_goal_handle.get_result_async()

        while (
            not get_result_future.done()
            and not goal_handle.is_cancel_requested
            and goal_handle.is_active
        ):
            rclpy.spin_once(self, timeout_sec=1)
        if not goal_handle.is_active:
            self.get_logger().info('PTZ action aborted')
            return PtzRelTo.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('PTZ action canceled')
            return PtzRelTo.Result()

        result = get_result_future.result().result
        status = get_result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
            return PtzRelTo.Result(result)
        if status == GoalStatus.STATUS_ABORTED:
            goal_handle.abort()
            return PtzRelTo.Result(result)
        goal_handle.succeed()
        return PtzRelTo.Result(result)

    def frame_ptz_actionHandler(self, goal_handle):
        """Pan & tilt the camera to point to a specific frame in the TF tree."""
        abs_goal = PtzMove.Goal()
        abs_goal.zoom = goal_handle.request.zoom

        try:
            # The the transformation between the camera base link & the reference frame
            tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(100.0))
            tf2_ros.TransformListener(tf_buffer)
            tf_stamped = tf_buffer.lookup_transform(
                self.camera_base_link,
                goal_handle.request.frame_id,
                rclpy.time.Time(),
                rclpy.duration.Duration(5.0),
            )

            # we want to get the pan & tilt needed to point at the frame
            r = math.sqrt(
                tf_stamped.transform.translation.x**2
                + tf_stamped.transform.translation.y**2
                + tf_stamped.transform.translation.z**2
            )
            yaw = math.atan2(
                tf_stamped.transform.translation.y, tf_stamped.transform.translation.x
            )
            pitch = -math.asin(tf_stamped.transform.translation.z / r)

            # yaw & pitch are right-handed, but pan & tilt are left-handed, so invert them
            abs_goal.pan = -yaw
            abs_goal.tilt = -pitch

            # clamp the adjusted pan to lie within [-pi, pi]
            while abs_goal.pan < -math.pi:
                abs_goal.pan += math.pi
            while abs_goal.pan > math.pi:
                abs_goal.pan -= math.pi

            # clamp the adjusted tilt to lie within [-pi/2, pi/2]
            while abs_goal.tilt < -math.pi / 2:
                abs_goal.tilt += math.pi
            while abs_goal.tilt > math.pi / 2:
                abs_goal.tilt -= math.pi

            ptz_client = ActionClient(self, PtzMove, 'move_ptz/position_abs')
            ptz_client.wait_for_server()

            def feedback_cb(feedback):
                goal_handle.publish_feedback(PtzFrame.Feedback(feedback.feedback))

            abs_goal_future = ptz_client.send_goal_async(
                abs_goal,
                feedback_cb=feedback_cb,
            )

            rclpy.spin_until_future_complete(self, abs_goal_future)
            abs_goal_handle = abs_goal_future.result()
            # Check if goal accepted (it always is here)
            get_result_future = abs_goal_handle.get_result_async()

            while (
                not get_result_future.done()
                and not goal_handle.is_cancel_requested
                and goal_handle.is_active
            ):
                rclpy.spin_once(self, timeout_sec=1)
            if not goal_handle.is_active:
                self.get_logger().info('PTZ action aborted')
                return PtzFrame.Result()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('PTZ action canceled')
                return PtzFrame.Result()

            result = get_result_future.result().result
            status = get_result_future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                goal_handle.succeed()
                return PtzFrame.Result(result)
            if status == GoalStatus.STATUS_ABORTED:
                goal_handle.abort()
                return PtzFrame.Result(result)
            goal_handle.succeed()
            return PtzFrame.Result(result)

        except Exception as err:
            err_msg = (
                f'Failed to look up transform between {self.camera_base_link} and '
                f'{goal_handle.request.frame_id}: {err}'
            )
            self.get_logger().error(err_msg)
            result = PtzFrame.Result()
            result.success = False
            result.message = err_msg
            goal_handle.abort()
            return result


def main(args=None):
    rclpy.init(args=args)
    node = AxisPtzControlNode('axis_ptz_action_server_node')
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
