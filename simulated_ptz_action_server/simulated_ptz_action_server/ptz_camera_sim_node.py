#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import math
import tf2_ros
from threading import Lock
import transforms3d
import uuid

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from ptz_action_server_msgs.action import PtzMove, PtzFrame, PtzRelTo
from ptz_action_server_msgs.msg import PtzState
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64

import cv2
from cv_bridge import CvBridge, CvBridgeError


class SimulatedPtzCameraNode(Node):
    """Node that implements the PTZ action server for a simulated camera

    Actual camera data and joint states are provided by Gazebo plugins
    We use a velocity_controllers/JointPositionController to control the actual pan and tilt
    joints, subscribing to /joint_states to determine when the camera has finished moving after
    receiving a command.

    Zoom is implemented as a digital zoom, so expect poor resolution at higher levels.
    """
    def __init__(self, name):
        super().__init__(name)
        self.pan_limits = [
            self.declare_parameter('min_pan', -math.pi).value,
            self.declare_parameter('max_pan', math.pi).value
        ]
        self.tilt_limits = [
            self.declare_parameter('min_tilt', -math.pi/2).value,
            self.declare_parameter('max_tilt', math.pi/2).value
        ]
        self.zoom_limits = [
            self.declare_parameter('min_logical_zoom', 1.0).value,
            self.declare_parameter('max_logical_zoom', 5.0).value
        ]

        self.camera_base_link = self.declare_parameter("camera_base_link", "base_link").value

        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.current_zoom = 1.0
        self.is_moving = False
        self.is_zooming = False

        self.ANGLE_TOLERANCE = 1.0 * math.pi / 180.0  # allow 1 degree either way for PTZ commands\
        self.ZOOM_RATE = 0.1

        self.bridge = CvBridge()

        self._goal_handle = None
        self._goal_handle_ind = None
        self._goal_lock = Lock()
        self._goal_lock_ind = Lock()
        self.ptz_abs_action = ActionServer(
            self,
            PtzMove,
            '/move_ptz/position_abs',
            execute_callback=self.ptz_abs_actionHandler,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback_direct)
        self.ptz_rel_action = ActionServer(
            self,
            PtzMove,
            '/move_ptz/position_rel',
            execute_callback=self.ptz_rel_actionHandler,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback_indirect)
        self.tf_ptz_abs_action = ActionServer(
            self,
            PtzRelTo,
            '/move_ptz/position_abs_tf',
            execute_callback=self.tf_ptz_abs_actionHandler,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback_indirect)
        self.frame_ptz_action = ActionServer(
            self,
            PtzFrame,
            'move_ptz/frame',
            execute_callback=self.frame_ptz_actionHandler,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback_indirect)
        self.run()

    def run(self):
        """Start the action server, begin publishing the joint states
        """
        self.position_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.img_sub = self.create_subscription(Image, 'image_raw_nozoom', self.image_callback, 10)
        self.img_pub = self.create_publisher(Image, 'image_raw', 10)
        self.state_pub = self.create_publisher(PtzState, 'ptz_state', 10)

        self.pan_cmd = self.create_publisher(Float64, '/pan_position_controller/command', 10)
        self.tilt_cmd = self.create_publisher(Float64, '/tilt_position_controller/command', 10)

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.camera_vector_frame = f"camera_vector_frame_{str(uuid.uuid1()).replace('-', '_')}"

        # spin and publish the ptz status at 10Hz
        self.ptz_state_timer = self.create_timer(0.1, self.ptz_status_callback)

    def destroy_node(self):
        self.ptz_abs_action.destroy()
        self.ptz_rel_action.destroy()
        self.tf_ptz_abs_action.destroy()
        self.frame_ptz_action.destroy()
        self.ptz_state_timer.destroy()
        super().destroy_node()

    def ptz_status_callback(self):
        psn = PtzState()
        psn.pan = self.current_pan
        psn.tilt = self.current_tilt
        psn.zoom = self.current_zoom
        if self.is_moving or self.is_zooming:
            psn.mode = PtzState.MODE_POSITION
        else:
            psn.mode = PtzState.MODE_IDLE
        self.state_pub.publish(psn)

# We need a seperate accepted goal callback for "indirect" actions:
# (position_rel, position_abs_tf, and frame_ptz) since these call the "direct" action position_abs
# having a seperate goal handle for each means we can have an indirect and direct at the same time
# but not two in either category
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

    def ptz_abs_actionHandler(self, goal_handle):
        """Handler for moving the camera to an absolute pan/tilt/zoom

        Sends the desired pan & tilt positions to the JointPositionControllers and publishes feedback at 10Hz
        until the camera reaches the desired position
        """
        goal_pan = goal_handle.request.pan
        if goal_pan < self.pan_limits[0]:
            self.get_logger().warning(f"Pan of {goal_handle.request.pan} is too small; clamping to {self.pan_limits[0]}")
            goal_pan = self.pan_limits[0]
        elif goal_pan > self.pan_limits[1]:
            self.get_logger().warning(f"Pan of {goal_handle.request.pan} is too large; clamping to {self.pan_limits[1]}")
            goal_pan = self.pan_limits[1]

        goal_tilt = goal_handle.request.tilt
        if goal_tilt < self.tilt_limits[0]:
            self.get_logger().warning(f"Tilt of {goal_handle.request.tilt} is too small; clamping to {self.tilt_limits[0]}")
            goal_tilt = self.tilt_limits[0]
        elif goal_tilt > self.tilt_limits[1]:
            self.get_logger().warning(f"Tilt of {goal_handle.request.tilt} is too large; clamping to {self.tilt_limits[1]}")
            goal_tilt = self.tilt_limits[1]

        goal_zoom = goal_handle.request.zoom
        if goal_zoom < self.zoom_limits[0]:
            self.get_logger().warning(f"Zoom of {goal_handle.request.zoom} is too small; clamping to {self.zoom_limits[0]}")
            goal_zoom = self.zoom_limits[0]
        elif goal_zoom > self.zoom_limits[1]:
            self.get_logger().warning(f"Zoom of {goal_handle.request.zoom} is too large; clamping to {self.zoom_limits[1]}")
            goal_zoom = self.zoom_limits[1]

        delta_zoom = 0.0
        if self.current_zoom > goal_zoom:
            delta_zoom = -self.ZOOM_RATE
        elif self.current_zoom < goal_zoom:
            delta_zoom = self.ZOOM_RATE

        msg = Float64(data=goal_pan)
        self.pan_cmd.publish(msg)
        msg = Float64(data=goal_tilt)
        self.tilt_cmd.publish(msg)

        rate = self.create_rate(10)
        is_moving = True
        is_zooming = True
        fback = PtzMove.Feedback()
        is_error = False
        while (is_moving or is_zooming) and not goal_handle.is_cancel_requested and goal_handle.is_active and not is_error:
            rate.sleep()

            # adjust the zoom
            if delta_zoom == 0.0 or abs(goal_zoom - self.current_zoom) <= self.ZOOM_RATE:
                is_zooming = False
                self.current_zoom = goal_zoom
            else:
                self.current_zoom = self.current_zoom + delta_zoom
            zoom_to_go = goal_zoom - self.current_zoom

            # the pan angle can wrap around 360 degrees, so it's possible the current pan and desired pan are
            # on opposite sides of the limit (e.g. 359 degrees moving toward 0, which should result in an error of 1
            # not 359)
            pan_to_go = min(abs(goal_pan - self.current_pan), \
                            abs((goal_pan - 2*math.pi) - self.current_pan), \
                            abs((goal_pan + 2*math.pi) - self.current_pan) )
            tilt_to_go = abs(goal_tilt - self.current_tilt)

            # check if we've reached the goal
            if pan_to_go <= self.ANGLE_TOLERANCE and tilt_to_go <= self.ANGLE_TOLERANCE:
                is_moving = False

            # check if we've stopped moving; this may indicate an error with the controller
            if fback.pan_remaining == pan_to_go and fback.tilt_remaining == tilt_to_go and fback.zoom_remaining == zoom_to_go:
                self.get_logger().warning("Camera seems to have stopped moving; aborting PTZ action")
                is_error = True
                is_moving = False

            fback.pan_remaining = pan_to_go
            fback.tilt_remaining = tilt_to_go
            fback.zoom_remaining = zoom_to_go
            goal_handle.publish_feedback(fback)

        if not goal_handle.is_active:
            self.get_logger().info('PTZ action aborted')
            self.is_moving = False
            return PtzMove.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('PTZ action canceled')
            return PtzMove.Result()
        elif is_error:
            self.get_logger().warning("PTZ action failed")
            ptz_resp = PtzMove.Result()
            ptz_resp.success = False
            ptz_resp.message = "Camera seems to have stopped moving; aborting PTZ action."
            goal_handle.abort()
            return ptz_resp
        else:
            self.get_logger().info("PTZ simulator reached goal position")
            ptz_resp = PtzMove.Result()
            ptz_resp.success = True
            goal_handle.succeed()
            return ptz_resp


    def ptz_rel_actionHandler(self, goal_handle):
        """Handler for moving the camera to a relative pan/tilt/zoom from current

        Converts the relative pan/tilt/zoom to absolute positions and uses the position_abs action
        """
        abs_goal = PtzMove.Goal(self.ptz_state.pan + goal_handle.request.pan,
                           self.ptz_state.tilt + goal_handle.request.tilt,
                           self.ptz_state.zoom + goal_handle.request.zoom
        )

        ptz_client = ActionClient(self, PtzMove, "move_ptz/position_abs")
        ptz_client.wait_for_server()
        def feedback_cb(feedback):
            goal_handle.publish_feedback(PtzMove.Feedback(feedback.feedback))

        abs_goal_future = ptz_client.send_goal_async(abs_goal,
            feedback_cb = feedback_cb,
        )

        rclpy.spin_until_future_complete(self, abs_goal_future)
        abs_goal_handle = abs_goal_future.result()
        # Check if goal accepted (it always is here)
        get_result_future = abs_goal_handle.get_result_async()

        while not get_result_future.done() and not goal_handle.is_cancel_requested and goal_handle.is_active:
            rclpy.spin_once(self,timeout_sec=1)
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
        elif status == GoalStatus.STATUS_ABORTED:
            goal_handle.abort()
            return result
        else:
            goal_handle.succeed()
            return result

    def tf_ptz_abs_actionHandler(self, goal_handle):
        """Handler for a TF-aware absolute PTZ action

        We calculate the TF between the camera's base link & the link provided in the request, and then use
        the normal absolute PTZ action internally to move the camera to the corresponding position
        """
        # calculate the initial absolute goal as if there is no TF to apply
        abs_goal = PtzMove.Goal(goal_handle.request.pan,
                           goal_handle.request.tilt,
                           goal_handle.request.zoom
        )

        if goal_handle.request.frame_id:
            try:
                # add a temporary frame to represent the desired camera vector
                camera_vector_q = transforms3d.euler.euler2quat(0, -goal_handle.request.tilt, goal_handle.request.pan)
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
                tf_listener = tf2_ros.TransformListener(tf_buffer)
                tf_stamped = tf_buffer.lookup_transform(self.camera_base_link, self.camera_vector_frame, rclpy.time.Time(), rclpy.duration.Duration(5.0))
                q = tf_stamped.transform.rotation
                (roll, pitch, yaw) = transforms3d.euler.quat2euler([q.w, q.x, q.y, q.z])

                # yaw & pitch are right-handed, but pan & tilt are left-handed, so invert them
                abs_goal.pan = -yaw
                abs_goal.tilt = -pitch

                # clamp the adjusted pan to lie within [-pi, pi]
                while abs_goal.pan < -math.pi:
                    abs_goal.pan += math.pi
                while abs_goal.pan > math.pi:
                    abs_goal.pan -= math.pi

                # clamp the adjusted tilt to lie within [-pi/2, pi/2]
                while abs_goal.tilt < -math.pi/2:
                    abs_goal.tilt += math.pi
                while abs_goal.tilt > math.pi/2:
                    abs_goal.tilt -= math.pi

            except Exception as err:
                self.get_logger().error(f"Failed to look up transform between {self.camera_base_link} and {goal_handle.request.frame_id}: {err}")

        ptz_client = ActionClient(self, PtzMove, "move_ptz/position_abs")
        ptz_client.wait_for_server()
        def feedback_cb(feedback):
            goal_handle.publish_feedback(PtzRelTo.Feedback(feedback.feedback))

        abs_goal_future = ptz_client.send_goal_async(abs_goal,
            feedback_cb = feedback_cb,
        )

        rclpy.spin_until_future_complete(self, abs_goal_future)
        abs_goal_handle = abs_goal_future.result()
        # Check if goal accepted (it always is here)
        get_result_future = abs_goal_handle.get_result_async()

        while not get_result_future.done() and not goal_handle.is_cancel_requested and goal_handle.is_active:
            rclpy.spin_once(self,timeout_sec=1)
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
        elif status == GoalStatus.STATUS_ABORTED:
            goal_handle.abort()
            return PtzRelTo.Result(result)
        else:
            goal_handle.succeed()
            return PtzRelTo.Result(result)

    def frame_ptz_actionHandler(self, goal_handle):
        """Pan & tilt the camera to point to a specific frame in the TF tree
        """
        abs_goal = PtzMove.Goal()
        abs_goal.zoom = goal_handle.request.zoom

        try:
            # The the transformation between the camera base link & the reference frame
            tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(100.0))
            tf_listener = tf2_ros.TransformListener(tf_buffer)
            tf_stamped = tf_buffer.lookup_transform(self.camera_base_link, goal_handle.request.frame_id, rclpy.time.Time(), rclpy.duration.Duration(5.0))

            # we want to get the pan & tilt needed to point at the frame
            r = math.sqrt(tf_stamped.transform.translation.x**2 + tf_stamped.transform.translation.y**2 + tf_stamped.transform.translation.z**2)
            yaw = math.atan2(tf_stamped.transform.translation.y, tf_stamped.transform.translation.x)
            pitch = -math.asin(tf_stamped.transform.translation.z/r)

            # yaw & pitch are right-handed, but pan & tilt are left-handed, so invert them
            abs_goal.pan = -yaw
            abs_goal.tilt = -pitch

            # clamp the adjusted pan to lie within [-pi, pi]
            while abs_goal.pan < -math.pi:
                abs_goal.pan += math.pi
            while abs_goal.pan > math.pi:
                abs_goal.pan -= math.pi

            # clamp the adjusted tilt to lie within [-pi/2, pi/2]
            while abs_goal.tilt < -math.pi/2:
                abs_goal.tilt += math.pi
            while abs_goal.tilt > math.pi/2:
                abs_goal.tilt -= math.pi

            ptz_client = ActionClient(self, PtzMove, "move_ptz/position_abs")
            ptz_client.wait_for_server()
            def feedback_cb(feedback):
                goal_handle.publish_feedback(PtzFrame.Feedback(feedback.feedback))

            abs_goal_future = ptz_client.send_goal_async(abs_goal,
                feedback_cb = feedback_cb,
            )

            rclpy.spin_until_future_complete(self, abs_goal_future)
            abs_goal_handle = abs_goal_future.result()
            # Check if goal accepted (it always is here)
            get_result_future = abs_goal_handle.get_result_async()

            while not get_result_future.done() and not goal_handle.is_cancel_requested and goal_handle.is_active:
                rclpy.spin_once(self,timeout_sec=1)
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
            elif status == GoalStatus.STATUS_ABORTED:
                goal_handle.abort()
                return PtzFrame.Result(result)
            else:
                goal_handle.succeed()
                return PtzFrame.Result(result)

        except Exception as err:
            err_msg = f"Failed to look up transform between {self.camera_base_link} and {goal_handle.request.frame_id}: {err}"
            self.get_logger().error(err_msg)
            result = PtzFrame.Result()
            result.success = False
            result.message = err_msg
            goal_handle.abort()
            return result

    def joint_state_callback(self, data):
        """Read the current joint_states and update the camera's current position
        """
        if 'ptz_camera_pan_joint' in data.name:
            index = data.name.index('ptz_camera_pan_joint')
            self.current_pan = data.position[index]

        if 'ptz_camera_tilt_joint' in data.name:
            index = data.name.index('ptz_camera_tilt_joint')
            self.current_tilt = data.position[index]

    def image_callback(self, img):
        """Apply a digital zoom to the input image and republish it
        """
        if self.current_zoom != 1.0:
            # convert to OpenCV, zoom into the middle, convert back to ROS, and publish
            try:
                cv_img = self.bridge.imgmsg_to_cv2(img, "8UC3")
                (rows,cols,channels) = cv_img.shape

                # if the zoom factor is Z, crop the image to the center 1/Z and resize back the original dimensions
                z = self.current_zoom    # this is volatile, so store a local copy!
                w = cols / z
                h = rows / z
                cv_img = cv_img[int(rows/2 - h/2) : int(rows/2 + h/2), int(cols/2 - w/2) : int(cols/2 + w/2)]
                cv_img = cv2.resize(cv_img, (cols, rows))

                # convert back to a ROS image
                img = self.bridge.cv2_to_imgmsg(cv_img, "rgb8")

            except CvBridgeError as err:
                self.get_logger().error(f"OpenCV Error {err}")

        # (re-)publish the image
        self.img_pub.publish(img)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedPtzCameraNode('simulated_ptz_camera_controller')
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()