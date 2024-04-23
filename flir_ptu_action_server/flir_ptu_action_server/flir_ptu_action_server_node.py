#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus
from ptz_action_server_msgs.action import PtzMove
from ptz_action_server_msgs.msg import PtzState
from sensor_msgs.msg import JointState

from math import pi
from threading import Lock

class FlirD46PtzControlNode(Node):
    """Provides a PTZ.action compatible interface for sending pan-tilt positions to the flir_ptu driver
    """

    HW_MIN_PAN = -2.775073510670984
    HW_MAX_PAN = 2.775073510670984

    HW_MIN_TILT = -0.8203047484373349
    HW_MAX_TILT = 0.5410520681182421

    def __init__(self, name):
        super().__init__(name)
        self.cmd_topic = self.declare_parameter('~cmd_topic', '/ptu/cmd').value
        self.act_ns = self.declare_parameter('~act_ns', '/ptu').value
        self.pan_joint = self.declare_parameter('~pan_joint', 'ptu_pan').value
        self.tilt_joint = self.declare_parameter('~tilt_joint', 'ptu_tilt').value

        self.MIN_PAN = float(self.declare_parameter('~min_pan', -2.356194490192345).value)
        self.MAX_PAN = float(self.declare_parameter('~max_pan', 2.356194490192345).value)
        self.MIN_TILT = float(self.declare_parameter('~min_tilt', -0.7853981633974483).value)
        self.MAX_TILT = float(self.declare_parameter('~max_tilt', 1.5707963267948966).value)

        self.INVERT_PAN = bool(self.declare_parameter('~invert_pan', False).value)
        self.INVERT_TILT = bool(self.declare_parameter('~invert_tilt', False).value)

        # allow 1 degree of error when tracking the position feedback
        self.position_tolerance = 1 * pi / 180.0
        self.goal_pan = 0.0
        self.goal_tilt = 0.0

        self.current_pan = 0.0
        self.current_tilt = 0.0

        self.is_moving = False

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
            handle_accepted_callback=self.handle_accepted_callback_direct)
        self.ptz_rel_action = ActionServer(
            self,
            PtzMove,
            f'{self.act_ns}/move_ptz/position_rel',
            execute_callback=self.ptz_rel_actionHandler,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback_indirect)
    def destroy_node(self):
        self.ptz_abs_action.destroy()
        self.ptz_rel_action.destroy()
        super().destroy_node()

    def start(self):
        """Starts the subscribers, publishers, and service handlers
        """
        self.cmd_pub = self.create_publisher(JointState, self.cmd_topic)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback)
        self.status_pub = self.create_publisher(PtzState, f'{self.act_ns}/ptz_state')
        self.ptz_abs_srv.start()
        self.ptz_rel_srv.start()

    def clamp_value(self, x, min, max, metavar='pan'):
        if x < min:
            self.get_logger().warning(f'Requested {metavar}={x} is too small. Clamping to {min}')
            return min
        elif x > max:
            self.get_logger().warning(f'Requested {metavar}={x} is too large. Clamping to {max}')
            return max
        else:
            return x

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
        """Handler for the PTZ.action call

        Converts the PTZ request into a JointState we write to the flir_ptu driver
        and then wait until /joint_states reports that the PTU has finished moving

        @param goal_handle  The PtzActionGoal object to process
        """
        # make sure the requested position is valid
        pan = self.clamp_value(goal_handle.request.pan, self.MIN_PAN, self.MAX_PAN, 'pan')
        tilt = self.clamp_value(goal_handle.request.tilt, self.MIN_PAN, self.MAX_TILT, 'tilt')

        self.goal_pan = pan
        self.goal_tilt = tilt

        # tell the underlying driver to start moving
        self.is_moving = True
        cmd = self.mk_joint_state(self.goal_pan, self.goal_tilt)
        self.cmd_pub.publish(cmd)

        # sleep until joint_states reports that the pan and tilt joints have reached the desired position OR
        # have stopped moving
        rate = self.create_rate(10)

        # Note that the Flir PTU driver sometimes reports bad joint state information (especially over ethernet?)
        # The unit moves correctly, but the position and speed are incorrect.
        # To avoid the action server hanging, implement a hard timeout
        MAX_WAIT = 60.0         # moving from -135 to + 135 takes ~50s.
        elapsed_wait = 0.0
        while self.is_moving and not goal_handle.is_cancel_requested and goal_handle.is_active and elapsed_wait < MAX_WAIT:
            feedback = PtzMove.Feedback()
            feedback.zoom_remaining = 0.0
            feedback.pan_remaining = self.goal_pan - self.current_pan
            feedback.tilt_remaining = self.goal_tilt - self.current_tilt
            goal_handle.publish_feedback(feedback)
            rate.sleep()
            elapsed_wait = elapsed_wait + 0.1   # rate is 10Hz, so each spin is 0.1s

        if not goal_handle.is_active:
            self.get_logger().info('PTZ action aborted')
            self.is_moving = False
            return PtzMove.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.is_moving = False
            self.get_logger().info('PTZ action canceled')
            return PtzMove.Result()
        elif elapsed_wait >= MAX_WAIT:
            self.get_logger().warning("PTZ action timed out.  Assuming movement is complete")
            self.is_moving = False
            ptz_resp = PtzMove.Result()
            ptz_resp.success = True
            ptz_resp.message = "PTZ action timed out.  Assuming movement is complete"
            goal_handle.succeed()
            return ptz_resp
        else:
            self.get_logger().debug("PTZ action completed successfully")
            ptz_resp = PtzMove.Result()
            ptz_resp.success = True
            goal_handle.succeed()
            return ptz_resp

    def ptz_rel_actionHandler(self, goal_handle):
        """Handler for moving the camera to a relative pan/tilt/zoom from current

        Converts the relative pan/tilt to absolute positions and uses the position_abs action
        """
        abs_goal = PtzMove.Goal(self.current_pan + goal_handle.request.pan, self.current_tilt + goal_handle.request.tilt, 0)
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

    def joint_state_callback(self, data):
        """Monitors /joint_states to determine if the pan and tilt joints have reached their goal positions yet

        @param data  The JointState information received on /joint_states
        """
        try:
            pan_index = data.name.index(self.pan_joint)
            tilt_index = data.name.index(self.tilt_joint)

            # read the current pan and tilt positions and calculate their deltas.
            # we cannot use the velocity field, as this is the joints' desired velocity, not
            # its actual velocity.
            pan = data.position[pan_index]
            tilt = data.position[tilt_index]

            # convert from the hardware coordinate range to the action coordiante range
            if self.INVERT_PAN:
                pan = -pan
            while pan > self.MAX_PAN:
                pan = pan - 2*pi
            while pan < self.MIN_PAN:
                pan = pan + 2*pi

            if self.INVERT_TILT:
                tilt = -tilt
            while tilt > self.MAX_TILT:
                tilt = tilt - 2*pi
            while tilt < self.MIN_TILT:
                tilt = tilt + 2*pi

            dpan = pan - self.current_pan
            dtilt = tilt - self.current_tilt

            self.current_pan = pan
            self.current_tilt = tilt

            # publish the current PTZ state
            state = PtzState()
            state.pan = pan
            state.tilt = tilt
            state.zoom = 0
            if self.is_moving:
                state.mode = PtzState.MODE_POSITION
            else:
                state.mode = PtzState.MODE_IDLE
            self.status_pub.publish(state)

            if self.is_moving and (\
                (dpan == 0 and dtilt == 0) and ( \
                    abs(self.goal_pan - pan) < self.position_tolerance and \
                    abs(self.goal_tilt - tilt) < self.position_tolerance \
                )):
                self.is_moving = False

        except ValueError as err:
            # the pan and tilt joints are not in the message. this is normal.
            pass
        except Exception as err:
            self.get_logger().error(err)

    def mk_joint_state(self, goal_pan, goal_tilt):
        """Convert the PtzResult to a JointState we can write back to the driver

        @param  goal_pan   The desired pan angle, in radians
        @param  goal_tilt  The desired tilt angle, in radians
        @return a JointState object corresponding to the requested PTZ position
        """
        # convert from the action coordinate range to the hardware coordiante range
        if self.INVERT_PAN:
            goal_pan = -goal_pan
        while goal_pan > self.HW_MAX_PAN:
            goal_pan = goal_pan - 2*pi
        while goal_pan < self.HW_MIN_PAN:
            goal_pan = goal_pan + 2*pi

        if self.INVERT_TILT:
            goal_tilt = -goal_tilt
        while goal_tilt > self.HW_MAX_TILT:
            goal_tilt = goal_tilt - 2*pi
        while goal_tilt < self.HW_MIN_TILT:
            goal_tilt = goal_tilt + 2*pi

        js = JointState()
        js.name = ['ptu_pan', 'ptu_tilt']
        js.velocity = [0.1, 0.1]
        js.position = [goal_pan, goal_tilt]
        js.effort = [0, 0]
        return js

def main(args=None):
    rclpy.init(args=args)
    node = FlirD46PtzControlNode('flir_ptu_action_server_node')
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
