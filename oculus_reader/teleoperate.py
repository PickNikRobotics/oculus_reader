from oculus_reader.reader import OculusReader
import rclpy
import rclpy.time
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException  # type: ignore[attr-defined]
from geometry_msgs.msg import TransformStamped
import numpy as np

TIP_FRAME_ID = 'grasp_link'


def quaternion_from_matrix(M):
    """Convert a 4x4 homogeneous transform to a quaternion [x, y, z, w]."""
    R = M[:3, :3]
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def invert_homog(T):
    """Inverse of a 4x4 rigid (rotation+translation) transform."""
    inv = np.eye(4)
    R_T = T[:3, :3].T
    inv[:3, :3] = R_T
    inv[:3, 3] = -R_T @ T[:3, 3]
    return inv


def _transform_stamped_to_matrix(ts):
    """Convert a geometry_msgs/TransformStamped to a 4x4 numpy matrix."""
    T = np.eye(4)
    t = ts.transform.translation
    T[:3, 3] = [t.x, t.y, t.z]
    q = ts.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    T[:3, :3] = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])
    return T


class HandClutch:
    """
    Tracks a per-hand reference pose that only moves while a clutch is engaged.

    On clutch press (rising edge) we anchor:
      - the controller pose at the moment of press
      - the reference pose at the moment of press, which is either the current
        tip pose (if provided) or the last reference pose (fallback)
    While engaged, the controller's translation and rotation deltas-since-anchor
    are applied to the reference *in the parent (world) frame*, not in the
    reference's own body frame -- so "controller moves down in world" always
    means "reference moves down in world", regardless of the reference's
    current orientation. Because delta == identity at the moment of engagement,
    the reference does not jump *relative to its anchor*. If a fresh tip pose
    is passed in on each press, the reference re-anchors to the live tip, so
    the robot starts every clutched motion from wherever it currently is --
    even if it has been moved by something else in between presses.
    On clutch release the reference is frozen until the next press.
    """

    def __init__(self, initial_ref=None):
        self.T_ref = np.eye(4) if initial_ref is None else initial_ref.copy()
        self.engaged = False
        self._T_ctrl_anchor = None
        self._T_ref_anchor = None

    def update(self, T_ctrl, clutch_pressed, T_tip_current=None):
        if clutch_pressed and not self.engaged:
            self._T_ctrl_anchor = T_ctrl.copy()
            # Re-anchor the reference to the live tip pose on every press, so
            # the robot picks up from its current pose. Fall back to the last
            # known reference if the tip pose is unavailable this tick.
            anchor = T_tip_current if T_tip_current is not None else self.T_ref
            self._T_ref_anchor = anchor.copy()
            self.T_ref = anchor.copy()
            self.engaged = True
        elif not clutch_pressed and self.engaged:
            self.engaged = False

        if self.engaged and self._T_ctrl_anchor is not None and self._T_ref_anchor is not None:
            # World-frame translation delta (just a vector subtraction)
            dt_world = T_ctrl[:3, 3] - self._T_ctrl_anchor[:3, 3]
            # World-frame rotation delta (left-multiplication of anchor's inverse)
            dR_world = T_ctrl[:3, :3] @ self._T_ctrl_anchor[:3, :3].T

            self.T_ref = np.eye(4)
            self.T_ref[:3, :3] = dR_world @ self._T_ref_anchor[:3, :3]
            self.T_ref[:3, 3] = self._T_ref_anchor[:3, 3] + dt_world

        return self.T_ref


class TeleopNode(Node):
    def __init__(self):
        super().__init__('oculus_teleop')

        # Parent TF frame for all published controller / reference poses.
        # Default 'world' lets the script run standalone in RViz. The launch
        # file overrides this to 'quest_origin' and adds a static TF that
        # aligns the Quest tracking frame to the robot's world frame.
        self.declare_parameter('parent_frame_id', 'world')
        self.parent_frame_id = (
            self.get_parameter('parent_frame_id').get_parameter_value().string_value
        )

        self.oculus_reader = OculusReader()
        self.br = tf2_ros.TransformBroadcaster(self)

        # TF lookup for initializing each reference frame at the robot tip.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self.timer_callback)
        self._streaming_started = False
        self._reference_initialized = False

        # One clutch per hand. Reference poses are set to the tip pose once
        # TF resolves (see _try_initialize_reference). Grip = clutch.
        self.clutch_r = HandClutch()
        self.clutch_l = HandClutch()

        self.get_logger().info(
            f"Publishing under '{self.parent_frame_id}'. "
            f"Waiting for TF '{self.parent_frame_id}' -> '{TIP_FRAME_ID}' "
            f"to initialize references..."
        )

    def timer_callback(self):
        T_tip = self._lookup_tip_pose()

        if not self._reference_initialized:
            if T_tip is not None:
                self.clutch_r.T_ref = T_tip.copy()
                self.clutch_l.T_ref = T_tip.copy()
                self._reference_initialized = True
                self.get_logger().info(
                    f"References initialized at '{TIP_FRAME_ID}' pose. "
                    'Waiting for Quest data... '
                    '(put the headset on if nothing happens)'
                )
            else:
                self.get_logger().info(
                    f"Waiting for TF '{self.parent_frame_id}' -> '{TIP_FRAME_ID}'...",
                    throttle_duration_sec=2.0,
                )
                return  # cannot publish references until they're initialized

        transformations, buttons = self.oculus_reader.get_transformations_and_buttons()
        transformations = transformations or {}
        buttons = buttons or {}

        if transformations and not self._streaming_started:
            parts = []
            if 'r' in transformations:
                parts.append('right')
            if 'l' in transformations:
                parts.append('left')
            detected = ', '.join(parts) if parts else 'none'
            self.get_logger().info(
                f'Streaming started. Controllers detected: {detected}. '
                'Hold the grip button to engage the clutch.'
            )
            self._streaming_started = True

        # _process_hand publishes the reference TF every tick (whether the
        # controller is visible or not), so RViz keeps a stable TF tree.
        # T_tip is passed so that pressing the clutch re-anchors the
        # reference to the current tip pose (no jump from a stale position).
        self._process_hand(
            hand='r', clutch=self.clutch_r, button_key='RG',
            raw_frame='oculus_r', ref_frame='oculus_r_reference',
            transformations=transformations, buttons=buttons, T_tip=T_tip,
        )
        self._process_hand(
            hand='l', clutch=self.clutch_l, button_key='LG',
            raw_frame='oculus_l', ref_frame='oculus_l_reference',
            transformations=transformations, buttons=buttons, T_tip=T_tip,
        )

    def _lookup_tip_pose(self):
        """
        Look up the tip pose in parent_frame_id. Returns a 4x4 numpy matrix on
        success, or None if the TF isn't available this tick (e.g. before TF
        comes up, or a momentary tf2 timeout).
        """
        try:
            ts = self.tf_buffer.lookup_transform(
                self.parent_frame_id,
                TIP_FRAME_ID,
                rclpy.time.Time(),  # latest available
            )
        except TransformException:
            return None
        return _transform_stamped_to_matrix(ts)

    def _process_hand(self, hand, clutch, button_key, raw_frame, ref_frame,
                      transformations, buttons, T_tip):
        """
        Update one hand's clutch (if the controller is visible this tick) and
        publish both the raw and the reference TF. The reference is published
        every tick so RViz keeps showing it even when the controller is lost
        or the clutch is released. T_tip is the latest tip pose looked up
        from TF (or None) -- used to re-anchor the reference on each clutch
        press.
        """
        if hand in transformations:
            T_ctrl = transformations[hand]
            self.publish_transform(T_ctrl, raw_frame)

            clutch_pressed = bool(buttons.get(button_key, False)) if buttons else False
            was_engaged = clutch.engaged
            clutch.update(T_ctrl, clutch_pressed, T_tip_current=T_tip)

            if clutch.engaged and not was_engaged:
                self.get_logger().info(f'{ref_frame}: clutch engaged')
            elif was_engaged and not clutch.engaged:
                self.get_logger().info(f'{ref_frame}: clutch released')

        # Always publish the reference, even when the controller is briefly lost.
        self.publish_transform(clutch.T_ref, ref_frame)

    def publish_transform(self, transform, name):
        translation = transform[:3, 3]
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = name
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        quat = quaternion_from_matrix(transform)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
