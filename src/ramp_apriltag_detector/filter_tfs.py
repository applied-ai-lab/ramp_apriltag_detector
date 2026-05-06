from typing import Any, List, Optional
import copy

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from lti_filters.discrete_filters import DiscreteManualLTI


class SampleValidator:
    """Functional wrapper around the FIR / slew-clamp pipeline.

    Sits between ``lookup_transform`` and ``FilterTf.pre_filter`` and
    decides — per tag, per sample — whether the new measurement is
    plausible enough to feed into the filter at all.  Gates:

    1. **Workspace gate** (hard reject, no fallback).  Anything outside
       ``workspace_min`` / ``workspace_max`` (axis-aligned bounding box,
       in ``parent_frame``) is dropped.  This is the "behind the robot"
       guard.
    2. **Staleness gate** (hard reject).  Samples older than
       ``max_staleness_s`` are dropped — protects against
       ``lookup_transform(Time(0))`` returning a cached pose during
       AprilTag occlusion.
    3. **Translation-jump gate** (reject-with-fallback).  Samples whose
       Euclidean distance from the previous filtered pose exceeds
       ``max_translation_jump`` are dropped, but only for up to
       ``consecutive_reject_limit`` consecutive frames; after that the
       sample is admitted (the downstream clamp slews to it).  This
       balances "occlusion-flip rejection" against "real motion of the
       grasped pin must eventually be tracked".
    4. **Quaternion-flip gate** (reject-with-fallback).  Same fallback
       protocol, but on ``|q_new · q_prev|``.  Catches PnP root
       flips that survive the translation gate.

    The class is stateless w.r.t. the FIR — it only owns the per-tag
    rejection counters.  ``validate`` returns ``True`` to admit and
    ``False`` to drop; the caller is responsible for skipping the FIR
    update and broadcast on a drop.
    """

    def __init__(
        self,
        tf_names: List[str],
        workspace_min: np.ndarray = np.array([0.10, -0.76, 0.78]),
        workspace_max: np.ndarray = np.array([0.78,  0.76, 1.1]),
        max_staleness_s: float = 1.0,
        max_translation_jump: float = 0.15,
        max_quat_flip_dot: float = 0.5,   # cos(60°) — flips are usually 90°+
        consecutive_reject_limit: int = 5,
    ) -> None:
        self.workspace_min = np.asarray(workspace_min, dtype=float)
        self.workspace_max = np.asarray(workspace_max, dtype=float)
        self.max_staleness_s = float(max_staleness_s)
        self.max_translation_jump = float(max_translation_jump)
        self.max_quat_flip_dot = float(max_quat_flip_dot)
        self.consecutive_reject_limit = int(consecutive_reject_limit)
        self._jump_reject_count = {k: 0 for k in tf_names}
        self._quat_reject_count = {k: 0 for k in tf_names}

    def validate(
        self,
        tf_id: str,
        tf_new_stamped: TransformStamped,
        tf_prev_filtered_stamped: TransformStamped,
    ) -> bool:
        t  = tf_new_stamped.transform.translation
        q  = tf_new_stamped.transform.rotation
        pos = np.array([t.x, t.y, t.z])
        q_new = np.array([q.x, q.y, q.z, q.w])

        # 1. Workspace gate — hard reject (no fallback, no counter).
        if np.any(pos < self.workspace_min) or np.any(pos > self.workspace_max):
            rospy.logwarn_throttle(
                2.0,
                f"[filter_tfs] {tf_id}: rejected — outside workspace "
                f"({pos.tolist()})"
            )
            return False

        # 2. Staleness gate — hard reject.
        age = (rospy.Time.now() - tf_new_stamped.header.stamp).to_sec()
        if age > self.max_staleness_s:
            rospy.logwarn_throttle(
                2.0,
                f"[filter_tfs] {tf_id}: rejected — stale ({age:.2f}s old)"
            )
            return False

        # 3. Translation-jump gate — reject with N-frame fallback.
        prev_t = tf_prev_filtered_stamped.transform.translation
        prev_pos = np.array([prev_t.x, prev_t.y, prev_t.z])
        if np.linalg.norm(pos - prev_pos) > self.max_translation_jump:
            self._jump_reject_count[tf_id] += 1
            if self._jump_reject_count[tf_id] < self.consecutive_reject_limit:
                rospy.logwarn_throttle(
                    2.0,
                    f"[filter_tfs] {tf_id}: rejected — jump "
                    f"{np.linalg.norm(pos - prev_pos)*100:.1f} cm "
                    f"(streak {self._jump_reject_count[tf_id]}/"
                    f"{self.consecutive_reject_limit})"
                )
                return False
            # Streak has saturated — admit and let pre_filter clamp it.
        else:
            self._jump_reject_count[tf_id] = 0

        # 4. Quaternion-flip gate — same fallback protocol.
        prev_q_msg = tf_prev_filtered_stamped.transform.rotation
        q_prev = np.array([prev_q_msg.x, prev_q_msg.y, prev_q_msg.z, prev_q_msg.w])
        # |dot| because q and -q encode the same rotation.
        dot = abs(float(np.dot(q_new, q_prev)))
        if dot < self.max_quat_flip_dot:
            self._quat_reject_count[tf_id] += 1
            if self._quat_reject_count[tf_id] < self.consecutive_reject_limit:
                rospy.logwarn_throttle(
                    2.0,
                    f"[filter_tfs] {tf_id}: rejected — quat flip "
                    f"|q·q'|={dot:.2f} "
                    f"(streak {self._quat_reject_count[tf_id]}/"
                    f"{self.consecutive_reject_limit})"
                )
                return False
        else:
            self._quat_reject_count[tf_id] = 0

        return True


class FilterTf:
    def __init__(
        self,
        tf_names: List[str],
        filter: DiscreteManualLTI,
        parent_frame: str = None,
        validator: Optional[SampleValidator] = None,
    ) -> None:
        self._tf_names = tf_names
        self._tf_dict = dict.fromkeys(self._tf_names, None)
        self._tf_prev = dict.fromkeys(self._tf_names, None)
        self._tf_dict_filtered = dict.fromkeys(self._tf_names, None)

        self._max_translation = 0.1

        # Optional sample-rejection wrapper.  When None the legacy
        # clamp-only behaviour is preserved.
        self._validator = validator
        
        assert filter.N == 6, " Filter for tf filter must have a dim of 6 "
        
        filters = list(copy.deepcopy(filter) for _ in range(len(self._tf_names)))
        self._filter_dict = dict((key, value) for key, value in zip(self._tf_names, filters))
        
        self._filter_input = np.zeros(6)

        if parent_frame:
            self._parent_frame = parent_frame
        else:
            self._parent_frame = "base_link"
            
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer)
        self._broadcaster = tf2_ros.TransformBroadcaster()
            
    def __call__(self, *args: Any, **kwds: Any) -> None:
        return self.find_tfs()

    def find_tfs(self) -> None:
        
        # Add rate limiting to not consume 100% of CPU
        rate = rospy.Rate(30) # 30Hz filtering frequency
        while not rospy.is_shutdown():
            # Get the tf transform
            # Side-effect, preserves the last transform if nothing was found.
            for tf_id in self._tf_names:
                try:
                    self._tf_dict[tf_id] = self._tf_buffer.lookup_transform(self._parent_frame, tf_id, rospy.Time(0))   
                except (
                        tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException,
                        ) as e:       
                        continue
                
            # Update the filtered transforms
            for tf_id in self._tf_names:
                # Set the previous tf to current tf if none?
                if self._tf_prev[tf_id] is None:
                    self._tf_prev[tf_id] = self._tf_dict[tf_id]
                
                # Check to see if the filtered transforms are initialised
                if self._tf_dict_filtered[tf_id] is None:
                    if self._tf_dict[tf_id]:
                        # Seed-time gates: only workspace + staleness apply
                        # (no prior pose ⇒ no jump / quat-flip reference).
                        if self._validator is not None and not self._seed_validate(
                                tf_id, self._tf_dict[tf_id]):
                            continue
                        # Create the filtered tf
                        self._tf_dict_filtered[tf_id] = self.create_filtered_tf(self._tf_dict[tf_id])
                        # Initialise the filter
                        self.initialise_filter(tf_id, self._tf_dict[tf_id])
                # If filtered transforms is set
                else:
                    # Reject implausible samples up-front.  When the
                    # validator drops the sample the FIR is left alone
                    # and the previous filtered pose is *not*
                    # re-broadcast — staleness propagates honestly to
                    # downstream consumers.
                    if self._validator is not None and not self._validator.validate(
                            tf_id, self._tf_dict[tf_id], self._tf_dict_filtered[tf_id]):
                        continue
                    # Use the current value of the tf_dict_filtered as the previous tf and the latest
                    # update in tf_dict as the latest update
                    self.pre_filter(self._tf_dict_filtered[tf_id], self._tf_dict[tf_id])
                    # Run second order filter
                    self.advance_filter(tf_id, self._tf_dict_filtered[tf_id], self._tf_dict[tf_id])
                    # Publish the filtered transform
                    self._tf_dict_filtered[tf_id].header.stamp = rospy.Time.now()
                    self._broadcaster.sendTransform(self._tf_dict_filtered[tf_id])

            # Sleep as to not consume 100% of the CPU
            rate.sleep()
        return
    
    def _seed_validate(self, tf_id: str, tf_new_stamped: TransformStamped) -> bool:
        """Workspace + staleness gates only — used at filter-seed time
        when no prior pose exists for jump / quat-flip comparison."""
        v = self._validator
        t = tf_new_stamped.transform.translation
        pos = np.array([t.x, t.y, t.z])
        if np.any(pos < v.workspace_min) or np.any(pos > v.workspace_max):
            rospy.logwarn_throttle(
                2.0,
                f"[filter_tfs] {tf_id}: seed rejected — outside workspace "
                f"({pos.tolist()})"
            )
            return False
        age = (rospy.Time.now() - tf_new_stamped.header.stamp).to_sec()
        if age > v.max_staleness_s:
            rospy.logwarn_throttle(
                2.0,
                f"[filter_tfs] {tf_id}: seed rejected — stale ({age:.2f}s old)"
            )
            return False
        return True

    @staticmethod
    def create_filtered_tf(tf_stamped: TransformStamped) -> TransformStamped:
        tf_stamped_new = TransformStamped()
        tf_stamped_new.header.stamp = tf_stamped.header.stamp
        tf_stamped_new.header.frame_id = tf_stamped.header.frame_id
        tf_stamped_new.child_frame_id = tf_stamped.child_frame_id + "_filtered"

        tf_stamped_new.transform = copy.deepcopy(tf_stamped.transform)
        return tf_stamped_new
    
    @staticmethod
    def tf_stamped_to_trans_quat(tf_stamped: TransformStamped):
        tf = tf_stamped.transform
        trans = tf.translation
        quat = tf.rotation
        return trans, quat
    
    def initialise_filter(self, tf_id: str, tf_new_stamped: TransformStamped):
        trans, _ = self.tf_stamped_to_trans_quat(tf_new_stamped)
        self._filter_input[0:3] = copy.deepcopy(np.array([trans.x, trans.y, trans.z]))
        # Assume the tf is stationary so eta is np.zeros
        self._filter_input[3:] = copy.deepcopy(np.zeros(3))
        # Initialise the filter
        self._filter_dict[tf_id].initialise(iters=1000, x_init=None, u_init=self._filter_input)
        return        
    
    def advance_filter(self, tf_id: str, tf_prev_stamped: TransformStamped, tf_new_stamped: TransformStamped) -> None:
        
        tf_new = tf_new_stamped.transform
        
        self._trans_new = np.array([tf_new.translation.x, tf_new.translation.y, tf_new.translation.z]) 
        self._rot_new = R.from_quat(np.array([tf_new.rotation.x, 
                                              tf_new.rotation.y, 
                                              tf_new.rotation.z, 
                                              tf_new.rotation.w]))
    
        tf_prev = tf_prev_stamped.transform
        self._rot_prev = R.from_quat(np.array([tf_prev.rotation.x, 
                                               tf_prev.rotation.y, 
                                               tf_prev.rotation.z, 
                                               tf_prev.rotation.w]))

        # rot_delta_mat = np.matmul(self._rot_new.as_matrix(), rot_prev.as_matrix().transpose())
        self._filter_input[0:3] = copy.deepcopy(self._trans_new)
        
        filter_arr = copy.deepcopy(self._filter_dict[tf_id].advance(self._filter_input))
        
        self._tf_dict_filtered[tf_id].transform.translation.x = copy.deepcopy(filter_arr[0])
        self._tf_dict_filtered[tf_id].transform.translation.y = copy.deepcopy(filter_arr[1])
        self._tf_dict_filtered[tf_id].transform.translation.z = copy.deepcopy(filter_arr[2])


        # 1. Get Quaternions as numpy arrays
        q_new = self._rot_new.as_quat()
        q_prev = self._rot_prev.as_quat()

        # 2. FIXED: The "Shortest Path" / Sign-Flip Bug
        # Quaternions q and -q represent the same rotation.
        # Slerp can spin 360 degrees if we don't ensure they are in the same hemisphere.
        if np.dot(q_prev, q_new) < 0:
            q_new = -q_new
            # Re-create the rotation object with the flipped sign
            self._rot_new = R.from_quat(q_new)

        # 3. Create Slerp object (Using DCM for SciPy 1.3.3 compatibility)
        slerp = Slerp([0.0, 1.0], R.from_matrix([self._rot_prev.as_matrix(), self._rot_new.as_matrix()]))

        # 4. Interpolate and extract
        # We use [0.3] as a list to satisfy SciPy's 1D array requirement
        filtered_rotation = slerp([0.3])[0] 
        quat_new_filtered = filtered_rotation.as_quat()

        # 5. FIXED: Normalization
        # Over time, math errors can make the quaternion length != 1.0, which ROS hates.
        norm = np.linalg.norm(quat_new_filtered)
        if norm > 0:
            quat_new_filtered /= norm
        else:
            # Fallback to previous if something went catastrophically wrong
            quat_new_filtered = q_prev
        
        self._tf_dict_filtered[tf_id].transform.rotation.x = copy.deepcopy(quat_new_filtered[0])
        self._tf_dict_filtered[tf_id].transform.rotation.y = copy.deepcopy(quat_new_filtered[1])
        self._tf_dict_filtered[tf_id].transform.rotation.z = copy.deepcopy(quat_new_filtered[2])
        self._tf_dict_filtered[tf_id].transform.rotation.w = copy.deepcopy(quat_new_filtered[3])

        return

    def pre_filter(self, tf_prev_stamped: TransformStamped, tf_new_stamped: TransformStamped) -> None:
        
        tf_new = tf_new_stamped.transform
        
        self._trans_new = np.array([tf_new.translation.x, tf_new.translation.y, tf_new.translation.z]) 
        self._rot_new = R.from_quat(np.array([tf_new.rotation.x, 
                                              tf_new.rotation.y, 
                                              tf_new.rotation.z, 
                                              tf_new.rotation.w]))
    
        tf_prev = tf_prev_stamped.transform
        
        self._trans_prev = np.array([tf_prev.translation.x, tf_prev.translation.y, tf_prev.translation.z])
        self._rot_prev = R.from_quat(np.array([tf_prev.rotation.x, 
                                               tf_prev.rotation.y, 
                                               tf_prev.rotation.z, 
                                               tf_prev.rotation.w]))
        
        # Update the current transform to reduce disturbances
        # --- Translation limiting ---
        dp_vec = self._trans_new - self._trans_prev
        dp_norm = np.linalg.norm(dp_vec)

        if dp_norm > self._max_translation:
            dp_vec = dp_vec / dp_norm * self._max_translation
        self._trans_new = self._trans_prev + dp_vec
        
        # Update the transform
        tf_new_stamped.transform.translation.x = self._trans_new[0]
        tf_new_stamped.transform.translation.y = self._trans_new[1]
        tf_new_stamped.transform.translation.z = self._trans_new[2]
        
        # Do orientation 
        return
