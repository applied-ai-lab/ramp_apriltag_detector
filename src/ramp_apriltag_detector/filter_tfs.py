from typing import Any, List
import copy

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from lti_filters.discrete_filters import DiscreteManualLTI


class FilterTf:
    def __init__(self, tf_names: List[str], filter: DiscreteManualLTI, parent_frame: str=None) -> None:
        self._tf_names = tf_names
        self._tf_dict = dict.fromkeys(self._tf_names, None)
        self._tf_prev = dict.fromkeys(self._tf_names, None)
        self._tf_dict_filtered = dict.fromkeys(self._tf_names, None)
        
        self._max_translation = 0.1
        
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
                        # Create the filtered tf 
                        self._tf_dict_filtered[tf_id] = self.create_filtered_tf(self._tf_dict[tf_id])
                        # Initialise the filter
                        self.initialise_filter(tf_id, self._tf_dict[tf_id])
                # If filtered transforms is set
                else:
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

        # rot_delta_mat = np.matmul(self._rot_new.as_dcm(), rot_prev.as_dcm().transpose())
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
        slerp = Slerp([0.0, 1.0], R.from_dcm([self._rot_prev.as_dcm(), self._rot_new.as_dcm()]))

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
