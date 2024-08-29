from typing import Any, List
import copy

import numpy as np
from pyquaternion import Quaternion as pyquat
from liegroups import SO3
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
        self._tf_dict_filtered = dict.fromkeys(self._tf_names, None)
        
        assert filter.N == 6, " Filter for tf filter must have a dim of 6 "
        
        filters = list(copy.deepcopy(filter) for _ in range(len(self._tf_names)))
        self._filter_dict = dict((key, value) for key, value in zip(self._tf_names, filters))
        
        self._filter_input = np.zeros(6)

        if parent_frame:
            self._parent_frame = parent_frame
        else:
            self._parent_frame = "base_link"
            
    def __call__(self, *args: Any, **kwds: Any) -> None:
        return self.find_tfs()

    def find_tfs(self) -> None:
        
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        broadcaster = tf2_ros.TransformBroadcaster()
        
        while not rospy.is_shutdown():
            rate = rospy.Rate(30.0)
            try:
                # Get the tf transform
                for tf_id in self._tf_names:
                    self._tf_dict[tf_id] = tf_buffer.lookup_transform(self._parent_frame, tf_id, rospy.Time(0))          
                # Update the filtered transforms
                for tf_id in self._tf_names:
                    # Check to see if the filtered transforms are initialised
                    if self._tf_dict_filtered[tf_id] is None:
                        # Create the filtered tf 
                        self._tf_dict_filtered[tf_id] = self.create_filtered_tf(self._tf_dict[tf_id])
                        # Initialise the filter
                        self.initialise_filter(tf_id, self._tf_dict[tf_id])
                    # If filtered transforms is set
                    else:
                        # Use the current value of the tf_dict_filtered as the previous tf and the latest
                        # update in tf_dict as the latest update
                        self.advance_filter(tf_id, self._tf_dict_filtered[tf_id], self._tf_dict[tf_id])

                    # Publish the filtered transform
                    self._tf_dict_filtered[tf_id].header.stamp = rospy.Time.now()
                    broadcaster.sendTransform(self._tf_dict_filtered[tf_id])
                    
            
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
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
        
        trans_new = np.array([tf_new.translation.x, tf_new.translation.y, tf_new.translation.z]) 
        rot_new = R.from_quat(np.array([tf_new.rotation.x, tf_new.rotation.y, tf_new.rotation.z, tf_new.rotation.w]))
    
        tf_prev = tf_prev_stamped.transform
        rot_prev = R.from_quat(np.array([tf_prev.rotation.x, tf_prev.rotation.y, tf_prev.rotation.z, tf_prev.rotation.w]))

        rot_delta_mat = np.matmul(rot_new.as_matrix(), rot_prev.as_matrix().transpose())

        self._filter_input[0:3] = copy.deepcopy(trans_new)
        
        filter_arr = copy.deepcopy(self._filter_dict[tf_id].advance(self._filter_input))
        
        self._tf_dict_filtered[tf_id].transform.translation.x = copy.deepcopy(filter_arr[0])
        self._tf_dict_filtered[tf_id].transform.translation.y = copy.deepcopy(filter_arr[1])
        self._tf_dict_filtered[tf_id].transform.translation.z = copy.deepcopy(filter_arr[2])

        slerp = Slerp([0.0, 1.0], R.from_matrix([rot_prev.as_matrix(), rot_new.as_matrix()]))

        filtered_rotation = slerp(0.1)
        quat_new_filtered = filtered_rotation.as_quat()
        
        self._tf_dict_filtered[tf_id].transform.rotation.x = copy.deepcopy(quat_new_filtered[0])
        self._tf_dict_filtered[tf_id].transform.rotation.y = copy.deepcopy(quat_new_filtered[1])
        self._tf_dict_filtered[tf_id].transform.rotation.z = copy.deepcopy(quat_new_filtered[2])
        self._tf_dict_filtered[tf_id].transform.rotation.w = copy.deepcopy(quat_new_filtered[3])

        return

        
        
                
    
