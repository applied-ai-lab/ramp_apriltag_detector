from typing import Any, list
import copy

import numpy as np
from pyquaternion import Quaternion 
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from lti_filters.discrete_filters import DiscreteManualLTI


class FilterTf:
    def __init__(self, tf_names: list[str], filter: DiscreteManualLTI, parent_frame: str = None) -> None:
        self._tf_names = tf_names
        self._tf_dict = dict.fromkeys(self._tf_names, None)
        self._tf_dict_filtered = dict.fromkeys(self._tf_names, None)
        
        assert filter.N == 7, " Filter for tf filter must have a dim of 7 "
        
        filters = list(copy.deepcopy(filter) for _ in range(len(self._tf_names)))
        self._filter_dict = dict((key, value) for key, value in zip(self._tf_names, filters))
        
        self._temp_arr = np.zeros(7)
        
        if parent_frame:
            self._parent_frame = parent_frame
        else:
            self._parent_frame = "base_link"

    def find_tfs(self) -> None:
        
        tf_buffer = tf2_ros.Buffer()
        
        while not rospy.is_shutdown():
            rate = rospy.Rate(30.0)
            try:
                for tf_id in self._tf_names:
                    self._tf_dict[tf_id] = tf_buffer.lookup_transform(self._parent_frame, tf_id, rospy.Time(0))                  
                    
            
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                rate.sleep()
        return
    
    
    def initialise_filter(self, tf_id: str, tf_new_stamped: TransformStamped):
        pass
    
    def advance_filter(self, tf_id: str, tf_prev_stamped: TransformStamped, tf_new_stamped: TransformStamped):
        
        tf_new = tf_new_stamped.transform
        
        trans_new = np.array([tf_new.translation.x, tf_new.translation.y, tf_new.translation.z]) 
        rot_new = Quaternion(tf_new.rotation.w, tf_new.rotation.x, tf_new.y, tf_new.z)
    
        tf_prev = tf_prev_stamped.transform
        rot_prev = Quaternion(tf_prev.rotation.w, tf_prev.rotation.x, tf_prev.y, tf_prev.z)
            
        eta = Quaternion.log_map(rot_prev, rot_new)
        
        self._temp_arr[0:3] = copy.deepcopy(trans_new)
        self._temp_arr[3:] = copy.deepcopy(np.array([eta.w, eta.x, eta.y, eta.z]))
        
        filter_arr = self._filter_dict[tf_id].advance(self._temp_arr)
        
        self._tf_dict_filtered[tf_id].translation.x = filter_arr[0]
        self._tf_dict_filtered[tf_id].translation.y = filter_arr[1]
        self._tf_dict_filtered[tf_id].translation.z = filter_arr[2]
        
        eta_filtered = Quaternion(filter_arr[3], filter_arr[4], filter_arr[5], filter_arr[6])
        quat_filtered = Quaternion.exp_map(rot_prev, eta_filtered).normalised
        
        self._tf_dict_filtered[tf_id].rotation.w = quat_filtered.w
        self._tf_dict_filtered[tf_id].rotation.x = quat_filtered.x
        self._tf_dict_filtered[tf_id].rotation.y = quat_filtered.y
        self._tf_dict_filtered[tf_id].rotation.z = quat_filtered.z
        
        return        
        


        
        
                
    
