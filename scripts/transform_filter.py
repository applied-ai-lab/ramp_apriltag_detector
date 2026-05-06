#!/usr/bin/env python3

import numpy as np
import rospy

from lti_filters.discrete_filters import (DiscreteManualLTI, LTIBaseParams)

from ramp_apriltag_detector.filter_tfs import FilterTf, SampleValidator
from ramp_apriltag_detector.server import Beamtracker
from ramp_apriltag_detector.FIR_filter import LowPassFilterCoefficients, DiscreteFIRFilter, MediumPassFilterCoefficients


def extract_tag_names(tag_list):
    tag_names = []

    for tag_data in tag_list:
        tag_names.append(tag_data['name'])
    return tag_names


if __name__ == "__main__":
    rospy.init_node("filter_tfs")
    
    # Get beam names
    beam_names = Beamtracker().beam_names

    standalone_tags = rospy.get_param('/apriltag_ros_continuous_node/standalone_tags')
    tag_names = extract_tag_names(standalone_tags)

    tf_to_filter = beam_names + tag_names

    # Create an instance of a filter
    # Filter dim must be 6
    dim = 6
    # Sampling frequency 
    dt = 1.0 / 30.0 # Seconds
    damping_ratio = 1.0 #CHANGED 2.0->1.0 Slightly under damped with no ringing
    rise_time = 0.4 # CHANGED 0.8-> 0.4 Seconds
    
    # params = LTIBaseParams(rise_time, damping_ratio)
    # filter = DiscreteManualLTI(dim, params, dt)
    filter_coefs = MediumPassFilterCoefficients()
    filter = DiscreteFIRFilter(dim,filter_coefs)

    # parent_frame = "camera_link"
    parent_frame = "base_link"
    
    # Sample-rejection wrapper.  Anything outside the workspace AABB,
    # older than ``max_staleness_s``, jumping more than 15 cm, or
    # quat-flipped is dropped — with a 5-frame consecutive-rejection
    # streak before the jump / quat-flip gates fall back to the legacy
    # clamp behaviour.  Workspace bounds are in ``parent_frame``
    # (base_link) and intentionally generous; tighten per deployment.
    workspace_min = np.array(rospy.get_param("~workspace_min", [0.10, -0.80, 0.50]))
    workspace_max = np.array(rospy.get_param("~workspace_max", [1.20,  0.80, 1.60]))
    validator = SampleValidator(
        tf_names=tf_to_filter,
        workspace_min=workspace_min,
        workspace_max=workspace_max,
        max_staleness_s=rospy.get_param("~max_staleness_s", 0.5),
        max_translation_jump=rospy.get_param("~max_translation_jump", 0.15),
        max_quat_flip_dot=rospy.get_param("~max_quat_flip_dot", 0.5),
        consecutive_reject_limit=rospy.get_param("~consecutive_reject_limit", 5),
    )

    # Create the tf filter
    tf_filter = FilterTf(tf_to_filter, filter, parent_frame, validator=validator)
    
    # Filter tfs
    tf_filter()
