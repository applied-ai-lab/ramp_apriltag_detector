#!/usr/bin/env python3

import rospy

from lti_filters.discrete_filters import (DiscreteManualLTI, LTIBaseParams)

from ramp_apriltag_detector.filter_tfs import FilterTf
from ramp_apriltag_detector.server import Beamtracker


if __name__ == "__main__":
    rospy.init_node("filter_tfs")
    
    # Get beam names
    beam_names = Beamtracker().beam_names
    
    # Create an instance of a filter
    # Filter dim must be 6
    dim = 6
    # Sampling frequency 
    dt = 1.0 / 30.0 # Seconds
    damping_ratio = 0.8 # Slightly under damped with no ringing
    rise_time = 2.0 # Seconds
    
    params = LTIBaseParams(rise_time, damping_ratio)
    filter = DiscreteManualLTI(dim, params, dt)

    parent_frame = "camera_link"
    
    # Create the tf filter
    tf_filter = FilterTf(beam_names, filter, parent_frame)
    
    # Filter tfs
    tf_filter()
