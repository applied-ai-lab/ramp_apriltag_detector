#!/usr/bin/env python3

import rospy

from ramp_apriltag_detector.server import (Beamtracker, listen_to)


if __name__ == "__main__":
    rospy.init_node("beam_perception")
    
    # Get the mode from the ros params
    mode = rospy.get_param('mode')

    beam_tracker = Beamtracker()
    listen_to(beam_tracker, mode=mode)
