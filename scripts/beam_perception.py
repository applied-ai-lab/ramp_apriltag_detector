#!/usr/bin/env python3

import rospy

from ramp_apriltag_detector.server import (Beamtracker, listen_to)


if __name__ == "__main__":
    rospy.init_node("beam_perception")
    
    beam_tracker = Beamtracker()
    listen_to(beam_tracker)