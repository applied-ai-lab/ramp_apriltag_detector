# RAMP April Detector

Authors [Alexander Mitchell](https://github.com/mitch722) and [Ben Kaye](https://github.com/ben-kaye) (2024)

## 0. Overview

This package detects the RAMP april tags, filters and combines the two tags into one when they are visible.

## 1. Install

The dependencies for this package are [lti_filters](https://github.com/applied-ai-lab/lti_filters) and april_tag_ros.

## 2. Running

There are currently three launch files. The first filters the tf frames published by the april tag reader. The second detects the april tags and publishes the pose. The third one runs the first two and publishes estimates of the beam poses.

```bash
$ roslaunch ramp_apriltag_detector beam_perception.launch # This runs both the april tag filter, april tag detector and beam pose estimator.
```
The standalne filter is here:

```bash
$ roslaunch ramp_apriltag_detector tf_filter.launch
```
