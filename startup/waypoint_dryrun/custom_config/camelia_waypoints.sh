#!/bin/bash

rostopic pub /red/waypoints uav_ros_msgs/Waypoints "waypoints:
- pose:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: 'eagle/carto_raw'
    pose:
      position:
        x: -4.3
        y: -7.5
        z: 1.0
      orientation:
        x: 0.0
        y: 0.0
        z: 1.0
        w: 0.0
  waiting_time: 10.0
- pose:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: 'eagle/carto_raw'
    pose:
      position:
        x: 1.3
        y: 3.0
        z: 1.0
      orientation:
        x: 0.0
        y: 0.0
        z: 1.0
        w: 0.0
  waiting_time: 10.0
- pose:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: 'eagle/carto_raw'
    pose:
      position:
        x: 1.3
        y: 4.0
        z: 1.0
      orientation:
        x: 0.0
        y: 0.0
        z: 1.0
        w: 0.0
  waiting_time: 10.0
- pose:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: 'eagle/carto_raw'
    pose:
      position:
        x: 2.3
        y: 4.0
        z: 1.0
      orientation:
        x: 0.0
        y: 0.0
        z: 1.0
        w: 0.0
  waiting_time: 10.0
- pose:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: 'eagle/carto_raw'
    pose:
      position:
        x: 2.3
        y: 3.0
        z: 1.0
      orientation:
        x: 0.0
        y: 0.0
        z: 1.0
        w: 0.0
  waiting_time: 10.0
- pose:
    header:
      seq: 0
      stamp:
        secs: 0
        nsecs: 0
      frame_id: 'eagle/carto_raw'
    pose:
      position:
        x: 1.3
        y: 4.0
        z: 1.0
      orientation:
        x: 0.0
        y: 0.0
        z: 1.0
        w: 0.0
