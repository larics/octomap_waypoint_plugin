# SITL Obstacle Avoidance

Make sure Ardupilot v4.1 is built in order for ```GUID_OPTIONS``` parameter to have effect.

## Sending Mission

1) ```./start.sh``` - Start the simulation
2) Use the 2D nav goal in the open RVIZ window to select mission points.
2) ```rosservice call /red/rviz_clicker/publish_mission "{}"``` - Start the mission


