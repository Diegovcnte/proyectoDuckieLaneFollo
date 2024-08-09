#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
dt-exec roslaunch led_emitter led_emitter_node.launch veh:=jefazo
#dt-exec roslaunch apriltag apriltag.launch veh:=jefazo
dt-exec roslaunch lane_follow lane_follow_node.launch veh:=jefazo
rosrun lane_follow lane_following.py
# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
