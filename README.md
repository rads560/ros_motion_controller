# ros_motion_controller
This is a program that allows a robot to be commanded to poses from two nodes: the gui node and the clock node. 
The clock node sends positions to the robot based on the minute hand of the local time.
The gui node sends positions to the robot based on where the user clicks on a GUI representing a clock with one hand.
Note that the user must choose positions towards the outer edge of the clock in the GUI for the position to be updated.

The robot is simulated using a turtle bot. There is a motion controller with some simple motion control logic to go from the 
last pose to the next pose. However, this is not currently working right and will require more time to debug. Instead, one can
simulate the turtle bot using the teleport mode instead and see the turtle jump to the target position that we are commanding it to.

## Prerequisites
- make sure docker is installed! see https://docs.docker.com/get-started/get-docker/
- alternatively, ROS 2 Humble is installed
- install and set up an x server such as XQuartz

## To Run
Clone this repo and cd into it.
```
docker build -t my/ros:ros-motion-controller .
docker run -it -v $(pwd):/Workspace -e DISPLAY=host.docker.internal:0 -v /tmp/.X11-unix:/tmp/.X11-unix --rm my/ros:ros-motion-controller
```

Once in the docker container, run:
```
source /opt/ros/humble/setup.bash
cd Workspace
apt update
rosdep install -i --from-path src --rosdistro humble -y
source /opt/ros/humble/setup.bash
colcon build
```

Make a new terminal and get back into the same docker container using `docker ps` and `docker exec -it <tag> /bin/bash`
Then run:
```
source /opt/ros/humble/setup.bash
cd Workspace
source install/local_setup.bash
_insert launch command here_
```

## Launch commands
```
ros2 launch launch_manager launch_headless.xml
ros2 launch launch_manager launch_simulated_teleport.xml
ros2 launch launch_manager launch_simulated.xml
```

- The launch_headless file will just launch without any GUIs and allow a user to see the log output.
- The launch_simulated_teleport file will launch with a turtle bot sim and the teleport mode.
- The launch_simulated file will launch a WIP motion controller that isn't currently tuned. However, if you would like to sub that out 
for your own motion controller, running with that file should be the right way to go!

## How it works
There are four main nodes in this program: clock_node, gui_node, pose_manager_node, and motion_ctl_node. Both the clock_node and the gui_node handle logic for deciding which pose to send and then sending those poses to the pose_manager. The pose_manager handles the logic for determining which pose to send through to the motion controller. Namely, if it has been 30 seconds since the last gui pose, the pose manager will return to sending clock poses which are published every 1 second. In addition, if the user chooses a gui pose and then presses the space bar, even if the full 30 seconds has not completed, the pose manager will switch back to clock following.

Target poses are published to a topic: `target_pose` 

To swap out the motion controller, simply add a node that subscribes to `target_pose` and add it to the launch file in place of the motion_ctl package.

## Future Improvements
I didn't have time for everything I wanted to do, so here are some things I'd like to add if given more time:
- set the rotation of the pose, not just position
- use actions to abort GUI commands rather than an empty message
- tune the motion controller
- add unit tests

## Hope you enjoy! 
Feel free to contact Radhika Agrawal for any questions/concerns. :) 
