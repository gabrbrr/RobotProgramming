source /opt/ros/noetic/setup.bash
source devel/setup.bash
export TURTLEBOT3_MODEL=burger

gnome-terminal -- bash -c "roscore; exec bash"
sleep 5  

gnome-terminal -- bash -c "roslaunch dmap_localizer dmap_localizer.launch; exec bash"
gnome-terminal -- bash -c "rviz; exec bash"  
gnome-terminal -- bash -c "roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch; exec bash"