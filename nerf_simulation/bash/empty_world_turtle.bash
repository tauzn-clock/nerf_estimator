# Get the relative path of the map file
map_location="$(realpath "$(dirname "${BASH_SOURCE[0]}")")/../maps"
echo $map_location
# Set the TURTLEBOT3_MODEL environment variable to burger
export TURTLEBOT3_MODEL=burger
# Set the LDS_MODEL environment variable to LDS-01
export LDS_MODEL=LDS-01


roslaunch nerf_simulation empty_world.launch &
sleep 5
roslaunch nerf_simulation spawn_turtle.launch &
sleep 5
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$map_location/map.yaml &
sleep 5
roslaunch nerf_simulation rviz_turtle.launch