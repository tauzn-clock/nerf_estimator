export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-01
roslaunch nerf_simulation empty_world.launch &
sleep 5
roslaunch turtlebot3_bringup turtlebot3_robot.launch &
sleep 5
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml