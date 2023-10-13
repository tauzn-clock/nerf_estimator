export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-01
roslaunch nerf_simulation empty_world.launch &
sleep 5
roslaunch nerf_simulation spawn_turtle.launch &
sleep 5
roslaunch turtlebot3_bringup turtlebot3_remote.launch &
sleep 5
roslaunch turtlebot3_navigation amcl.launch &
sleep 5
roslaunch nerf_simulation odom_navigation_turtle.launch &
sleep 5
roslaunch nerf_simulation camera_rviz.launch