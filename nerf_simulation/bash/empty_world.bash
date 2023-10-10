roslaunch nerf_simulation empty_world.launch &
sleep 5
roslaunch jackal_gazebo spawn_jackal.launch x:=2 y:=2 &
sleep 5
roslaunch jackal_navigation odom_navigation_demo.launch &
sleep 5
roslaunch jackal_viz view_robot.launch config:=navigation