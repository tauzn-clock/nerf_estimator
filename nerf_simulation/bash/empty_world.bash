roslaunch nerf_simulation empty_world.launch &
sleep 5
roslaunch jackal_gazebo spawn_jackal.launch x:=2 y:=2 joystick:=false config:=front_laser&
sleep 5
roslaunch jackal_navigation odom_navigation_demo.launch &
sleep 5
roslaunch nerf_simulation camera_rviz.launch