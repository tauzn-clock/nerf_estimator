roslaunch nerf_simulation empty_world.launch &
sleep 5
roslaunch jackal_gazebo spawn_jackal.launch x:=2 y:=2 &
sleep 5
roslaunch nerf_simulation camera_rviz.launch