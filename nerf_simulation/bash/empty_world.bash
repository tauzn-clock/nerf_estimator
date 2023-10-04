roslaunch nerf_simulation empty_world.launch
sleep 5
roslaunch jackal_gazebo spawn_jackal.launch :x:=0.5 :y:=0.5
sleep 5
roslaunch jackal_viz view_robot.launch