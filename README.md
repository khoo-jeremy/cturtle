# cturtle

Group Number: 24

---
Commands to run code on gazebo:
```
roslaunch mie443_contest2 turtlebot_world.launch world:=2
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/<absolute_path_to_catkin_ws>/catkin_ws/src/cturtle/mie443_contest2/maps/map_2.yaml
roslaunch turtlebot_rviz_launchers view_navigation.launch  

//localize the turtlebot by performing the 2D pose estimate on RVIZ first, then run:
rosrun mie443_contest2 contest2
```
---
The output file is found at:
```
catkin_ws/contest2.txt                                                                 
```
