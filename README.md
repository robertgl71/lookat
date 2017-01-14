# lookat(percepció)
1. $ cd ~/catkin_ws/src
2. $ catkin_create_pkg lookat std_msgs geometry_msgs rospy roscpp
3. Cloneu el repo dins el nou package
4. $ cd ~/catkin_ws
5. $ catkin_make
6. $ roscore
7. $ roslaunch lookat lookat.launch
8. $ rosrun lookat point_listener
