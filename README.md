# P3_UVic

1. roslaunch ur_gazebo ur5.launch limited:=true

2. roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true

3. roslaunch ur5_moveit_config moveit_rviz.launch config:=true

4. rosrun ur5_move_group ur5_move_group_node
