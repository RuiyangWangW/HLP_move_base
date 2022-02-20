# HLP_move_base

## Build
Run `catkin_make`

## Run move_base
This is the key package for our HLP <br />
Run `source ~/HLP_move_base/devel/setup.bash` <br />
`roslaunch planner move_base.launch map_file:=/global_path_to_map_yaml_file.yaml`

## Visualize Path
Run `rosrun rviz rviz`
You can visulize map under topic `/binary_map` and path under topic `/move_base/Global_Planner/plan` 
