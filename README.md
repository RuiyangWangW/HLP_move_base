# HLP_move_base

## Build
Run `catkin_make`

## Publish Binary Map
Run `rosrun map_server map_server ~/HLP_move_base/src/planner/maps/map.yaml /map:=/binary_map`

## Run move_base
This is the key package for our HLP
Run `source ~/HLP_move_base/devel/setup.bash`
`roslaunch planner move_base.launch`

## Visualize Path
Run `rosrun rviz rviz`
You can visulize map under topic `/binary_map`
and path under topic `/move_base/Global_Planner/plan`
