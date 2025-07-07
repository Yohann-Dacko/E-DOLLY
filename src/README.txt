ros2 launch e_dollyv2 launch_sim.launch.py world:=./src/e_dollyv2/worlds/obstacles.world
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/e_dollyv2/config/mapper_params_online_async.yaml use_sim_time:=true
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=src/e_dollyv2/config/nav2_params.yaml

ros2 run ball_tracker detect_ball --ros-args --remap image_in:=camera/image_raw
