ros2 run mujoco_sim2sim mujoco_sim_viewer_frontend.py --ros-args   -p model_path:="/home/edify/Code/jingchu01/JC01-7DOF-URDF/JC01-URDF-18所/scene_jingchu01.xml"   -p enable_viewer:=true   -p viewer_fps:=100.0

ros2 run mujoco_sim2sim mujoco_sim_bridge --ros-args   --params-file /home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/config/jc01_amp_full_body_sim2sim.yaml   -p rl_cfg_path:=/home/edify/Code/jc01_deploy/src/humanoid_rl_controller/rl_master/config/rl_cfg_amp_mjlab_jc01_full_body.yaml   -p startup_mode_id:=0   -p enable_viewer:=false   -p enable_python_viewer_stream:=true   -p enable_python_viewer_inspector:=true -p fixed_base_height:=0.872 

ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 "{data: 1000}"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib:$LD_LIBRARY_PATH

sudo bash -c '
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
./install/rl_master/lib/rl_master/RL_solver --mode-id 0
'

sudo bash -c '
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
./install/imu_communication_yesense/lib/imu_communication_yesense/imu_communication_yesense
'