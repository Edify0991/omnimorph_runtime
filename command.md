export ROS_LOG_DIR=/tmp/jc01_ros_logs
mkdir -p "$ROS_LOG_DIR"

ros2 run mujoco_sim2sim mujoco_sim_bridge --ros-args \
  --params-file /home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/config/jc01_amp_full_body_sim2sim.yaml \
  -p rl_cfg_path:=/home/edify/Code/jc01_deploy/src/humanoid_rl_controller/rl_master/config/rl_cfg_jc01.yaml \
  -p startup_mode_id:=0 \
  -p enable_viewer:=false \
  -p enable_python_viewer_stream:=true \
  -p enable_python_viewer_inspector:=true

ros2 run mujoco_sim2sim mujoco_sim_viewer_frontend.py --ros-args \
  -p model_path:="/home/edify/Code/jingchu01/JC01-7DOF-URDF/JC01-URDF-18所/scene_jingchu01.xml" \
  -p enable_viewer:=true \
  -p viewer_fps:=100.0

ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 "{data: 1000}"
