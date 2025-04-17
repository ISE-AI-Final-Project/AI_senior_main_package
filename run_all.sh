#!/bin/bash

# Load Conda for terminals that need it
conda_init="source ~/anaconda3/etc/profile.d/conda.sh"

# Terminal 1: URSim + RViz
gnome-terminal -- bash -c "
source /home/icetenny/ros2_ws/install/local_setup.bash && \
source /home/icetenny/ws_ur/install/local_setup.bash && \
source /home/icetenny/ws_moveit/install/local_setup.bash && \
ros2 launch my_robot_cell_control realur3e_bringup.launch.py rviz_config:=main.rviz; exec bash"
sleep 8

# Terminal 2: ZED Camera
gnome-terminal -- bash -c "
source /home/icetenny/ros2_ws/install/local_setup.bash && \
ros2 launch zed_wrapper my_zed_camera.launch.py publish_tf:=false camera_model:=zed2i_gpu1; exec bash"


# Terminal 3: ISM Server
gnome-terminal -- bash -c "
$conda_init && \
conda activate sam6d && \
cd /home/icetenny/senior-1/SAM-6D/SAM-6D && \
python3 ism_server.py; exec bash"

# Terminal 4: PEM Server
gnome-terminal -- bash -c "
$conda_init && \
conda activate sam6d && \
cd /home/icetenny/senior-1/SAM-6D/SAM-6D && \
python3 pem_server.py; exec bash"
sleep 5

# Terminal 5: Main Node
gnome-terminal -- bash -c "
source /home/icetenny/ros2_ws/install/local_setup.bash && \
source /home/icetenny/ws_ur/install/local_setup.bash && \
source /home/icetenny/ws_moveit/install/local_setup.bash && \
ros2 launch main_pkg main.launch.py; exec bash"