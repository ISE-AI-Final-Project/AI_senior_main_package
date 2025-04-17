#!/bin/bash

gnome-terminal \
  --tab --title="URSim + RViz" -e "bash -c 'source /home/icetenny/ros2_ws/install/local_setup.bash; \
                                           source /home/icetenny/ws_ur/install/local_setup.bash; \
                                           source /home/icetenny/ws_moveit/install/local_setup.bash; \
                                           ros2 launch my_robot_cell_control ursim_bringup.launch.py rviz_config:=main.rviz; \
                                           exec bash'" \
  --tab --title="ZED Camera" -e "bash -c 'source /home/icetenny/ros2_ws/install/local_setup.bash; \
                                          ros2 launch zed_wrapper zed_camera.launch.py publish_tf:=false camera_model:=zed2i; \
                                          exec bash'" \
  --tab --title="Main Node" -e "bash -c 'source /home/icetenny/ros2_ws/install/local_setup.bash; \
                                         source /home/icetenny/ws_ur/install/local_setup.bash; \
                                         source /home/icetenny/ws_moveit/install/local_setup.bash; \
                                         ros2 launch main_pkg main.launch.py; \
                                         exec bash'" \
  --tab --title="ISM Server" -e "bash -c 'source ~/anaconda3/etc/profile.d/conda.sh; \
                                          conda activate sam6d; \
                                          cd /home/icetenny/senior-1/SAM-6D/SAM-6D; \
                                          python3 ism_server.py; \
                                          exec bash'" \
  --tab --title="PEM Server" -e "bash -c 'source ~/anaconda3/etc/profile.d/conda.sh; \
                                          conda activate sam6d; \
                                          cd /home/icetenny/senior-1/SAM-6D/SAM-6D; \
                                          python3 pem_server.py; \
                                          exec bash'"
