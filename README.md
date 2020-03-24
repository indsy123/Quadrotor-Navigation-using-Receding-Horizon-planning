# Quadrotor-Navigation-using-Receding-Horizon-planning
ROS package for quadrotor navigation with receding horizon motion plannning in limited camera FOV as discussed in 
"Reactive Receding Horizon Planning and Control of Quadrotors with Limited On-Board Sensing", IROS 2020 Submission.
Installation Step:
1. Clone the repository in your workspace and build using standard "catkin build". Installing RotorS package from repository "https://github.com/ethz-asl/rotors_simulator.git" is optional and not needed if you are not working with the simulator. Install Open-VINS package from "https://github.com/rpng/open_vins" and install in the same folder. The launch files for this project is provided in folder "ov_msckf_launch". The file "pgeneva_udrone1_ros.launch" was used in the expeirments and is set for intel realsense T265 camera, you need to change camera parameters for a different camera. Similarly, realsense-ros folder is here for completeness. If you are using realsensen d435 camera, please use the official intel repository   
2. This package depends on: 
  a) gurobipy optimizer "https://www.gurobi.com/documentation/8.1/quickstart_mac/the_gurobi_python_interfac.html". The academic   license is free. 
   b) Some python libraries like pyquaternion but you can install them if found missing from your computer using standard pip install command.
   Installation instruction of both the libraries can be found online. 

Runnning the simulations: 
1. Run "roslaunch realsense2_camera rs_t265.launch" and "roslaunch realsense2_camera rs_rgbd.launch". to start cameras for state estimation and pointcloud, this may change as per own setting. 
2. Run "roslaunch ov_msckf pgeneva_udrone1_ros.launch run_number:=XX" to start the state estimator and mavros on the pixhawk, XX could be any run_number you chose for data logging, you may choose the folders for logging. 
3. Run "roslaunch geometric_controller mav_launch_trajectory_camera_fov.launch" to start the motion planning, you need to set the parameters in the launch file along with providing the final position. 

