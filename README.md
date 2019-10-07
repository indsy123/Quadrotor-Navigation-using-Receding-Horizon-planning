# Quadrotor-Navigation-using-Receding-Horizon-planning
ROS package for quadrotor navigation with receding horizon motion plannning in limited camera FOV as discussed in 
"Receding Horizon Control of Quadrotors with Limited On-Board Sensing", ICRA 2020 Submission.
Installation Step:
1. Install RotorS package from repository "https://github.com/ethz-asl/rotors_simulator.git" and replace the folder titled "rotors_gazebo" from this or just add the launch files and models from that folder. This is needed to get the necessary models for simulations. 
2. This package depends on: 
  a) gurobipy optimizer "https://www.gurobi.com/documentation/8.1/quickstart_mac/the_gurobi_python_interfac.html". The academic   license is free. 
   b) Python libraries of Gudhi "http://gudhi.gforge.inria.fr/introduction/". These are required to construct the VR complex in the camera's FOV
   Installation instruction of both the libraries can be found at the above mentioned websites. 
3. close the other two folders in your catkin_ws and build using "catkin build" command. 

Runnning the simulations: 
1. Run "roslaunch rotors_gazebo single_firefly.launch world:=multi_object2.world". 
2. Run "roslaunch geometric_controller single_mav.launch" to start the simulations. 

The world files that can be used: a) multi_object.world b) world_for_experiment.world c) world_for_experiment2.world d) oak_tree_world.world
The initial position of the quadrotor can be set in "single_firefly.launch" while the other parameters for simulation can be set in "single_mav.launch"
