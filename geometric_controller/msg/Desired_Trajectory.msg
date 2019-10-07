# The custom msg file created by isy on 10/11/2017
# it gives the desired trajectory for the controller
# desired_position is desired positon (x,y,z)
# desired_velocity is the linear velocity (x,y,z)
# desired_acceleration is the linear acceleration (x,y,z)
# desired_direction is the direction vector (x,y,z)
Header header
geometry_msgs/Point pdes
geometry_msgs/Point vdes
geometry_msgs/Point ades
geometry_msgs/Point jdes
geometry_msgs/Point ddes
geometry_msgs/Point ddes_dot
geometry_msgs/Point ddes_ddot
int32 controller
