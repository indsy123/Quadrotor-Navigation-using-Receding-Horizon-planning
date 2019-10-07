# this msg publishes the polynomial coeffcients in x, y and z direction along with some booleans for trajectory planning 
Header header
int32 polynomial_order	
float32[] poly_x
float32[] poly_y
float32[] poly_z
float32[] segment_end_times
geometry_msgs/Point desired_direction
int32 number_of_segments
float32 planning_start_time
string  trajectory_mode 

 

