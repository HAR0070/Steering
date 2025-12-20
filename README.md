# Steering
Testing steering
The steering angle is converted to velocity commands through PID 
Velocity is command is send to arduino -> CAN -> AK80-8 motor controller 

The feedback and commands are published as ROS1 topics 

# Testing 
Collect rosbag using rqt_bag 
check topics which are published using rqt 
the omodetry_ecef gives orientations as quaternions 

# Braking
The braking folder contains arduino code to read braking value
this code is directly applied to the braking pot of the vehicle

Regen unit testing
