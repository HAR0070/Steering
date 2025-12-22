# Steering
Testing steering
The steering angle is converted to velocity commands through PID 
Velocity is command is send to arduino -> CAN -> AK80-8 motor controller 

The feedback and commands are published as ROS1 topics 

Once overtaken -- or while not commanded -- the CAN command to steer is stoped -- we dont command 0 position

# Testing 
Collect rosbag using rqt_bag 
check topics which are published using rqt 
the omodetry_ecef gives orientations as quaternions 

# Braking
The braking folder contains arduino code to read braking value
this code is directly applied to the braking pot of the vehicle

Regen unit testing
If the wiring to brake pot is not is not correct - the throttle and brake will give fault 
To detect the fault - understand which signal are coming from where -- look at the signal connection diagrams in manual

Unit Test data
Regen braking data is available in here - LINK
Manual braking data is available in here - LINK

Fine Tuning braking 
Curtis controller 
