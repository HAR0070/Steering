# Steering
Testing steering
The steering angle is converted to velocity commands through PID 
Velocity is command is send to arduino -> CAN -> AK80-8 motor controller 

The feedback and commands are published as ROS1 topics 

The steering data collection stage 1 is sucessfull - model is perfectly fit and test data is visibily distinguishable 
<img width="1212" height="374" alt="image" src="https://github.com/user-attachments/assets/363c33bf-2589-4775-87ea-05c4abbe07a4" />

test_success.ipynb has the final data augmentation and results 

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

# To Do 
Should do better feature selection and pruning for the ML model
