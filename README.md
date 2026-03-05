## Objective 
In emergency driving situations, a driver’s immediate instinct is typically to regain control of the vehicle through steering. Motivated by this natural response, this work aims to develop a steering-based full takeover mechanism that enables the driver to promptly assume control of the vehicle during critical events.

Details on the steering modifications done can be found [here](https://har0070.github.io/har.github.io/posts/autonomy/#steering-setup)

## Steering controller -  works on arduino and MCP2515 can controller with 8MHz crystal and 1Mbps 
Arduino code is present in CAN_send_v1 folder 

Python script takes the joystic inputs to command the steering, with disabling and enabling ability 
It was found that in AK80-8 motor, sending position reference through servo mode sometimes causes sudden jerk. Considering this as a potential hazard I decided to use velocity command and build a PID loop. 
Now a steady feedback from the motor is neccesary for operation. 

joystick_ros_pub.py is the joystick controller. 
The feedback and commands are published as ROS1 topics 

## Steering Data collection and data analysis 
Data was collected for two scenarios first run was through campus running with joystick controller, recording steering data 
- current (torque)
- position, velocity
- Vehicle velocities
- Vhicle yaw, pitch and roll
- GPS location 

Data visualization and  analysis files are in Steering_data folder 
A XGBoost regression tree was trained to predict the torque requirement, and if actual torque exceeds predicted by more that 1.5 std dev for 3 consecutive instance then takeover is initiated. Driver gets the full vehicle control. More details is selection of model and why not classical technique and more ... is described in my article [here]( https://har0070.github.io/har.github.io/posts/autonomy/#steering-takeover-design) 

test_success.ipynb has the final data augmentation and results 
Driver_takerover.py has the online implementation of the model.

The steering takeover model's prediction on test data is visibily distinguishable 
<img width="1212" height="374" alt="image" src="https://github.com/user-attachments/assets/363c33bf-2589-4775-87ea-05c4abbe07a4" />

## System specs
Ubuntu 20.04 , ROS1,  python 3.12 , Arduino UNO, MCP2515 


