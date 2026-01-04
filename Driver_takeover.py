import pygame
import serial
import time
import sys
import datetime
import rospy 
import numpy as np
from scipy.spatial.transform import Rotation as R
import joblib 


from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from fixposition_driver_msgs.msg import FpaOdomenu

## how to import custom message
model = joblib.load("boost_tree_model.joblib")


ARDUINO_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200                  # for higher baudrate - in brake we made some change

# Joystick param
DEADZONE = 0.1
SEND_INTERVAL = 0.05 

# PID param 
KP = 50
KD = 0.5
KI = 0.5
SPD_REF = 6000
CUR_LIM = 2.5
EXTREME_POS = 650

# motor param
POLE_PAIR = 21
GR = 8
KT = 0.199

#Odom feedback
odom_fb = []

# class data:

#     def __init__(self):
#         x = 121  # total number of features

#         self.base_min = []
#         self.prev = np.zeros(( x ,10))
#         self.full_feature = []
#         self.takeover_count = 0

    
#     def lag_features(self, df : list):

#         for lag in [1,2,3,4,5,10]:
#             made_features = self.prev[lag]
#             self.full_feature.append(made_features)


#     def emw_feature(self, df):

#         # take exponential average of with span of 5 and 10 
#         for span in [5,10]:
#             alpha = [2/(span + 1)]
#             # convert everything to numpy array and smooth 
            
#             lag_feature = df*alpha + self.prev[allocation_id]*(1-alpha)
        
#             self.full_feature.append(lag_feature)

#     def diff_features(self , df):
#         # take difference from prev steps 
#         for d in [1,2,3,4]:
#             # convert everything to numpy array and apply diff
            
#             diff_feature = df - self.prev[allocation_id][d]
        
#             self.full_feature.append(diff_feature)
        

#     def add_feature(self , features : list):
#         self.base_min = features
#         # order is [pos_fb ,speed_fb ,curr_fb ] + [vx , vy , v_speed , yaw , pitch , roll]
#         f_dev = namedtuple('feature', ['pos', 'steering_spd' ,'steering_accel' ,'roll' , 'vx' , 'vy' , 'pitch' , 'yaw' , 'beta' , 'yaw_term' , 'v_speed'])
#         target = namedtuple('target' , )
#         # processing yaw 
#         pos = features[0] 
#         steering_spd = features[1]*(POLE_PAIR*GR)
#         steering_accel  = steering_spd - self.prev[1]
#         torque = features[2]*KT



#         df = f_dev(pos = pos , steering_spd=steering_spd , )

#         self.lag_features(df)
#         self.emw_feature(df)
#         self.diff_features(df)

                
#     def takover_logic(self , y_pred):
#         self.prev_ped.append(y_pred)
#         std = std(self.prev_ped)

#         if self.target.torque > y_pred + 0.5*std:
#             self.takeover_count += 1
            

#     def prediction(self , features : list):
#         self.base_min = features

#         self.add_feature()

#         # Take self.full_feature 

#         y_pred = model.predict(np.array(self.full_feature))
#         self.takover_logic(y_pred)
#         # check 3 point logic 
#         if self.takeover_count > 3:
#             return True
#         else:
#             return False


class DataBuffer:
    def __init__(self):
        self.base_dim = 8   # NO torque here
        self.max_hist = 11

        self.base_hist = np.zeros((self.max_hist, self.base_dim))
        self.torque_hist = np.zeros(self.max_hist)

        self.valid_steps = 0

        # EWMA state
        self.ewma_base_5 = np.zeros(self.base_dim)
        self.ewma_base_10 = np.zeros(self.base_dim)
        self.ewma_torque_5 = 0.0
        self.ewma_torque_10 = 0.0

        self.a5 = 2 / (5 + 1)
        self.a10 = 2 / (10 + 1)

        self.prev_preds = []
        self.takeover_count = 0

    def update(self, base_x, torque):
        self.base_hist[1:] = self.base_hist[:-1]
        self.torque_hist[1:] = self.torque_hist[:-1]

        self.base_hist[0] = base_x
        self.torque_hist[0] = torque

        if self.valid_steps == 0:
            self.ewma_base_5 = base_x.copy()
            self.ewma_base_10 = base_x.copy()
            self.ewma_torque_5 = torque
            self.ewma_torque_10 = torque
        else:
            self.ewma_base_5 = self.a5 * base_x + (1 - self.a5) * self.ewma_base_5
            self.ewma_base_10 = self.a10 * base_x + (1 - self.a10) * self.ewma_base_10
            self.ewma_torque_5 = self.a5 * torque + (1 - self.a5) * self.ewma_torque_5
            self.ewma_torque_10 = self.a10 * torque + (1 - self.a10) * self.ewma_torque_10

        self.valid_steps += 1

    def build_features(self):
        if self.valid_steps < 11:
            return None

        feats = []

        # base(t)
        feats.append(self.base_hist[0])

        # base lags
        for lag in [1, 2, 3, 5, 10]:
            feats.append(self.base_hist[lag])

        # torque lags
        for lag in [1, 2, 3, 5, 10]:
            feats.append(np.array([self.torque_hist[lag]]))

        # ewma
        feats.append(self.ewma_base_5)
        feats.append(self.ewma_base_10)
        feats.append(np.array([self.ewma_torque_5]))
        feats.append(np.array([self.ewma_torque_10]))

        # diffs
        for d in [1, 2, 3, 4]:
            feats.append(self.base_hist[0] - self.base_hist[d])
        for d in [1, 2, 3, 4]:
            feats.append(np.array([self.torque_hist[0] - self.torque_hist[d]]))

        return np.concatenate(feats)

    def predict(self, base_x, torque):
        self.update(base_x, torque)

        x = self.build_features()
        if x is None:
            return False

        y_pred = model.predict(x.reshape(1, -1))[0]

        # takeover logic
        self.prev_preds.append(y_pred)
        if len(self.prev_preds) > 10:
            self.prev_preds.pop(0)

        if len(self.prev_preds) < 5:
            return False

        std = np.std(self.prev_preds)

        if torque > y_pred + 0.5 * std:
            self.takeover_count += 1
        else:
            self.takeover_count = max(0, self.takeover_count - 1)

        return self.takeover_count > 3


def find_controller():
    """Initializes pygame and finds the first available joystick."""
    print("Initializing controller...")
    pygame.init()
    pygame.joystick.init()
    
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("Error: No joystick or controller found.")
        return None
        
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Found controller: {joystick.get_name()}")
    print(f"Axes: {joystick.get_numaxes()}, Buttons: {joystick.get_numbuttons()}")
    return joystick

def to_twist(x , y=None):
    msg = Twist() 
    msg.linear.x = x
    if y:
        msg.linear.y = y
    
    return msg

def to_msg(line , y=None):

    msg = Float32MultiArray()

    content = line.strip().split(" ")
    check = set(content)
    if "RX" in check:
        position = float(content[-16])
        speed = float(content[-12])
        current = float(content[-8])
        temperature = float(content[-4][:-1])
        err = float(content[-1])
        msg.data = [position, speed, current , temperature, err]
    else:
        msg.data = [0, 0, 0 , 0, 100]

    return msg

def pid_pos_vel(ref_pos , pos_fb , speed_fb , curr_fb , integral_err):

    pos_err = ref_pos - pos_fb          # if ref > current - velocity -ve
    spd_err = -speed_fb     

    if abs(pos_err) < 1.5:
        integral_err = 0
    else:
        integral_err += pos_err

    velocity = KP * pos_err + KD * spd_err + KI*integral_err ## positive position error -> Positive velocity 

    if velocity > SPD_REF:
        velocity = SPD_REF
        integral_err -= pos_err
    elif -velocity > SPD_REF:
        velocity = -SPD_REF
        integral_err -= pos_err
        
    if pos_fb > EXTREME_POS:
        velocity = 0
        integral_err -= 0
    elif -pos_fb > EXTREME_POS:
        velocity = 0
        integral_err -= 0

    if curr_fb > CUR_LIM:
        print(f"your hitting current limit {curr_fb}")
        velocity *=0.5

    print(f" vel = {velocity} pos_err = {pos_err} spd_err = {spd_err} pos_fb = {pos_fb}  pos_ref = {ref_pos} spd_fb = {speed_fb}  \r")

    return velocity , integral_err

def connect_to_arduino(port, baud):
    """Tries to connect to the Arduino over serial."""
    print(f"Connecting to Arduino at {port}...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
        # Wait for the Arduino to reset (common after opening serial)
        time.sleep(2)
        print("Connection successful.")
        return ser
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {port}.")
        return None

def map_axis_to_position(axis_value, inverted=False):
    """Converts a joystick axis (-1.0 to 1.0) to a velocity (-100 to 100)."""
    # Apply deadzone
    if abs(axis_value) < DEADZONE:
        axis_value = 0.0
        
    # Map the value from [-1.0, 1.0] to [-520, 520]
    position = int(axis_value * 600)

    if inverted:
        position = -position
        
    return position


def read_odomenu(msg):
    # p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    v = msg.velocity.twist.linear
    # w = msg.velocity.twist.angular
    
    yaw , pitch , roll = R.from_quat([q.x,q.y,q.z,q.w]).as_euler('zyx' , degrees=True)
    vx , vy = v.x , v.y 
    v_speed = (vx**2 + vy**2)**0.5
    odom_fb = [vx , vy , v_speed , yaw , pitch , roll]


def main_loop( controller, arduino_serial , filename=None):
    """The main application loop."""
    print("Reading from Left Analog Stick (X/Y)...")
    try:
        pub_steering = rospy.Publisher('/steering_pub' , Twist , queue_size=10)
        pub_motor_fb = rospy.Publisher('/motor_feedback', Float32MultiArray, queue_size=10)
        sub_odomenu = rospy.Subscriber('/fixposition/fpa/odomenu' , FpaOdomenu , queue_size=10 )
        rospy.init_node('steering_pub')
        rate = rospy.Rate(20) # 20Hz
        takeover_cmd = False

        buffer = DataBuffer()

        integral_err = 0

        while not rospy.is_shutdown():
            # This is CRITICAL. You must pump the event queue.
            pygame.event.pump()

            axis_x = controller.get_axis(0)
            # axis_y = controller.get_axis(1)

            pos_x = map_axis_to_position(axis_x)
            # pos_y = map_axis_to_position(axis_y, inverted=True)

            try:
                while arduino_serial.in_waiting > 0:
                    # Read all available bytes and throw them away (or print them for debug)
                    line = arduino_serial.readline().decode('utf-8', errors='ignore').strip()

                if line:  
                    msg = to_msg(line)
                    pos_fb = msg.data[0]
                    speed_fb = msg.data[1]
                    curr_fb = msg.data[2]
                    pub_motor_fb.publish(msg)
                    
                    # collecting for prediction

                    if msg.data[-1] != 100:
                        vel_x , integral_err = pid_pos_vel(pos_x , pos_fb , speed_fb, curr_fb , integral_err)
                        # print(f"vel_x = {vel_x}  pos_ref = {pos_x} pos_fb = {pos_fb}")

                        if takeover_cmd:
                            message = f"<{vel_x},{1}>\n"
                        else:
                            message = f"<{vel_x},{0}>\n"

                        # start_time = time.time()
                        arduino_serial.write(message.encode('utf-8'))
                        msg = to_twist(vel_x)
                        pub_steering.publish(msg)

                feature = [pos_fb ,speed_fb ] + odom_fb
                torque = curr_fb*KT 

                takeover_cmd = buffer.predict(feature , torque)

                        
            except IndexError:
                pass
            
            except serial.SerialException as e:
                print(f"\n Serial write error: {e}")
                raise
            except Exception as e:
                print(f"\n Serial write error: {e}")
                raise 

            # # to read what the Arduino sends back, otherwise the buffer fills up and crashes the connection.
            if filename:
                with open(filename, "a") as log_file:
                    try:
                        if line:
                                # Get timestamp
                                timestamp = datetime.datetime.now().strftimvel_xe("%H:%M:%S.%f")[:-3]
                                
                                log_entry = f"[{timestamp}] Cmd: <{pos_x},{vel_x}> | Fb: {line}"
                                
                                # 1. Print to Console (One line)
                                print(f"{log_entry}          ", end='\n', flush=True)
                                
                                # 2. Write to File
                                log_file.write(log_entry + "\n")
                                # Force write to disk immediately (safer if script crashes)
                                log_file.flush() 
                    except KeyboardInterrupt:
                        raise
                    except Exception as e:
                        pass

            rate.sleep()  

    except Exception as e:
        print(f"the main loop broke due to error {e}")

    except KeyboardInterrupt:
        print("\nExiting program.")
        sys.exit(1)
    finally:
        # Clean up
        if arduino_serial and arduino_serial.is_open:
            arduino_serial.close()
        pygame.joystick.quit()
        pygame.quit()
        print("Connections closed.")


if __name__ == "__main__":
    controller = find_controller()
    # filename = input("Entre log file name")

    if controller:
        arduino_serial = connect_to_arduino(ARDUINO_PORT, BAUD_RATE)
        if arduino_serial:
            main_loop(controller, arduino_serial)

        else:
            print("Could not connect to Arduino. Exiting.")
    else:
        print("Could not find controller. Exiting.")
    sys.exit()
