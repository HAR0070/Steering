import pygame
import serial
import time
import sys
import datetime
# import rospy 
import rclpy
from rclpy.node import Node

import numpy as np
from scipy.spatial.transform import Rotation as R
import joblib 

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from fixposition_driver_msgs.msg import FpaOdomenu

import warnings
warnings.filterwarnings("ignore")


try:
    model = joblib.load("boost_tree_model.joblib")
except FileNotFoundError:
    print("WARNING: Model file not found. Prediction will fail.")
    model = None


ARDUINO_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200                  # for higher baudrate - in brake we made some change

# Joystick param
DEADZONE = 0.1
SEND_INTERVAL = 0.05 
AXIS = 2   ## HAR-TUF  has joystick axis = 2, ASL - axis = 0

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


class DataBuffer:
    def __init__(self):
        self.base_dim = 11  # NO torque here
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

    # def build_features(self):
    #     if self.valid_steps < 11:
    #         return None

    #     feats = []

    #     # base(t)
    #     feats.append(self.base_hist[0])

    #     # base lags
    #     for lag in [1, 2, 3,4, 5, 10]:
    #         feats.append(self.base_hist[lag])

    #     # torque lags
    #     for lag in [1, 2, 3, 4, 5, 10]:
    #         feats.append(np.array([self.torque_hist[lag]]))

    #     # ewma
    #     feats.append(self.ewma_base_5)
    #     feats.append(self.ewma_base_10)
    #     feats.append(np.array([self.ewma_torque_5]))
    #     feats.append(np.array([self.ewma_torque_10]))

    #     # diffs
    #     for d in [1, 2, 3, 4]:
    #         feats.append(self.base_hist[0] - self.base_hist[d])
    #     for d in [1, 2, 3, 4]:
    #         feats.append(np.array([self.torque_hist[0] - self.torque_hist[d]]))

    #     return np.concatenate(feats)
    
    def build_features(self):
        if self.valid_steps < 11:
            return None

        # Helper to access base features by column
        # self.base_hist is shape (11, 11) -> (time, feature_index)
        # We perform a transpose to access it as (feature_index, time)
        base_T = self.base_hist.T 
        
        # Feature Indices (based on training base_features list)
        # 0:pos, 1:spd, 2:acel, 3:roll, 4:vx, 5:vy, 6:pitch, 7:yaw, 8:beta, 9:term, 10:v_spd
        
        feature_vector = []

        # ---------------------------------------------------------
        # GROUP 1: Base State at time t (11 features)
        # Training: predictors = base_features + ...
        # ---------------------------------------------------------
        feature_vector.extend(self.base_hist[0])

        # ---------------------------------------------------------
        # GROUP 2: Lags (72 features)
        # Training: for feat in base_features + ['torque']: for lag in lags:
        # Note: Training groups by FEATURE, then by LAG
        # ---------------------------------------------------------
        lags = [1, 2, 3, 4, 5, 10]
        
        # 2a. Base Feature Lags
        for feat_idx in range(11): 
            for lag in lags:
                feature_vector.append(base_T[feat_idx, lag])
        
        # 2b. Torque Lags (torque_hist is 1D array)
        for lag in lags:
            feature_vector.append(self.torque_hist[lag])

        # ---------------------------------------------------------
        # GROUP 3: Smooths / EWMA (22 features)
        # Training: for feat in base_features: for lag in rols [5, 10]:
        # ---------------------------------------------------------
        for feat_idx in range(11):
            # ewma_base_5 is an array of 11 items. We pick the specific feature.
            feature_vector.append(self.ewma_base_5[feat_idx])
            feature_vector.append(self.ewma_base_10[feat_idx])

        # ---------------------------------------------------------
        # GROUP 4: Diffs (48 features)
        # Training: for feat in base_features + ['torque']: for lag in diff [1, 2, 3, 4]:
        # ---------------------------------------------------------
        diffs = [1, 2, 3, 4]
        
        # 4a. Base Feature Diffs
        for feat_idx in range(11):
            for d in diffs:
                # diff is value(t) - value(t-d)
                val = base_T[feat_idx, 0] - base_T[feat_idx, d]
                feature_vector.append(val)

        # 4b. Torque Diffs
        for d in diffs:
            val = self.torque_hist[0] - self.torque_hist[d]
            feature_vector.append(val)

        # ---------------------------------------------------------
        # GROUP 5: Specific Extras (3 features)
        # Training: .extend(['vel_trend_10', 'torque_smooth_10' , 'torque_smooth_5'])
        # ---------------------------------------------------------
        
        # 5a. vel_trend_10 (THE MISSING FEATURE)
        # Simple rolling mean of steering_spd (index 1) over window 10
        vel_trend = np.mean(base_T[1, 0:10]) 
        feature_vector.append(vel_trend)

        # 5b. torque_smooth_10 (EWMA 10)
        feature_vector.append(self.ewma_torque_10)

        # 5c. torque_smooth_5 (EWMA 5)
        feature_vector.append(self.ewma_torque_5)

        # ---------------------------------------------------------
        # Verification
        # 11 + 66 + 6 + 22 + 44 + 4 + 3 = 156 Features
        # ---------------------------------------------------------
        return np.array(feature_vector)

    def predict(self, base_x, torque):
        # base_x is [pos_fb ,speed_fb ] + self.odom_fb 
        # self.odom has [vx , vy , v_speed , yaw , pitch , roll]
        base_x = np.array(base_x)
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


class steering_fb(Node):

    def __init__(self , ARDUINO_PORT , BAUD_RATE):
        super().__init__('steering_fb')
        self.pub_steering = self.create_publisher(Twist, '/steering_pub', 10)
        self.pub_motor_fb = self.create_publisher(Float32MultiArray , '/Float32MultiArray' , 10)
        self.sub_odomenu = self.create_subscription(
            FpaOdomenu,
            '/fixposition/fpa/odomenu',
            self.read_odomenu,
            10)
        self.sub_odomenu  # prevent unused variable warning
        
        timer_period = SEND_INTERVAL  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        # State variables
        self.integral_err = 0
        self.arduino_serial = None
        self.controller = None
        self.takeover_cmd = False
        self.buffer = DataBuffer()
        
        #Odom feedback
        self.odom_fb = [0.0]*6
        
        # State variables for Feature Engineering
        self.prev_rpm = 0.0
        self.prev_yaw = 0.0
        self.prev_yaw_init = False
        
        # Initialization
        self.get_logger().info("Initializing Controller and Serial...")
        
        try:
            self.find_controller()
            self.axis_x = self.controller.get_axis(AXIS)
        except Exception as e:
            print(f"couldn't connect to controller: {e}")
            
        try:
            self.connect_to_arduino(ARDUINO_PORT, BAUD_RATE)
        except Exception as e:
            print(f"Couldn't connect to arduino: {e}")

        # This is CRITICAL must pump the event queue.
        pygame.event.pump()

    def find_controller(self):
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
        
        self.controller = joystick      

    def to_twist(self, x , y=None):
        msg = Twist() 
        msg.linear.x = float(x)
        if y:
            msg.linear.y = float(y)
        
        return msg

    def to_msg(self ,line , y=None):

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
    
    def pid_pos_vel(self, ref_pos , pos_fb , speed_fb , curr_fb , integral_err):

        pos_err = ref_pos - pos_fb          # if ref > current - velocity -ve
        spd_err = -speed_fb     

        if abs(pos_err) < 1.5:
            integral_err = 0.0
        else:
            integral_err += pos_err

        velocity = KP * pos_err + KD * spd_err + KI*integral_err ## positive position error -> Positive velocity 

        if velocity > SPD_REF:
            velocity = SPD_REF
            integral_err -= pos_err
        elif -velocity > SPD_REF:
            velocity = -SPD_REF
            integral_err -= pos_err
        
        # You wont be able to return - incase motor overshoots
        # if pos_fb > EXTREME_POS:
        #     velocity = 0.0
        #     integral_err -= 0.0
        # elif -pos_fb > EXTREME_POS:
        #     velocity = 0.0
        #     integral_err -= 0.0

        if curr_fb > CUR_LIM:
            print(f"your hitting current limit {curr_fb}")
            velocity *=0.5

        # print(f" vel = {velocity} pos_err = {pos_err} spd_err = {spd_err} pos_fb = {pos_fb}  pos_ref = {ref_pos} spd_fb = {speed_fb}  \r")

        return velocity , integral_err

    def connect_to_arduino(self , port, baud):
        """Tries to connect to the Arduino over serial."""
        print(f"Connecting to Arduino at {port}...")
        try:
            ser = serial.Serial(port, baud, timeout=1)
            # Wait for the Arduino to reset (common after opening serial)
            time.sleep(2)
            print("Connection successful.")
            self.arduino_serial = ser 
        except serial.SerialException as e:
            print(f"Error: Could not open serial port {port}.")
            return None

    def map_axis_to_position(self, axis_value, inverted=False):
        """Converts a joystick axis (-1.0 to 1.0) to a velocity (-100 to 100)."""
        # Apply deadzone
        if abs(axis_value) < DEADZONE:
            axis_value = 0.0
            
        # Map the value from [-1.0, 1.0] to [-520, 520]
        position = int(axis_value * 600)

        if inverted:
            position = -position
            
        return position

    def read_odomenu(self, msg):
        # p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.velocity.twist.linear
        # w = msg.velocity.twist.angular
        
        yaw , pitch , roll = R.from_quat([q.x,q.y,q.z,q.w]).as_euler('zyx' , degrees=True)
        vx , vy = v.x , v.y 
        v_speed = (vx**2 + vy**2)**0.5
        self.odom_fb = [vx , vy , v_speed , yaw , pitch , roll]
    
    def timer_callback( self ):

        try:
            # This is CRITICAL must pump the event queue.
            pygame.event.pump()
            # ... (Joystick reading code remains the same) ...
            if self.controller:
                self.axis_x = self.controller.get_axis(AXIS)
                pos_x = self.map_axis_to_position(self.axis_x)
            else:
                pos_x = 0.0

            # ... (Serial reading logic remains the same) ...
            line = None
            if self.arduino_serial and self.arduino_serial.in_waiting > 0:
                while self.arduino_serial.in_waiting > 0:
                    raw_line = self.arduino_serial.readline()
                    try: line = raw_line.decode('utf-8', errors='ignore').strip()
                    except: pass

            if line:  
                msg = self.to_msg(line)
                pos_fb = msg.data[0]
                speed_fb = msg.data[1]
                curr_fb = msg.data[2]
                self.pub_motor_fb.publish(msg)
                
                if msg.data[-1] != 100:
                    vel_x , self.integral_err = self.pid_pos_vel(pos_x , pos_fb , speed_fb, curr_fb , self.integral_err)
    
                    # message = f"<{vel_x},{0}>\n"
                    if self.takeover_cmd:
                        message = f"<{vel_x},{1}>\n"
                    else:
                        message = f"<{vel_x},{0}>\n"
                        print(f" Driver takeover")
                        print(f" Driver takeover")
                        print(f" Driver takeover")

                    self.arduino_serial.write(message.encode('utf-8'))
                    msg = self.to_twist(vel_x)
                    self.pub_steering.publish(msg)
                    
                # ---------------- FEATURE ENGINEERING ----------------
                    
                # 1. Extract Odom (Ensure read_odomenu populates this correctly)
                # self.odom_fb is [vx, vy, v_speed, yaw, pitch, roll]
                vx = self.odom_fb[0]
                vy = self.odom_fb[1]
                v_speed = self.odom_fb[2]
                yaw_curr = self.odom_fb[3]
                pitch = self.odom_fb[4]
                roll = self.odom_fb[5]

                # 2. Convert Position to Radians (MATCHING TRAINING: pos_data = df['pos']*np.pi/180)
                pos_rad = pos_fb * np.pi / 180.0

                # 3. Calculate Steering Accel (diff of rpm)
                steering_acel = speed_fb - self.prev_rpm
                self.prev_rpm = speed_fb

                # 4. Calculate Beta (arctan(vy/vx))
                if abs(vx) > 0.01:
                    beta = np.arctan(vy / vx)
                else:
                    beta = 0.0
                
                # 5. Calculate Yaw Term (yaw_diff / DT / v_speed)
                if not self.prev_yaw_init:
                    self.prev_yaw = yaw_curr
                    self.prev_yaw_init = True
                
                yaw_diff = yaw_curr - self.prev_yaw
                # Handle Wrap Around (e.g. 179 -> -179 is a small move, not huge)
                if yaw_diff > 180: yaw_diff -= 360
                elif yaw_diff < -180: yaw_diff += 360
                
                # Yaw Rate (diff/DT) divided by speed
                if v_speed > 0.1:
                    yaw_term = (yaw_diff / SEND_INTERVAL) / (v_speed + 1e-6)
                else:
                    yaw_term = 0.0
                
                self.prev_yaw = yaw_curr
                
                # 6. CONSTRUCT FEATURE VECTOR (Strict Order Check)
                # Training: ['pos', 'steering_spd', 'steering_acel', 'roll', 'vx', 'vy', 'pitch', 'yaw', 'beta', 'yaw_term', 'v_speed']
                    
                feature = [
                    pos_rad,        # 0. pos (Radians)
                    speed_fb,       # 1. steering_spd
                    steering_acel,  # 2. steering_acel
                    roll,           # 3. roll
                    vx,             # 4. vx
                    vy,             # 5. vy
                    pitch,          # 6. pitch
                    yaw_curr,       # 7. yaw
                    beta,           # 8. beta
                    yaw_term,       # 9. yaw_term
                    v_speed         # 10. v_speed
                ]

                # 7. PREDICT
                torque = curr_fb * KT
                self.takeover_cmd = self.buffer.predict(feature, torque)
            
        except IndexError:
            pass
        
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")
            raise

        except KeyboardInterrupt:
            print("\nExiting program.")
            sys.exit(1)
            
        except Exception as e:
            self.get_logger().error(f"Error in timer: {e}")
        
    def cleanup(self):
        """Explicit cleanup function called on shutdown."""
        self.get_logger().info("Shutting down...")
        if self.arduino_serial and self.arduino_serial.is_open:
            self.arduino_serial.close()
        pygame.quit()
        
def main(args=None):
    rclpy.init(args=args)

    ros_node = steering_fb(ARDUINO_PORT , BAUD_RATE)
    
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup happens here, NOT in the timer loop
        ros_node.cleanup()
        ros_node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":

    main()