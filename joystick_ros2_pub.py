import pygame
import serial
import time
import sys
import datetime
# import rospy 
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


ARDUINO_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600

DEADZONE = 0.1
SEND_INTERVAL = 0.05 

KP = 10
KD = 0.5
KI = 0.5
SPD_REF = 4000
CUR_LIM = 2.5

class steering_fb(Node):

    def __init__(self , ARDUINO_PORT , BAUD_RATE):
        super().__init__('steering_fb')
        self.pub_vel = self.create_publisher(Twist, '/steering_pub', 10)
        self.pub_mfb = self.create_publisher(Float32MultiArray , '/Float32MultiArray' , 10)
        
        timer_period = SEND_INTERVAL  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        # State variables
        self.integral_err = 0
        self.arduino_serial = None
        self.controller = None
        
        # Initialization
        self.get_logger().info("Initializing Controller and Serial...")
        
        try:
            self.find_controller()
            self.axis_x = self.controller.get_axis(0)
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
        msg.linear.x = x
        if y:
            msg.linear.y = y
        
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
            
        if curr_fb > CUR_LIM:
            print(f"your hitting current limit {curr_fb}")
            velocity *=0.5

        print(f" vel = {velocity} pos_err = {pos_err} spd_err = {spd_err} pos_fb = {pos_fb}  pos_ref = {ref_pos} spd_fb = {speed_fb}  \r")

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
        position = int(axis_value * 3100)

        if inverted:
            position = -position
            
        return position

    def timer_callback( self ):

        try:
            # This is CRITICAL must pump the event queue.
            pygame.event.pump()
            self.axis_x = self.controller.get_axis(0)
            pos_x = self.map_axis_to_position(self.axis_x)
            # pos_y = map_axis_to_position(axis_y, inverted=True)
    
            if self.arduino_serial.in_waiting > 0:
                # Read all available bytes and throw them away (or print them for debug)
                line = self.arduino_serial.readline().decode('utf-8', errors='ignore').strip()
                if line:  
                    msg = self.to_msg(line)
                    pos_fb = msg.data[0]
                    speed_fb = msg.data[1]
                    curr_fb = msg.data[2]
                    self.pub_mfb.publish(msg)
                    if msg.data[-1] != 100:
                        vel_x , self.integral_err = self.pid_pos_vel(pos_x , pos_fb , speed_fb, curr_fb , self.integral_err)
        
                        message = f"<{vel_x},{0}>\n"

                        self.arduino_serial.write(message.encode('utf-8'))
                        msg = self.to_twist(vel_x)

                        self.pub_vel.publish(msg)
        except IndexError:
            pass
        
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")
            raise

        except KeyboardInterrupt:
            print("\nExiting program.")
            sys.exit(1)
            
        except Exception as e:
            print(f"\n Serial write error: {e}")
            raise 
        
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