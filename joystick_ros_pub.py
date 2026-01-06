import pygame
import serial
import time
import sys
import datetime
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


ARDUINO_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

DEADZONE = 0.05
SEND_INTERVAL = 0.05 

KP = 10      #  on car value = 50
KD = 1      #   0.5
KI = 0.1     #   0.5
SPD_REF = 3000      #6000
CUR_LIM = 2.5
EXTREME_POS = 650

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
        # velocity = 0
        integral_err -= 0
    elif -pos_fb > EXTREME_POS:
        # velocity = 0
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

def main_loop( controller, arduino_serial , filename=None):
    """The main application loop."""
    print("Reading from Left Analog Stick (X/Y)...")
    try:
        pub_steering = rospy.Publisher('/steering_pub' , Twist , queue_size=10)
        pub_motor_fb = rospy.Publisher('/motor_feedback', Float32MultiArray, queue_size=10)
        rospy.init_node('steering_pub')
        rate = rospy.Rate(20) # 20Hz

        integral_err = 0

        while not rospy.is_shutdown():
            # This is CRITICAL. You must pump the event queue.
            pygame.event.pump()

            axis_x = controller.get_axis(0)
            axis_y = controller.get_button(7)

            pos_x = map_axis_to_position(axis_x)
            pos_y = map_axis_to_position(axis_y)

            try:
                while arduino_serial.in_waiting > 0:
                    # Read all available bytes and throw them away (or print them for debug)
                    line = arduino_serial.readline().decode('utf-8', errors='ignore').strip()
                    # print(line)
                if line:  
                    msg = to_msg(line)
                    pos_fb = msg.data[0]
                    speed_fb = msg.data[1]
                    curr_fb = msg.data[2]
                    pub_motor_fb.publish(msg)
                    if msg.data[-1] != 100:
                        vel_x , integral_err = pid_pos_vel(pos_x , pos_fb , speed_fb, curr_fb , integral_err)
        
                        message = f"<{vel_x},{pos_y}>\n"

                        # start_time = time.time()
                        arduino_serial.write(message.encode('utf-8'))
                        msg = to_twist(vel_x)

                        pub_steering.publish(msg)
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
