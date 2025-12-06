import pygame
import serial
import time
import sys
import datetime

ARDUINO_PORT = "/dev/ttyACM0"
BAUD_RATE = 9600

DEADZONE = 0.1
SEND_INTERVAL = 0.05 

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

def map_axis_to_velocity(axis_value, inverted=False):
    """Converts a joystick axis (-1.0 to 1.0) to a velocity (-100 to 100)."""
    # Apply deadzone
    if abs(axis_value) < DEADZONE:
        axis_value = 0.0
        
    # Map the value from [-1.0, 1.0] to [-1000, 1000]
    velocity = int(axis_value * 5000)

    if inverted:
        velocity = -velocity
        
    return velocity

def main_loop(filename, controller, arduino_serial):
    """The main application loop."""
    print("Reading from Left Analog Stick (X/Y)...")
    try:
        while True:
            # This is CRITICAL. You must pump the event queue.
            pygame.event.pump()

            axis_x = controller.get_axis(0)
            axis_y = controller.get_axis(1)

            vel_x = map_axis_to_velocity(axis_x)
            vel_y = map_axis_to_velocity(axis_y, inverted=True)

            message = f"<{vel_x},{vel_y}>\n"
            
            # Send the message as bytes
            try:
                start_time = time.time()
                arduino_serial.write(message.encode('utf-8'))
                end_time = time.time() # Record end time
                elapsed_time = end_time - start_time
                # print(f"Sent: X={vel_x} Y={vel_y}  (Raw: {message.strip()})  elapsed_time : {elapsed_time}", end='\r')
            except serial.SerialException as e:
                print(f"\n Serial write error: {e}")
                raise
            except Exception as e:
                print(f"\n Serial write error: {e}")
                raise 

            # to read what the Arduino sends back, otherwise the buffer fills up and crashes the connection.
            with open(filename, "a") as log_file:
                try:
                    if arduino_serial.in_waiting > 0:
                        # Read all available bytes and throw them away (or print them for debug)
                        line = arduino_serial.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                                # Get timestamp
                                timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                                
                                # Format log entry
                                log_entry = f"[{timestamp}] Cmd: <{vel_x},{vel_y}> | Fb: {line}"
                                
                                # 1. Print to Console (One line)
                                print(f"{log_entry}          ", end='\n', flush=True)
                                
                                # 2. Write to File
                                log_file.write(log_entry + "\n")
                                # Force write to disk immediately (safer if script crashes)
                                log_file.flush() 
                except Exception as e:
                    pass
                
            # Don't spam the Arduino, send at a fixed rate
            time.sleep(SEND_INTERVAL)
    except Exception as e:
        print(f"the main loop broke due to error {e}")

    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        # Clean up
        if arduino_serial and arduino_serial.is_open:
            # Send a "stop" command before closing
            # arduino_serial.write(b"<0,0>\n")
            arduino_serial.close()
        pygame.joystick.quit()
        pygame.quit()
        print("Connections closed.")

if __name__ == "__main__":
    controller = find_controller()
    filename = input("Entre log file name")
    if controller:
        arduino_serial = connect_to_arduino(ARDUINO_PORT, BAUD_RATE)
        if arduino_serial:
            main_loop(filename ,controller, arduino_serial)

        else:
            print("Could not connect to Arduino. Exiting.")
    else:
        print("Could not find controller. Exiting.")
    sys.exit()
