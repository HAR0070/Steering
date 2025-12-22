import pygame
import serial
import time
import sys
import datetime

ARDUINO_PORT = "COM7"  # for windows 
BAUD_RATE = 115200

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

def map_axis_to_brake(axis_value, inverted=False):
    """Converts a joystick axis (-1.0 to 1.0) to a velocity (-100 to 100)."""
    # Apply deadzone
    range = 5

    if abs(axis_value) < DEADZONE:
        axis_value = 0.0
        
    # Map the value from [-1.0, 1.0] to [-1000, 1000]
    brake = int(axis_value * range/5*255)

    if inverted:
        brake = -brake
    
    if -brake > 0:
        brake = 0
        
    return brake

def main_loop( controller, arduino_serial = None , filename=None):
    """The main application loop."""
    print("Reading from Left Analog Stick (X/Y)...")
    try:
        while True:
            # This is CRITICAL. You must pump the event queue.
            pygame.event.pump()

            axis_x = controller.get_axis(2)
            axis_y = controller.get_axis(3)

            # brake_1 = map_axis_to_brake(axis_x)
            brake_2 = map_axis_to_brake(axis_y)

            message = f"<{brake_2},{0}>\n"
            # Send the message as bytes
            try:
                # start_time = time.time()
                arduino_serial.write(message.encode('utf-8'))
                # end_time = time.time() # Record end time
                # elapsed_time = end_time - start_time
                print(f"Sent: X={brake_2} Y={0}  (Raw: {message.strip()}) ", end='\r')
            except serial.SerialException as e:
                print(f"\n Serial write error: {e}")
                raise
            except Exception as e:
                print(f"\n Serial write error: {e}")
                raise 

            # to read what the Arduino sends back, otherwise the buffer fills up and crashes the connection.
            with open(filename, "a") as log_file:
                try:
                    while arduino_serial.in_waiting > 0:
                        # Read all available bytes and throw them away (or print them for debug)
                        line = arduino_serial.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                                # Get timestamp
                                timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                                
                                # Format log entry
                                log_entry = f"[{timestamp}] Cmd: <{brake_2},{0}> | Fb: {line}"
                                
                                # 1. Print to Console (One line)
                                print(f"{log_entry}          ", end='\r', flush=True)
                                
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
    filename = input("Entre log file name  ")
    if controller:
        arduino_serial = connect_to_arduino(ARDUINO_PORT, BAUD_RATE)
        if arduino_serial:
            main_loop(controller, arduino_serial=arduino_serial , filename=filename )

        else:
            print("Could not connect to Arduino. Exiting.")
    else:
        print("Could not find controller. Exiting.")
    sys.exit()
