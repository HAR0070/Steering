import pygame 
import rospy 
from geometry_msgs.msg import Twist

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

def steering_pub(msg):
    
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()


if __name__  == "__main__":
    controller = find_controller()

    if controller:
        try:
            pub = rospy.Publisher('/steering_pub' , Twist , queue_size=10)
            rospy.init_node('steering_pub')
            rate = rospy.Rate(20) # 20Hz

            while True:
                pygame.event.pump()

                x = controller.get_axis(0)

                # print(f"this is the event vaue {x}")
                msg = to_twist(x)

                pub.publish(msg)
                rate.sleep()

        except KeyboardInterrupt:
            print("\nExiting program.") 
        
        except rospy.ROSInterruptException:
            print("\nExiting program.")

        finally:
            pygame.joystick.quit()
            pygame.quit()
            print('Connection closed')
