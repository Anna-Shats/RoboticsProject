from Line_Tracking import Line_Tracking
from motor import Ordinary_Car
import time

def follow_line(infrared, motor):
    """Function to follow the line based on infrared sensor readings"""
    try:
        while True:
            direction = infrared.direction_Infrared()
            
            if direction == 'middle':
                # Move forward at moderate speed when on the middle of the line
                motor.set_motor_model(-1200, -1200, -1200, -1200)
                print("Moving forward")
            elif direction == 'left':
                # Turn right when the left sensor detects the line
                motor.set_motor_model(-2000,-2000,2000,2000)
                print("Turning right")
            elif direction == 'right':
                # Turn left when the right sensor detects the line
                motor.set_motor_model(2000,2000,-2000,-2000)
                print("Turning left")
            elif direction is None:
                # Stop when no line is detected
                motor.set_motor_model(0, 0, 0, 0)
                print("Stopping - no line detected")
            
            # Small delay to prevent overwhelming the system
            time.sleep(0.1)
            
    except Exception as e:
        print(f"An error occurred: {e}")
        motor.set_motor_model(0, 0, 0, 0)

if __name__ == '__main__':
    print('Program is starting ... ')
    
    # Initialize the components
    infrared = Line_Tracking()
    PWM = Ordinary_Car()
    
    # When direction_Infrared is left, the robot will turn right.
    # When direction_Infrared is right, the robot will turn left.
    # When direction_Infrared is middle, the robot will move forward.
    # When direction_Infrared is None, the robot will stop.
    
    try:
        # Uncomment to test only the infrared sensors
        # infrared.test_Infrared()
        
        # Start line following
        infrared.run()
        
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program will be executed.
        print("\nProgram stopped by user")
    finally:
        # Ensure motors are stopped when the program exits
        PWM.set_motor_model(0, 0, 0, 0)
        PWM.close()