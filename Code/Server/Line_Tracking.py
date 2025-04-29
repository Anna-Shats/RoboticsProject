import time
from motor import *
from gpiozero import LineSensor
IR01 = 14
IR02 = 15
IR03 = 23
IR01_sensor = LineSensor(IR01)
IR02_sensor = LineSensor(IR02)
IR03_sensor = LineSensor(IR03)
class Line_Tracking:
    def __init__(self):
        pass

    def test_Infrared(self):
        PWM = Ordinary_Car()
        try:
            while True:
                if IR01_sensor.value !=True and IR02_sensor.value == True and IR03_sensor.value !=True:
                    print ('Middle')
                elif IR01_sensor.value !=True and IR02_sensor.value != True and IR03_sensor.value ==True:
                    print ('Right')
                elif IR01_sensor.value ==True and IR02_sensor.value != True and IR03_sensor.value !=True:
                    print ('Left')

        except KeyboardInterrupt:
            print ("\nEnd of program")

    def direction_Infrared(self):

        direction=None

        while True:
            if (IR01_sensor.value ==True and IR02_sensor.value != True and IR03_sensor.value !=True) or (IR01_sensor.value ==True and IR02_sensor.value == True and IR03_sensor.value !=True):
                direction='left'
                print ('Left')
            elif (IR01_sensor.value !=True and IR02_sensor.value != True and IR03_sensor.value ==True) or (IR01_sensor.value !=True and IR02_sensor.value == True and IR03_sensor.value ==True):
                direction='right'
                print ('Right')
            elif (IR01_sensor.value !=True and IR02_sensor.value == True and IR03_sensor.value !=True) or (IR01_sensor.value ==True and IR02_sensor.value == True and IR03_sensor.value ==True):
                direction='middle'
                print ('Middle')
            elif IR01_sensor.value !=True and IR02_sensor.value != True and IR03_sensor.value !=True:
                direction=None
                print ('None')
            return direction
        
    def run(self):
        PWM = Ordinary_Car()
        try:
            while True:
                # Middle sensor detects line - go straight
                if IR01_sensor.value !=True and IR02_sensor.value == True and IR03_sensor.value !=True:
                    print('Middle - Going straight')
                    PWM.set_motor_model(-1500, -1500, -1500, -1500)  # Forward at slower speed
                
                # Right sensor detects line - gentle right turn
                elif IR01_sensor.value !=True and IR02_sensor.value != True and IR03_sensor.value ==True:
                    print('Right - Gentle right turn')
                    PWM.set_motor_model(-1000, -1000, -500, -500)  # Slower right turn
                
                # Left sensor detects line - gentle left turn
                elif IR01_sensor.value ==True and IR02_sensor.value != True and IR03_sensor.value !=True:
                    print('Left - Gentle left turn')
                    PWM.set_motor_model(-500, -500, -1000, -1000)  # Slower left turn
                
                # No line detected - search pattern
                elif IR01_sensor.value !=True and IR02_sensor.value != True and IR03_sensor.value !=True:
                    print('No line - Searching')
                    # Gentle snake-like search pattern
                    PWM.set_motor_model(-800, -800, -1200, -1200)  # Slight left
                    time.sleep(0.3)
                    PWM.set_motor_model(-1200, -1200, -800, -800)  # Slight right
                    time.sleep(0.3)
                
                time.sleep(0.1)  # Small delay for smoother movement
                
        except KeyboardInterrupt:
            print("\nEnd of program")
        finally:
            PWM.set_motor_model(0, 0, 0, 0)  # Stop motors
            PWM.close()

line_tracking=Line_Tracking()
# Main program logic follows:
if __name__ == '__main__':
    print ('Program is starting ... ')
    try:
        line_tracking.run()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program  will be  executed.
        PWM.setMotorModel(0,0,0,0)
