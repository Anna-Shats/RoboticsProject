import time
from Motor import *
from gpiozero import Buzzer, LineSensor, DistanceSensor
from servo import *
from PCA9685 import PCA9685
import math
from ADC import *
from Command import COMMAND as cmd

IR01 = 14
IR02 = 15
IR03 = 23
IR01_sensor = LineSensor(IR01)
IR02_sensor = LineSensor(IR02)
IR03_sensor = LineSensor(IR03)

buzzer = Buzzer(17)

trigger_pin = 27
echo_pin    = 22
sensor = DistanceSensor(echo=echo_pin, trigger=trigger_pin ,max_distance=3)

class Ultrasonic:
    def __init__(self):        
        pass
    def get_distance(self):     # get the measurement results of ultrasonic module,with unit: cm
        distance_cm = sensor.distance * 100
        return  int(distance_cm)
    
    def run_motor(self,L,M,R):
        if (L < 30 and M < 30 and R <30) or M < 30 :
            self.PWM.setMotorModel(-1450,-1450,-1450,-1450) 
            time.sleep(0.1)   
            if L < R:
                self.PWM.setMotorModel(1450,1450,-1450,-1450)
            else :
                self.PWM.setMotorModel(-1450,-1450,1450,1450)
        elif L < 30 and M < 30:
            PWM.setMotorModel(1500,1500,-1500,-1500)
        elif R < 30 and M < 30:
            PWM.setMotorModel(-1500,-1500,1500,1500)
        elif L < 20 :
            PWM.setMotorModel(2000,2000,-500,-500)
            if L < 10 :
                PWM.setMotorModel(1500,1500,-1000,-1000)
        elif R < 20 :
            PWM.setMotorModel(-500,-500,2000,2000)
            if R < 10 :
                PWM.setMotorModel(-1500,-1500,1500,1500)
        else :
            self.PWM.setMotorModel(600,600,600,600)
                
    def run(self):
        self.PWM=Motor()
        self.pwm_S=Servo()
        for i in range(30,151,60):
                self.pwm_S.setServoPwm('0',i)
                time.sleep(0.2)
                if i==30:
                    L = self.get_distance()
                elif i==90:
                    M = self.get_distance()
                else:
                    R = self.get_distance()
        while True:
            for i in range(90,30,-60):
                self.pwm_S.setServoPwm('0',i)
                time.sleep(0.2)
                if i==30:
                    L = self.get_distance()
                elif i==90:
                    M = self.get_distance()
                else:
                    R = self.get_distance()
                self.run_motor(L,M,R)
            for i in range(30,151,60):
                self.pwm_S.setServoPwm('0',i)
                time.sleep(0.2)
                if i==30:
                    L = self.get_distance()
                elif i==90:
                    M = self.get_distance()
                else:
                    R = self.get_distance()
                self.run_motor(L,M,R)

class Line_Tracking:
    def __init__(self):
        pass

    def test_Infrared(self):
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
        
    def run(self):
        while True:
            self.LMR=0x00
            if IR01_sensor.value == True:
                self.LMR=(self.LMR | 4)
            if IR02_sensor.value == True:
                self.LMR=(self.LMR | 2)
            if IR03_sensor.value == True:
                self.LMR=(self.LMR | 1)
            if self.LMR==2:
                PWM.setMotorModel(800,800,800,800)
            elif self.LMR==4:
                PWM.setMotorModel(-1500,-1500,2500,2500)
            elif self.LMR==6:
                PWM.setMotorModel(-2000,-2000,4000,4000)
            elif self.LMR==1:
                PWM.setMotorModel(2500,2500,-1500,-1500)
            elif self.LMR==3:
                PWM.setMotorModel(4000,4000,-2000,-2000)
            elif self.LMR==7:
                #pass
                PWM.setMotorModel(0,0,0,0)

class Motor:
    def __init__(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)
        self.time_proportion = 3     #Depend on your own car,If you want to get the best out of the rotation mode, change the value by experimenting.
        self.adc = Adc()
    def duty_range(self,duty1,duty2,duty3,duty4):
        if duty1>4095:
            duty1=4095
        elif duty1<-4095:
            duty1=-4095        
        
        if duty2>4095:
            duty2=4095
        elif duty2<-4095:
            duty2=-4095
            
        if duty3>4095:
            duty3=4095
        elif duty3<-4095:
            duty3=-4095
            
        if duty4>4095:
            duty4=4095
        elif duty4<-4095:
            duty4=-4095
        return duty1,duty2,duty3,duty4
        
    def left_Upper_Wheel(self,duty):
        if duty>0:
            self.pwm.setMotorPwm(0,0)
            self.pwm.setMotorPwm(1,duty)
        elif duty<0:
            self.pwm.setMotorPwm(1,0)
            self.pwm.setMotorPwm(0,abs(duty))
        else:
            self.pwm.setMotorPwm(0,4095)
            self.pwm.setMotorPwm(1,4095)
    def left_Lower_Wheel(self,duty):
        if duty>0:
            self.pwm.setMotorPwm(3,0)
            self.pwm.setMotorPwm(2,duty)
        elif duty<0:
            self.pwm.setMotorPwm(2,0)
            self.pwm.setMotorPwm(3,abs(duty))
        else:
            self.pwm.setMotorPwm(2,4095)
            self.pwm.setMotorPwm(3,4095)
    def right_Upper_Wheel(self,duty):
        if duty>0:
            self.pwm.setMotorPwm(6,0)
            self.pwm.setMotorPwm(7,duty)
        elif duty<0:
            self.pwm.setMotorPwm(7,0)
            self.pwm.setMotorPwm(6,abs(duty))
        else:
            self.pwm.setMotorPwm(6,4095)
            self.pwm.setMotorPwm(7,4095)
    def right_Lower_Wheel(self,duty):
        if duty>0:
            self.pwm.setMotorPwm(4,0)
            self.pwm.setMotorPwm(5,duty)
        elif duty<0:
            self.pwm.setMotorPwm(5,0)
            self.pwm.setMotorPwm(4,abs(duty))
        else:
            self.pwm.setMotorPwm(4,4095)
            self.pwm.setMotorPwm(5,4095)
            
 
    def setMotorModel(self,duty1,duty2,duty3,duty4):
        duty1,duty2,duty3,duty4=self.duty_range(duty1,duty2,duty3,duty4)
        self.left_Upper_Wheel(duty1)
        self.left_Lower_Wheel(duty2)
        self.right_Upper_Wheel(duty3)
        self.right_Lower_Wheel(duty4)
            
    def Rotate(self,n):
        angle = n
        bat_compensate =7.5/(self.adc.recvADC(2)*3)
        while True:
            W = 2000

            VY = int(2000 * math.cos(math.radians(angle)))
            VX = -int(2000 * math.sin(math.radians(angle)))

            FR = VY - VX + W
            FL = VY + VX - W
            BL = VY - VX - W
            BR = VY + VX + W

            PWM.setMotorModel(FL, BL, FR, BR)
            print("rotating")
            time.sleep(5*self.time_proportion*bat_compensate/1000)
            angle -= 5

class Buzzer:
    def run(self,command):
        if command!="0":
            buzzer.on()
        else:
            buzzer.off()



ultrasonic=Ultrasonic()    
infrared=Line_Tracking()
PWM=Motor() 

if __name__ == '__main__':
    print('Program is starting...')
    PWM = Motor()
    ultrasonic = Ultrasonic()
    infrared = Line_Tracking()
    B = Buzzer()  # Use the custom Buzzer class we defined
    
    try:
        while True:
            # Check for obstacles
            distance = ultrasonic.get_distance()
            if distance < 20:  # If obstacle is closer than 20cm
                PWM.setMotorModel(0, 0, 0, 0)  # Stop the car
                B.run('1')  # Turn on buzzer
                continue  # Skip to next iteration
            
            # Line following logic
            LMR = 0x00
            if IR01_sensor.value == True:  # Left sensor
                LMR = (LMR | 4)
            if IR02_sensor.value == True:  # Middle sensor
                LMR = (LMR | 2)
            if IR03_sensor.value == True:  # Right sensor
                LMR = (LMR | 1)
            
            # Control logic based on line position
            if LMR == 2:  # Line is in the middle
                PWM.setMotorModel(800, 800, 800, 800)  # Move forward
                B.run('0')  # Ensure buzzer is off
            elif LMR == 4 or LMR == 6:  # Line is to the left
                PWM.setMotorModel(2000, 2000, -500, -500)  # Turn right
                B.run('0')
            elif LMR == 1 or LMR == 3:  # Line is to the right
                PWM.setMotorModel(-500, -500, 2000, 2000)  # Turn left
                B.run('0')
            elif LMR == 0:  # No line detected
                PWM.setMotorModel(0, 0, 0, 0)  # Stop
                B.run('0')
            
            time.sleep(0.01)  # Small delay to prevent CPU overload
            
    except KeyboardInterrupt:
        PWM.setMotorModel(0, 0, 0, 0)  # Stop motors
        B.run('0')  # Turn off buzzer
        print("\nProgram ended by user")
