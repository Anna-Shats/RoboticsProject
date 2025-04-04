import time
import json
from Motor import *
from gpiozero import Buzzer as GPIOBuzzer, LineSensor, DistanceSensor
from servo import *
from PCA9685 import PCA9685
from ADC import *

# === CONFIGURATION ===
PHASE = 1  # 1 = Mapping | 2 = Fast Run
PATH_FILE = "path.json"
EXIT_DISTANCE_THRESHOLD = 100  # cm

# === SENSOR SETUP ===
IR01 = 14
IR02 = 15
IR03 = 23
IR01_sensor = LineSensor(IR01)
IR02_sensor = LineSensor(IR02)
IR03_sensor = LineSensor(IR03)

buzzer = GPIOBuzzer(17)
trigger_pin = 27
echo_pin = 22
ultrasonic_sensor = DistanceSensor(echo=echo_pin, trigger=trigger_pin, max_distance=3)

# === HARDWARE CLASSES ===
class Ultrasonic:
    def get_distance(self):
        return int(ultrasonic_sensor.distance * 100)

class Buzzer:
    def run(self, command):
        if command != "0":
            buzzer.on()
        else:
            buzzer.off()

class Motor:
    def _init_(self):
        self.pwm = PCA9685(0x40, debug=True)
        self.pwm.setPWMFreq(50)

    def duty_range(self, d1, d2, d3, d4):
        return [max(min(d, 4095), -4095) for d in (d1, d2, d3, d4)]

    def setMotorModel(self, d1, d2, d3, d4):
        d1, d2, d3, d4 = self.duty_range(d1, d2, d3, d4)
        self._set_wheel(0, 1, d1)
        self._set_wheel(2, 3, d2)
        self._set_wheel(6, 7, d3)
        self._set_wheel(4, 5, d4)

    def _set_wheel(self, a, b, duty):
        if duty > 0:
            self.pwm.setMotorPwm(a, 0)
            self.pwm.setMotorPwm(b, duty)
        elif duty < 0:
            self.pwm.setMotorPwm(b, 0)
            self.pwm.setMotorPwm(a, abs(duty))
        else:
            self.pwm.setMotorPwm(a, 4095)
            self.pwm.setMotorPwm(b, 4095)

    def stop(self):
        self.setMotorModel(0, 0, 0, 0)

class Scanner:
    def _init_(self):
        self.servo = Servo()
        self.ultrasonic = Ultrasonic()

    def scan(self):
        readings = {}
        for angle, label in [(30, "left"), (90, "middle"), (150, "right")]:
            self.servo.setServoPwm('0', angle)
            time.sleep(0.2)
            readings[label] = self.ultrasonic.get_distance()
        return readings

# === INIT OBJECTS ===
PWM = Motor()
B = Buzzer()
ultrasonic = Ultrasonic()
scanner = Scanner()
path_memory = []

# === MAIN LOGIC ===
if _name_ == "_main_":
    print("Maze car starting...")

    try:
        if PHASE == 1:
            print("Phase 1: Mapping")
            maze_started = False

            while True:
                if not maze_started:
                    # Follow black line before maze
                    LMR = 0x00
                    if IR01_sensor.value: LMR |= 4
                    if IR02_sensor.value: LMR |= 2
                    if IR03_sensor.value: LMR |= 1

                    if LMR == 2:
                        PWM.setMotorModel(800, 800, 800, 800)
                    elif LMR == 4 or LMR == 6:
                        PWM.setMotorModel(2000, 2000, -500, -500)
                        maze_started = True
                        print("Maze entry detected — switching to ultrasonic.")
                        time.sleep(0.4)
                    elif LMR == 1 or LMR == 3:
                        PWM.setMotorModel(-500, -500, 2000, 2000)
                        maze_started = True
                        print("Maze entry detected — switching to ultrasonic.")
                        time.sleep(0.4)
                    elif LMR == 0:
                        PWM.stop()
                else:
                    # Maze navigation using ultrasonic + IR floor check
                    distances = scanner.scan()
                    L, M, R = distances["left"], distances["middle"], distances["right"]

                    # Exit detection
                    ultrasonic_exit = M > EXIT_DISTANCE_THRESHOLD
                    floor_exit = not (IR01_sensor.value and IR02_sensor.value and IR03_sensor.value)

                    print(f"Distances: L={L} M={M} R={R} | FloorExit={floor_exit}")

                    if ultrasonic_exit or floor_exit:
                        print("Maze exit detected!")
                        PWM.stop()
                        break

                    if L < 30 and M < 30 and R < 30:
                        # Dead end - turn around
                        PWM.setMotorModel(-1400, -1400, -1400, -1400)
                        time.sleep(0.5)
                        PWM.setMotorModel(-1500, -1500, 1500, 1500)
                        path_memory.append("left")
                        time.sleep(0.5)
                    elif L > R:
                        # Turn left
                        PWM.setMotorModel(-500, -500, 2000, 2000)
                        path_memory.append("left")
                        time.sleep(0.5)
                    elif R > L:
                        # Turn right
                        PWM.setMotorModel(2000, 2000, -500, -500)
                        path_memory.append("right")
                        time.sleep(0.5)
                    else:
                        # Move forward
                        PWM.setMotorModel(1000, 1000, 1000, 1000)
                        path_memory.append("forward")
                        time.sleep(0.4)

            # Save path
            with open(PATH_FILE, "w") as f:
                json.dump(path_memory, f)
            print(f"Path saved: {path_memory}")

        elif PHASE == 2:
            print("Phase 2: Fast run")
            with open(PATH_FILE, "r") as f:
                path_memory = json.load(f)

            for move in path_memory:
                if move == "forward":
                    PWM.setMotorModel(2000, 2000, 2000, 2000)
                    time.sleep(0.3)
                elif move == "left":
                    PWM.setMotorModel(-500, -500, 2000, 2000)
                    time.sleep(0.4)
                elif move == "right":
                    PWM.setMotorModel(2000, 2000, -500, -500)
                    time.sleep(0.4)

            PWM.stop()
            print("Fast run complete! Car reached exit.")

    except KeyboardInterrupt:
        PWM.stop()
        B.run("0")
        print("Program interrupted. Car stopped.")