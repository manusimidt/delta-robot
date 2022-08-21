import time
from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from math import sqrt, atan, degrees
import numpy as np

min_pulses = [500, 500, 500]
max_pulses = [2830, 2789, 2730]

# reference angle is the true servo angle which is calibrated in such a way
# that the upper arm is horizontal
REFERENCE_ANGLE = 120

W_B = 70
L1 = 120
L2 = 205

W_P = 26
U_P = 52
S_P = 91


def setup_servos() -> list:
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50

    servos = [
        servo.Servo(
            pca.channels[0],
            min_pulse=min_pulses[0],
            max_pulse=max_pulses[0],
            actuation_range=180,
        ),
        servo.Servo(
            pca.channels[1],
            min_pulse=min_pulses[1],
            max_pulse=max_pulses[1],
            actuation_range=180,
        ),
        servo.Servo(
            pca.channels[2],
            min_pulse=min_pulses[2],
            max_pulse=max_pulses[2],
            actuation_range=180,
        ),
    ]

    return servos


def calibrate_servos(servos: list, reference_angle: int = REFERENCE_ANGLE) -> None:
    # set all servos to reference angle to avoid complications
    servos[0].angle = reference_angle
    servos[1].angle = reference_angle
    servos[2].angle = reference_angle

    for i in range(len(servos)):
        input(f"Press enter to start calibration for servo {i}")
        print(f"Moving servo to {reference_angle} degrees")
        servos[i].angle = reference_angle

        while True:
            command = input(
                f"Current pulse: {max_pulses[i]}: Press + or - to adjust the angle so that it is horitontal to the ground: "
            )

            if len(command) > 0 and command[0] == "+":
                increment = max(1, len(command) * 10 - 10)
                max_pulses[i] = max_pulses[i] + increment
                servos[i].set_pulse_width_range(min_pulses[i], max_pulses[i])
                servos[i].angle = reference_angle

            elif len(command) > 0 and command[0] == "-":
                decrement = max(1, len(command) * 10 - 10)
                max_pulses[i] = max_pulses[i] - decrement
                servos[i].set_pulse_width_range(min_pulses[i], max_pulses[i])
                servos[i].angle = reference_angle
            else:
                break

    print("The max_pulses are ", max_pulses)


def ta(angle):
    # input: angle with reference to horizontal
    # output: actual servo angle

    # 0 -> REFERENCE_ANGLE
    actual_angle = -angle + REFERENCE_ANGLE
    if actual_angle > 140:
        return 140
    elif actual_angle < 50:
        return 50
    return actual_angle


def ca(x: float, y: float, z: float):
    # inverse kinematics
    a = W_B - U_P
    b = (S_P/2) - (sqrt(3)/2) * W_B
    c = W_P - 0.5 * W_B

    E1 = 2 * L1 * (y+a)
    F1 = 2*z*L1
    G1 = x**2+y**2+z**2+a**2+L1**2+2*y*a-L2**2

    E2 = -L1*(sqrt(3) * (x+b)+y+c)
    F2 = 2*z*L1
    G2 = x**2+y**2+z**2+b**2+c**2+L1**2+2*(x*b+y*c)-L2**2

    E3 = L1*(sqrt(3)*(x-b)-y-c)
    F3 = 2*z*L1
    G3 = x**2 + y**2 + z**2 + b**2 + c**2 + L1**2 + 2*(-x*b+y*c)-L2**2
    
    t11 = (-F1 + sqrt(E1**2 + F1**2 - G1**2))/(G1 - E1)
    t12 = (-F1 - sqrt(E1**2 + F1**2 - G1**2))/(G1 - E1)
    
    t21 = (-F2 + sqrt(E2**2 + F2**2 - G2**2))/(G2 - E2)
    t22 = (-F2 - sqrt(E2**2 + F2**2 - G2**2))/(G2 - E2)
    
    t31 = (-F3 + sqrt(E3**2 + F3**2 - G3**2))/(G3 - E3)
    t32 = (-F3 - sqrt(E3**2 + F3**2 - G3**2))/(G3 - E3)
    
    theta11 = 2*degrees(atan(t11))
    theta12 = 2*degrees(atan(t12))
    
    theta21 = 2*degrees(atan(t21))
    theta22 = 2*degrees(atan(t22))
    
    theta31 = 2*degrees(atan(t31))
    theta32 = 2*degrees(atan(t32))
    
    theta1 = theta11 if -60 < theta11 < 90 else theta12
    theta2 = theta21 if -60 < theta21 < 90 else theta22    
    theta3 = theta31 if -60 < theta31 < 90 else theta32
    return theta1, theta2, theta3

def move_to(x,y,z):
    theta1, theta2, theta3 = ca(x, y, z)
    servos[0].angle = ta(theta1)
    servos[1].angle = ta(theta2)
    servos[2].angle = ta(theta3)

if __name__ == "__main__":
    print("Starting initialization...")

    print("0, 0, 250 ", ca(0,0,250))
    
    servos = setup_servos()
    # calibrate_servos(servos)
    
    move_to(-30, 0, 100)
    for b in range(5):
        for i in range(30):
            move_to(-30+i, 0, 100+i*2)
            time.sleep(0.01)
            
        for i in range(30):
            move_to(0+i, 0, 160-i*2)
            time.sleep(0.01)
        
        for i in range(30):
            move_to(30-i, 0, 100+i*2)
            time.sleep(0.01)
                
        for i in range(30):
            move_to(0-i, 0, 160-i*2)
            time.sleep(0.01)    
        
    move_to(0, 0, 100)
    exit()
    for i in range(100):
        coordinate_str: str = input("Please enter coordinates: ").strip()
        if len(coordinate_str.split(' ')) != 3:
            continue
        
        
        x, y, z = coordinate_str.split(' ')
        x, y, z = int(x), int(y), int(z)
        theta1, theta2, theta3 = ca(x, y, z)
        
        print(f"Moving to position {coordinate_str}, calculated angles: {theta1} {theta2} {theta3}")
        
        servos[0].angle = ta(theta1)
        servos[1].angle = ta(theta2)
        servos[2].angle = ta(theta3)
