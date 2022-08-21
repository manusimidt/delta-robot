import time
from board import SCL, SDA
import busio

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
# You can optionally provide a finer tuned reference clock speed to improve the accuracy of the
# timing pulses. This calibration will be specific to each board and its environment. See the
# calibration.py example in the PCA9685 driver.
# pca = PCA9685(i2c, reference_clock_speed=25630710)
pca.frequency = 50

# To get the full range of the servo you will likely need to adjust the min_pulse and max_pulse to
# match the stall points of the servo.
# This is an example for the Sub-micro servo: https://www.adafruit.com/product/2201
# servo7 = servo.Servo(pca.channels[7], min_pulse=580, max_pulse=2350)
# This is an example for the Micro Servo - High Powered, High Torque Metal Gear:
#   https://www.adafruit.com/product/2307
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2600)
# This is an example for the Standard servo - TowerPro SG-5010 - 5010:
#   https://www.adafruit.com/product/155
# servo7 = servo.Servo(pca.channels[7], min_pulse=400, max_pulse=2400)
# This is an example for the Analog Feedback Servo: https://www.adafruit.com/product/1404
# servo7 = servo.Servo(pca.channels[7], min_pulse=600, max_pulse=2500)
# This is an example for the Micro servo - TowerPro SG-92R: https://www.adafruit.com/product/169
# servo7 = servo.Servo(pca.channels[7], min_pulse=500, max_pulse=2400)

# The pulse range is 750 - 2250 by default. This range typically gives 135 degrees of
# range, but the default is to use 180 degrees. You can specify the expected range if you wish:
# servo7 = servo.Servo(pca.channels[7], actuation_range=135)

min_pulses = [750, 750, 750] 

max_pulses = [2500, 2500, 2500]

servos = [
servo.Servo(pca.channels[0], min_pulse=min_pulses[0], max_pulse=max_pulses[0], actuation_range = 180),
servo.Servo(pca.channels[1], min_pulse=min_pulses[1], max_pulse=max_pulses[1], actuation_range = 180),
servo.Servo(pca.channels[2], min_pulse=min_pulses[2], max_pulse=max_pulses[2], actuation_range = 180),
]


for i in range(len(servos)):
    input(f"Press enter to start calibration for servo {i}" )
    print("Moving servo to 180 degrees")
    servos[i].angle = 180
    
    while True:
        command = input(f'Current pulse: {max_pulses[i]}: Press + or - to adjust the angle so that it is horitontal to the ground: ')

        if len(command)>0 and command[0] == '+':
            increment = max(1, len(command)*10-10)
            max_pulses[i] = max_pulses[i] + increment
            servos[i].set_pulse_width_range(min_pulses[i], max_pulses[i])
            servos[i].angle = 180


        elif len(command)>0 and command[0] == '-':
            decrement = max(1, len(command)*10-10)
            max_pulses[i] = max_pulses[i] - decrement
            servos[i].set_pulse_width_range(min_pulses[i], max_pulses[i])
            servos[i].angle = 180
        else: 
            break

    print("Moving servo to 0 degrees")
    servos[i].angle = 0
    while True:
        command = input(f'Current pulse: {min_pulses[i]}: Press + or - to adjust the angle so that it is horitontal to the ground: ')

        if len(command)>0 and command[0] == '+':
            increment = max(1, len(command)*10-10)
            min_pulses[i] = min_pulses[i] + increment
            servos[i].set_pulse_width_range(min_pulses[i], max_pulses[i])
            servos[i].angle = 0


        elif len(command)>0 and command[0] == '-':
            decrement = max(1, len(command)*10-10)
            min_pulses[i] = min_pulses[i] - decrement
            servos[i].set_pulse_width_range(min_pulses[i], max_pulses[i])
            servos[i].angle = 0
        else: 
            break
        
print("The max_pulses are ", max_pulses)
print("The min_pulses are ", min_pulses)