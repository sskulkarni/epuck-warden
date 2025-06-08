import sys
import os
import math
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

from utilities import MyCustomRobot
robot = MyCustomRobot(verbose=False)
robot.initialize_devices()
robot.enable_gps('pedestrian2_gps')

timestep = int(robot.getBasicTimeStep())

# Sensors
# iu = robot.getDevice('inertial unit')
# iu.enable(timestep)

gps = robot.getDevice('pedestrian2_gps')
gps.enable(timestep)

# Set target position (x, z)
target = [0.1, 0.0]  # You can change this to your desired point
def distance_to_target(pos, target):
    return ((pos[0] - target[0]) ** 2 + (pos[2] - target[1]) ** 2) ** 0.5

while robot.step(timestep) != -1:




    pos = gps.getValues()
    distance = distance_to_target(pos, target)

    if distance < 0.05:
        # Reached the target
        robot.leftMotor.setVelocity(0)
        robot.rightMotor.setVelocity(0)
        print("Reached target")
        break

    # Proportional speed control
    speed = 3.0
    robot.leftMotor.setVelocity(speed)
    robot.rightMotor.setVelocity(speed)




