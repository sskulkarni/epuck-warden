from controller import Robot, Camera

# Initialize the robot and the camera
robot = Robot()

# Time step (in milliseconds)
TIME_STEP = int(robot.getBasicTimeStep())

# Get the camera device and enable it
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)
camera.recognitionEnable(TIME_STEP)
camera_width = camera.getWidth()


leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)


camera.hasRecognition()

# Main loop
while robot.step(TIME_STEP) != -1:
    
    objects = camera.getRecognitionObjects()
    # print(len(objects))


