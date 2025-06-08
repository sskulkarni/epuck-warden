from controller import Robot
import math

class MyCustomRobot(Robot):
    def __init__(self, verbose=True):
        """ Initializes the robot and the required devices. """
        super().__init__()
        self.MAX_SPEED = 6.28
        self.ANGLE_THRESHOLD = 1
        self.DISTANCE_THRESHOLD = 0.002
        self.DISTANCE = 0.1
        self.timestep = 32
        self.verbose = verbose
        self.current_angle = 0
        
    def initialize_devices(self):
        """ Initializes sensors and actuators. """
        # Sensors
        # self.iu = self.getDevice('inertial unit')
        # self.iu.enable(self.timestep)
        # self.gps = self.getDevice('gps')
        # self.gps.enable(self.timestep)
        # Some devices, such as the InertialUnit, need some time to "warm up"
        # self.wait()
        # Actuators
        self.leftMotor = self.getDevice('left wheel motor')
        self.rightMotor = self.getDevice('right wheel motor')
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0.00)
        self.rightMotor.setVelocity(0.00)

    def enable_gps(self,device):
        self.gps = self.getDevice(device)
        self.gps.enable(self.timestep)