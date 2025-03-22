'''The coral subsystem yay'''

from wpilib import DigitalInput
from commands2 import Subsystem
from rev import SparkMaxConfig, SparkBase, SparkFlex

from constants import CoralConstants

class CoralTrack(Subsystem):
    def __init__(self):
        self.motor_controller = SparkFlex(
            CoralConstants.kCoralMotorPort, SparkFlex.MotorType.kBrushless
        )
        # self.motor_controller.configure(
        #     SparkMaxConfig(),
        #     SparkBase.ResetMode.kResetSafeParameters,
        #     SparkBase.PersistMode.kPersistParameters,
        # )

        self.left_detector = DigitalInput(CoralConstants.kLeftBreakerLight)
        self.right_detector = DigitalInput(CoralConstants.kRightBreakerLight)

    def set_motor(self, speed):
        """Sets Coral Track motor to a specific speed"""
        self.motor_controller.set(speed)

    def disable_motor(self):
        """Disables Coral Track motor"""
        self.motor_controller.set(0)

    def get_detectors(self):
        """Returns a list of the states of the three sensors"""
        # Not currently being used
        return [
            self.left_detector.get(),
            self.right_detector.get(),
        ]
    
    def detect_coral(self):
        """Returns True if Coral detected on track"""
        return self.left_detector.get() or self.right_detector.get()
    
    def periodic(self):
        ...# TODO: Add SmartDashboard tining HERE NOW or not