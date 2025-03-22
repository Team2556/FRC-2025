# Thing that controls the ultrasonic

from wpilib import SmartDashboard, AnalogInput
from constants import AlgaeConstants
from commands2.subsystem import Subsystem
import phoenix6

from constants import UltrasonicConstants

# Not using constants file right now that's s TODO
class UltrasonicSubsystem(Subsystem):
    def __init__(self):
        self.leftSensor = AnalogInput(UltrasonicConstants.kLeftSensorID)
        # self.rightSensor = AnalogInput(UltrasonicConstants.kRightSensorID)
    
    def checkLeftSensor(self):
        return self.leftSensor.getVoltage() <= UltrasonicConstants.kTargetDistance
    
    # def checkRightSensor(self):
    #     return self.rightSensor.getVoltage() <= UltrasonicConstants.kTargetDistance
    
    def periodic(self):
        # One of these two works btw
        SmartDashboard.putNumber("Coral/Left Sensor Value", round(self.leftSensor.getValue(), 2))
        SmartDashboard.putNumber("Coral/Left Sensor Weight", round(self.leftSensor.getVoltage(), 2))
        
        # SmartDashboard.putNumber("Coral/Right Sensor", round(self.rightSensor.getVoltage(), 2))