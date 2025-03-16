# Thing that controls the ultrasonic

from wpilib import SmartDashboard, AnalogInput
from constants import AlgaeConstants
from commands2.subsystem import Subsystem
import phoenix6

# Not using constants file
class UltrasonicSubsystem(Subsystem):
    def __init__(self):
        self.sensor1 = AnalogInput(1)
        self.sensor2 = AnalogInput(2)
    
    def checkIfReady(self):
        # TUNE THESE YAY
        ultraSonicDifference = 0.015
        ultraSonicMaxDistance = 0.4
        return (abs(self.sensor1.getVoltage() - self.sensor2.getVoltage()) < ultraSonicDifference
                and self.sensor1.getVoltage() < ultraSonicMaxDistance 
                and self.sensor2.getVoltage() < ultraSonicMaxDistance)
    
    # DONT USE UNLESS IF WIRED BECAUSE PRINT STATEMENTS BREAK THE RADIO
    # I REPEAT DONT USE UNLESS IF WIRED BECAUSE PRINT STATEMENTS BREAK THE RADIO
    def getDistance(self):
        return f"{round(self.sensor1.getVoltage(), 2)}, {round(self.sensor2.getVoltage(), 2)}"