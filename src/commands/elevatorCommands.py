'''Commands that set elevator to a position and increment elevator up or down'''

from commands2 import Command
from wpilib import XboxController, SmartDashboard
from wpimath.controller import PIDController
from wpimath.units import meters, inches, seconds, metersToInches, inchesToMeters
from phoenix6 import hardware
from constants import ElevatorConstants
from math import pi
import numpy as np
import time
from robotUtils import controlAugment
from subsystems import elevatorSubsystem

class SetElevatorCommand(Command):
    def __init__(self, elevatorSubsystem: elevatorSubsystem.ElevatorSubsystem, position):
        self.elevatorSubsystem = elevatorSubsystem
        self.addRequirements(self.elevatorSubsystem)
        self.position = position
        
    def initialize(self):
        self.elevatorSubsystem.update_setpoint(self.position, incremental=False)
        self.elevatorSubsystem.moveElevator()
    
    def isFinished(self):
        value = self.elevatorSubsystem.elevmotor_left.get_position()
        return (value <= self.position + ElevatorConstants.kTargetValueAccuracy
            and value >= self.position - ElevatorConstants.kTargetValueAccuracy)
        
class InstantSetElevatorCommand(Command):
    def __init__(self, elevatorSubsystem: elevatorSubsystem.ElevatorSubsystem, position):
        self.elevatorSubsystem = elevatorSubsystem
        self.addRequirements(self.elevatorSubsystem)
        self.position = position
        
    def initialize(self):
        self.elevatorSubsystem.update_setpoint(self.position, incremental=False)
        self.elevatorSubsystem.moveElevator()
                
    def isFinished(self):
        return True
        
class IncrementElevatorCommand(Command):
    def __init__(self, elevatorSubsystem: elevatorSubsystem.ElevatorSubsystem, amount):
        self.elevatorSubsystem = elevatorSubsystem
        self.addRequirements(self.elevatorSubsystem)
        self.amount = amount
        
    def initialize(self):
        self.elevatorSubsystem.update_setpoint(self.amount, incremental=True)
        self.elevatorSubsystem.moveElevator()
    
    def isFinished(self):
        # This just increments so it should automatically finish (but we can add a timer if otherwise)
        return True
    
# Here are the bad commands that work so we're keeping them
class ContinuousIncrementCommand(Command):
    def __init__(self, elevatorSubsystem: elevatorSubsystem.ElevatorSubsystem, function):
        self.elevatorSubsystem = elevatorSubsystem
        self.addRequirements(self.elevatorSubsystem)
        self.increment = 0
        self.function = function
    
    def execute(self):
        self.elevatorSubsystem.incrementElevator(self.function())
        # print(self.function(), self.elevatorSubsystem.get_position())
        
    def updateIncrement(self, increment):
        self.increment = increment