'''Commands that set elevator to a position and increment elevator up or down'''

from commands2 import Command, InterruptionBehavior
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
        self.updateCommandsFinished(0)
        
    def initialize(self):
        self.elevatorSubsystem.update_setpoint(self.position, incremental=False)
        self.elevatorSubsystem.moveElevator()
    
    def updateCommandsFinished(self, increment):
        SmartDashboard.putNumber(
            "Elevator/Commands Finished", 
            SmartDashboard.getNumber("Elevator/Commands Finished", 0) + increment
        )

    def isFinished(self):
        value = self.elevatorSubsystem.get_position()
        return (value <= (self.position + ElevatorConstants.kTargetValueAdder + ElevatorConstants.kTargetValueAccuracy)
            and value >= (self.position + ElevatorConstants.kTargetValueAdder - ElevatorConstants.kTargetValueAccuracy))
    
    def end(self, interrupted):
        self.updateCommandsFinished(1)
    
class HomeElevatorCommand(Command):
    '''Moves elevator down '''
    def __init__(self, elevatorSubsystem: elevatorSubsystem.ElevatorSubsystem):
        self.elevatorSubsystem = elevatorSubsystem
        self.addRequirements(self.elevatorSubsystem)
    
    def initialize(self):
        self.slowedDown = False
        self.elevatorSubsystem.setElevatorSpeed(-1 * ElevatorConstants.kHomingRate)
        
    def execute(self):
        if self.elevatorSubsystem.get_position() < ElevatorConstants.kLowEnoughToSlowDown and not self.slowedDown:
            self.elevatorSubsystem.setElevatorSpeed(
                -1 * ElevatorConstants.kHomingRate * ElevatorConstants.kLowEnoughSpeedMultiplier
            )
            self.slowedDown = True
        
    def isFinished(self): return self.elevatorSubsystem.getLimitBottom()
    
    def end(self, interrupted): 
        self.elevatorSubsystem.setElevatorSpeed(0)
        
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
    
    # def end(self, interrupted): 
    #   pass
        
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
        self.function = function
    
    def execute(self):
        speed = self.function()
        if not speed == 0:
            self.elevatorSubsystem.update_setpoint(self.speed, incremental=True)
            self.elevatorSubsystem.moveElevator()