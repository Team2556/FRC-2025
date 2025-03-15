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
        
    def initialize(self):
        self.elevatorSubsystem.update_setpoint(self.position, incremental=False)
        self.elevatorSubsystem.moveElevator()
    
    def isFinished(self):
        value = self.elevatorSubsystem.rotationsToDistance(
            self.elevatorSubsystem.elevmotor_left.get_position().value
        )
        return (value <= self.position + ElevatorConstants.kTargetValueAccuracy + ElevatorConstants.kTargetValueAdder
            and value >= self.position - ElevatorConstants.kTargetValueAccuracy + ElevatorConstants.kTargetValueAdder)
        
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
    
    # def end(self): self.elevatorSubsystem.elevator_motors_break()
        
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
    
class ElevatorHomeCommand(Command):
    def __init__(self, elevatorSubsystem: elevatorSubsystem.ElevatorSubsystem):
        self.elevatorSubsystem = elevatorSubsystem
        self.addRequirements(self.elevatorSubsystem)
        
    def initialize(self):
        self.elevatorSubsystem.incrementElevator(
            self.elevatorSubsystem.distanceToRotations(ElevatorConstants.kHomingRate)
        )
        
    def isFinished(self):
        if self.elevatorSubsystem.getLimitBottom():
            self.elevatorSubsystem.incrementElevator(0)
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
        
    def updateIncrement(self, increment):
        self.increment = increment
        
# class InstantTestFlipperCommand(Command):
#     def __init__(self, pneumaticsSubsystem: pneumaticSubsystem.PneumaticSubsystem):
#         self.pneumaticsSubsystem = pneumaticsSubsystem
#         self.addRequirements(self.pneumaticsSubsystem)
    
#     def initialize(self):
#         self.pneumaticsSubsystem.pulse_solenoid(0, 1)
#         self.pneumaticsSubsystem.pulse_solenoid(1, 1)
    
#     def isFinished(self): return True