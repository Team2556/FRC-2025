from commands2 import Command, Subsystem
from constants import ClimbConstants
from phoenix6 import signals

class Forward(Command):
    
    def __init__(self, climbSubsystem: Subsystem):
        self.climbSubsystem = climbSubsystem
        self.addRequirements(climbSubsystem)

    def initialize(self):
        ...

    def execute(self):
        self.climbSubsystem.forward()

    def isFinished(self):
        if self.climbSubsystem.climbMotor.get_forward_limit().value is signals.ForwardLimitValue.CLOSED_TO_GROUND:
            # do action when forward limit is closed
            ...
        if self.climbSubsystem.climbMotor.get_fault_forward_soft_limit().value:
            # do action when forward soft limit is reached
            ...


class Backward(Command):

    def __init__(self, climbSubsystem: Subsystem):
        self.climbSubsystem = climbSubsystem
        self.addRequirements(climbSubsystem)

    def initialize(self):
        ...

    def execute(self):
        self.climbSubsystem.backward()

    def isFinished(self):
        ...