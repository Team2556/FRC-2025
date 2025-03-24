from commands2 import Command
from phoenix6 import signals

from subsystems.climbSubsystem import ClimbSubsystem


class Forward(Command):
    
    def __init__(self, climbSubsystem: ClimbSubsystem):
        self.climbSubsystem = climbSubsystem
        self.addRequirements(climbSubsystem)

    def initialize(self):
        ...

    def execute(self):
        self.climbSubsystem.forward()

    def isFinished(self):
        return self.climbSubsystem.climbMotor.get_forward_limit().value == signals.ForwardLimitValue.CLOSED_TO_GROUND
            # do action when forward limit is closed
            # ...
        # if self.climbSubsystem.climbMotor.get_fault_forward_soft_limit().value:
            # do action when forward soft limit is reached
            # ...
    def end(self, interrupted):
        self.climbSubsystem.stop()


class Backward(Command):

    def __init__(self, climbSubsystem: ClimbSubsystem):
        self.climbSubsystem = climbSubsystem
        self.addRequirements(climbSubsystem)

    def initialize(self):
        ...

    def execute(self):
        self.climbSubsystem.backward()

    def isFinished(self):
        return self.climbSubsystem.climbMotor.get_reverse_limit().value == signals.ForwardLimitValue.CLOSED_TO_GROUND

    def end(self, interrupted):
        self.climbSubsystem.stop()