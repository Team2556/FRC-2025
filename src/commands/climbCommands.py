from commands2 import Command, Subsystem
from constants import ClimbConstants

class ReelCommand(Command):
    
    def __init__(self, climbSubsystem: Subsystem):
        self.climbSubsystem = climbSubsystem
        self.addRequirements(climbSubsystem)

    def initialize(self):
        ...

    def execute(self):
        self.climbSubsystem.reelRobot()

    def isFinished(self):
        ...


class UnreelCommand(Command):

    def __init__(self, climbSubsystem: Subsystem):
        self.climbSubsystem = climbSubsystem
        self.addRequirements(climbSubsystem)

    def initialize(self):
        ...

    def execute(self):
        self.climbSubsystem.unreelRobot()

    def isFinished(self):
        ...