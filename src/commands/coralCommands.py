'''Has a command for discharging and a default command for intaking'''

from commands2 import Command, InterruptionBehavior
from wpilib import Timer

from subsystems import coralSubsystem, elevatorSubsystem
from constants import CoralConstants

class DischargeCoralCommand(Command):
    def __init__(
        self, 
        coralTrack: coralSubsystem.CoralTrack, 
        elevatorSubsystem: elevatorSubsystem.ElevatorSubsystem,
        direction = 1,
    ):
        # Declare subsystems and add requirements
        self.coralTrack = coralTrack
        self.elevatorSubsystem = elevatorSubsystem # Not a requirement; just used for getting position
        self.addRequirements(self.coralTrack)
        
        
        self.direction = direction # Left is -1, Right is 1 
        
    def getDirection(self):
        """Get Direction of Discharge using April Tags/Odometry"""
        # TODO Use April Tags to automatically identify the needed direction for discharge
        return self.direction # Right now just manually find the direction
        
    def execute(self):
        # Constantly set the motor speed so default command doesn't run
        self.coralTrack.set_motor(CoralConstants.kDischargeMultiplier * self.getDirection())
        # Check for flippers

class CoralDefaultCommand(Command):
    '''The default command for coral... it does all the centering'''
    def __init__(self, coralSubsystem: coralSubsystem.CoralTrack):
        # Declare subsystems and add requirements
        self.coralSubsystem = coralSubsystem
        self.addRequirements(self.coralSubsystem)
        
    def execute(self):
        # Look guys it's Aidan's original code v4
        is_Left = self.coralSubsystem.left_detector.get()
        is_Right = self.coralSubsystem.right_detector.get()

        if is_Left and not is_Right:
            self.coralSubsystem.set_motor(1 * CoralConstants.kIntakeMultiplier)
        elif is_Right and not is_Left:
            self.coralSubsystem.set_motor(-1 * CoralConstants.kIntakeMultiplier)
        else:
            self.coralSubsystem.disable_motor()