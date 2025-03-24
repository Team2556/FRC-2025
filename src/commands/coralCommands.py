"""Has a command for discharging and a default command for intaking"""

from commands2 import Command

from constants import CoralConstants
from subsystems import coralSubsystem
from subsystems.coralSubsystem import CoralTrack
from subsystems.elevatorSubsystem import ElevatorSubsystem


class DischargeCoralCommand(Command):
    def __init__(self, coralTrack: CoralTrack, elevatorSubsystem: ElevatorSubsystem,
                 direction=1):
        super().__init__()
        # Declare subsystems and add requirements
        self.coralTrack = coralTrack
        self.addRequirements(self.coralTrack)
        
        self.elevatorSubsystem = elevatorSubsystem # Not a requirement; just used for getting position
        
        self.direction = direction # Left is -1, Right is 1 
        
        self.left_solenoid_channel = CoralConstants.kLeftFlipper
        self.right_solenoid_channel = CoralConstants.kRightFlipper
        
    def getDirection(self):
        """Get Direction of Discharge using April Tags/Odometry"""
        # TODO Use April Tags to automatically identify the needed direction for discharge
        return self.direction # Right now just manually find the direction
        
    def execute(self):
        # Constantly set the motor speed so default command doesn't run
        self.coralTrack.set_motor(CoralConstants.kDischargeMultiplier * self.getDirection())
        # Check for flippers

class CoralDefaultCommand(Command):
    """The default command for coral... it does all the centering"""
    def __init__(self, coralSubsystem: coralSubsystem.CoralTrack):
        super().__init__()
        # Declare subsystem and add requirement
        self.coralSubsystem = coralSubsystem
        self.addRequirements(self.coralSubsystem)
        
    def execute(self):
        # Look guys it's Aidan's original code v4
        is_left = self.coralSubsystem.left_detector.get()
        is_right = self.coralSubsystem.right_detector.get()

        if is_left and not is_right:
            self.coralSubsystem.set_motor(1 * CoralConstants.kIntakeMultiplier)
        elif is_right and not is_left:
            self.coralSubsystem.set_motor(-1 * CoralConstants.kIntakeMultiplier)
        else:
            self.coralSubsystem.disable_motor()