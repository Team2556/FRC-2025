'''Has a command for discharging and a default command for intaking'''

from commands2 import Command
from wpilib import Timer

from subsystems import coralSubsystem, pneumaticSubsystem, elevatorSubsystem
from constants import CoralConstants

class DischargeCoralCommand(Command):
    def __init__(
        self, 
        coralTrack: coralSubsystem.CoralTrack, 
        pneumaticHub: pneumaticSubsystem.PneumaticSubsystem, 
        activateFlippers = False,
    ):
        # Declare subsystems and add requirements
        self.coralTrack = coralTrack
        self.pneumaticHub = pneumaticHub
        self.addRequirements(self.coralTrack, pneumaticHub)
        
        self.activateFlippers = activateFlippers # Whether to activate flippers
        self.timer = Timer() # wpilib.Timer for waiting a bit after no beam brakes detected 
        
        self.left_solenoid_channel = 0
        self.right_solenoid_channel = 1
        
    def initialize(self):
        self.direction = self.getDirection()
        
    def getDirection(self):
        """Get Direction of Discharge using April Tags/Odometry"""
        # TODO Use April Tags to automatically identify the needed direction for discharge
        return 1
        
    def execute(self):
        # Constantly set the motor speed so default command doesn't run
        self.coralTrack.set_motor(CoralConstants.kDischargeMultiplier * self.direction)
        # Start the timer once all beam breaks don't see coral
        if not self.coralTrack.detect_coral:
            self.timer.start()
        # Activate flippers
        if not self.coralTrack.detect_coral and self.activateFlippers:
            self.pneumaticHub.pulse_solenoid(self.right_solenoid_channel, CoralConstants.kSolenoidPulseDuration)
            self.pneumaticHub.pulse_solenoid(self.left_solenoid_channel, CoralConstants.kSolenoidPulseDuration)
    
    def isFinished(self):
        if self.timer.get() >= CoralConstants.kTimeBetweenLeavingBeamBreaksAndDischargingCoral:
            return True

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
            
# for testing
class TestCommand(Command):
    '''For testing yay'''
    def __init__(self, coralTrack: coralSubsystem.CoralTrack):
        # Declare subsystems and add requirements
        self.coralTrack = coralTrack
        self.addRequirements(self.coralTrack)
        
        self.coralTrack.set_motor(0.1)
        
    def end(self):
        self.coralTrack.set_motor(0)