from commands2 import Command, InterruptionBehavior
from wpilib import Timer, SmartDashboard

from subsystems import coralSubsystem, elevatorSubsystem, pneumaticSubsystem
from constants import PneumaticConstants, ElevatorConstants


class DefaultPneumaticCommand(Command):
    def __init__(
        self, 
        pneumaticSubsystem: pneumaticSubsystem.PneumaticSubsystem,
        elevatorSubsystem: elevatorSubsystem.ElevatorSubsystem,
        coralSubsystem: coralSubsystem.CoralTrack,
    ):
        self.pneumaticSubsystem = pneumaticSubsystem
        self.elevatorSubsystem = elevatorSubsystem
        self.coralSubsystem = coralSubsystem

        # Only require pneumatics because it's just getting the data of the other two
        self.addRequirements(self.pneumaticSubsystem)

        self.InterruptionBehavior = InterruptionBehavior.kCancelIncoming
        
        # For getting delay between coral leaving beam breaks and activating
        self.timer = Timer()
        
        SmartDashboard.putNumber("Coral/Pneumatics Delay", PneumaticConstants.kScoreDelay)

    def initialize(self):
        # self.disablePneumatics()
        pass

    def execute(self):
        if (self.elevatorSubsystem.get_position() > ElevatorConstants.kCoralLv4 - 3 
            and not self.coralSubsystem.detect_coral() and self.coralSubsystem.coralFiring):
            if not self.timer.isRunning():
                self.timer.start()
            if self.timer.get() >= PneumaticConstants.kScoreDelay:
                self.pneumaticSubsystem.activateFlippers()
            else:
                self.pneumaticSubsystem.disableFlippers()
        else:
            self.timer.reset()
            self.pneumaticSubsystem.disableFlippers()
        SmartDashboard.putNumber("Coral/Pneumatics Timer", self.timer.get())
        PneumaticConstants.kScoreDelay = SmartDashboard.getNumber("Coral/Pneumatics Delay", PneumaticConstants.kScoreDelay)
            

class PulseFlippersCommand(Command):
    def __init__(self, pneumaticSubsystem: pneumaticSubsystem.PneumaticSubsystem):
        self.pneumaticSubsystem = pneumaticSubsystem
        self.addRequirements(self.pneumaticSubsystem)

        self.InterruptionBehavior = InterruptionBehavior.kCancelIncoming
        # self.timer = Timer()

        # self.pulse_duration = 0.5

    def initialize(self):
        pass
        # self.timer.reset()
        # self.timer.start()

    def execute(self):
        # if self.timer.get() < self.pulse_duration:
        SmartDashboard.putBoolean("Coral/Pneumatics Activated", True)
        # Extend the flippers
        self.LeftFlippersUp()
        self.RightFlippersUp()


    # def isFinished(self):
    #     # return self.timer.get() > self.pulse_duration
    #     return True

    def end(self, interrupted):
        self.LeftFlippersDown()
        self.RightFlippersDown()
        # self.timer.stop()


    def LeftFlippersUp(self):
        self.pneumaticSubsystem.enable_solenoid(
            PneumaticConstants.kLeftScoreSolenoid
        )
        self.pneumaticSubsystem.disable_solenoid(
            PneumaticConstants.kLeftRetractSolenoid
        )
    def RightFlippersUp(self):

        self.pneumaticSubsystem.enable_solenoid(
            PneumaticConstants.kRightScoreSolenoid
        )
        self.pneumaticSubsystem.disable_solenoid(
            PneumaticConstants.kRightRetractSolenoid
        )

    def LeftFlippersDown(self):
        self.pneumaticSubsystem.disable_solenoid(
            PneumaticConstants.kLeftScoreSolenoid
        )
        self.pneumaticSubsystem.enable_solenoid(
            PneumaticConstants.kLeftRetractSolenoid
        )

    def RightFlippersDown(self):
        self.pneumaticSubsystem.disable_solenoid(
            PneumaticConstants.kRightScoreSolenoid
        )
        self.pneumaticSubsystem.enable_solenoid(
            PneumaticConstants.kRightRetractSolenoid
        )