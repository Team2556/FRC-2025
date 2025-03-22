from commands2 import Command, InterruptionBehavior
from wpilib import Timer

from subsystems import coralSubsystem, elevatorSubsystem, pneumaticSubsystem
from constants import PneumaticConstants


class DefaultPneumaticCommand(Command):
    def __init__(self, pneumaticSubsystem: pneumaticSubsystem.PneumaticSubsystem):
        self.pneumaticSubsystem = pneumaticSubsystem
        self.addRequirements(self.pneumaticSubsystem)

        self.InterruptionBehavior = InterruptionBehavior.kCancelIncoming

    def initialize(self):
        self.pneumaticSubsystem.disable_solenoid(PneumaticConstants.kRightScoreSolenoid)
        self.pneumaticSubsystem.enable_solenoid(
            PneumaticConstants.kRightRetractSolenoid
        )

        self.pneumaticSubsystem.disable_solenoid(PneumaticConstants.kLeftScoreSolenoid)
        self.pneumaticSubsystem.enable_solenoid(PneumaticConstants.kLeftRetractSolenoid)


class PulseFlippersCommand(Command):
    def __init__(self, pneumaticSubsystem: pneumaticSubsystem.PneumaticSubsystem):
        self.pneumaticSubsystem = pneumaticSubsystem
        self.addRequirements(self.pneumaticSubsystem)

        self.InterruptionBehavior = InterruptionBehavior.kCancelSelf
        self.timer = Timer()

        self.pulse_duration = 1

    def initialize(self):
        self.timer.reset()
        self.timer.start()

    def execute(self):
        if self.timer.get() < self.pulse_duration:
            # Extend the flippers
            self.pneumaticSubsystem.enable_solenoid(
                PneumaticConstants.kRightScoreSolenoid
            )
            self.pneumaticSubsystem.enable_solenoid(
                PneumaticConstants.kLeftScoreSolenoid
            )

            self.pneumaticSubsystem.disable_solenoid(
                PneumaticConstants.kRightRetractSolenoid
            )
            self.pneumaticSubsystem.disable_solenoid(
                PneumaticConstants.kLeftRetractSolenoid
            )

    def isFinished(self):
        return self.timer.get() > self.pulse_duration

    def end(self, interrupted):
        self.timer.stop()
