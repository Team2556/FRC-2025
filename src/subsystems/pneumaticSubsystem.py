import commands2
import wpilib

from constants import PneumaticConstants


class PneumaticSubsystem(commands2.Subsystem):
    def __init__(self):
        self.hub = wpilib.PneumaticHub(PneumaticConstants.kHub)
        self.hub.clearStickyFaults()

        self.hub.enableCompressorAnalog(50, 80)

        self.solenoids = [
            (
                self.hub.makeSolenoid(channel)
                if self.hub.checkSolenoidChannel(channel)
                else False
            )
            for channel in range(16) # Number of solenoids in the hub
        ]

    def enable_solenoid(self, channel):
        """Enables a Solenoid based on channel."""
        solenoid = self.solenoids[channel]

        if solenoid:
            solenoid.set(True)

    def disable_solenoid(self, channel):
        """Disables a Solenoid based on channel."""
        solenoid = self.solenoids[channel]

        if solenoid:
            solenoid.set(False)

    def get_solenoid(self, channel: bool):
        """Returns state of Solenoid based on channel."""
        solenoid = self.solenoids[channel]

        if solenoid:
            solenoid.get()

    def pulse_solenoid(self, channel: bool, duration: float = 0.1):
        """Pulses a Solenoid based on channel."""
        solenoid = self.solenoids[channel]

        if solenoid:
            solenoid.setPulseDuration(duration)
            solenoid.startPulse()

    def toggle_solenoid(self, channel: bool):
        """Returns state of Solenoid based on channel."""
        solenoid = self.solenoids[channel]

        if solenoid:
            solenoid.toggle()