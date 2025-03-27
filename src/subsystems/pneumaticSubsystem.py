import commands2
import wpilib
from wpilib import SmartDashboard
from constants import PneumaticConstants


class PneumaticSubsystem(commands2.Subsystem):
    def __init__(self):
        self.hub = wpilib.PneumaticHub(PneumaticConstants.kHub)
        self.hub.clearStickyFaults()

        self.hub.enableCompressorAnalog(50, 60)
        # self.hub.enableCompressorDigital()

        self.solenoids = [
            (
                self.hub.makeSolenoid(channel)
                if self.hub.checkSolenoidChannel(channel)
                else False
            )
            for channel in range(16) # Number of solenoid channels in the hub
        ]
        self.solenoids_states = [sol.get for sol in self.solenoids]
        # SmartDashboard.putBooleanArray("Pneumatics/Compressor States", self.solenoids_states)

    def periodic(self):
        '''update solenoid states on smartdashboard'''
        self.solenoids_states = [sol.get() for sol in self.solenoids]
        # SmartDashboard.putBooleanArray("Pneumatics/Compressor Sates", self.solenoids_states)
        
    def activateFlippers(self):
        SmartDashboard.putBoolean("Coral/Pneumatics Activated", True)
        self.enable_solenoid(PneumaticConstants.kRightScoreSolenoid)
        self.enable_solenoid(PneumaticConstants.kLeftScoreSolenoid)
        
        self.disable_solenoid(PneumaticConstants.kRightRetractSolenoid)
        self.disable_solenoid(PneumaticConstants.kLeftRetractSolenoid)
    
    def disableFlippers(self):
        SmartDashboard.putBoolean("Coral/Pneumatics Activated", False)
        self.disable_solenoid(PneumaticConstants.kLeftScoreSolenoid)
        self.disable_solenoid(PneumaticConstants.kRightScoreSolenoid)
        
        self.enable_solenoid(PneumaticConstants.kLeftRetractSolenoid)
        self.enable_solenoid(PneumaticConstants.kRightRetractSolenoid)
        
    def enable_solenoid(self, channel):
        """Enables a Solenoid based on channel."""
        solenoid = self.solenoids[channel]
        if not solenoid: return
        
        solenoid.set(True)

    def disable_solenoid(self, channel):
        """Disables a Solenoid based on channel."""
        solenoid = self.solenoids[channel]
        if not solenoid: return 
        
        solenoid.set(False)

    def get_solenoid(self, channel: bool):
        """Returns state of Solenoid based on channel."""
        solenoid = self.solenoids[channel]
        if solenoid: return
        
        solenoid.get()

    def pulse_solenoid(self, channel: bool, duration: float = 0.1):
        """Pulses a Solenoid based on channel."""
        solenoid = self.solenoids[channel]
        if not solenoid: return
        
        solenoid.setPulseDuration(duration)
        solenoid.startPulse()

    def toggle_solenoid(self, channel: bool):
        """Returns state of Solenoid based on channel."""
        solenoid = self.solenoids[channel]
        if not solenoid: return
        
        solenoid.toggle()

    def disable_all_solenoids(self):
        """Disables all Solenoids."""
        for solenoid in self.solenoids:
            if not solenoid: continue
            
            solenoid.set(False)
    
    def simple_toggle_all(self):
        ''''toggle current values of solenoids'''
        for solenoid in self.solenoids:
            if not solenoid: continue
            solenoid.toggle()