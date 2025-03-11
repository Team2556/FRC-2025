import commands2
import wpilib

from subsystems import coralSubsystem, pneumaticSubsystem, ElevatorSubsystem


class CoralCommand(commands2.Command):
    def __init__(
        self,
        coralTrack: coralSubsystem.CoralTrack,
        pneumaticHub: pneumaticSubsystem.PneumaticSubsystem,
        elevator: ElevatorSubsystem.ElevatorSubsystem,
        timer: wpilib.Timer,
    ):
        super().__init__()
        self.coral_track = coralTrack
        self.pneumatic_hub = pneumaticHub
        self.elevator = elevator
        self.addRequirements(self.coral_track, self.pneumatic_hub, self.elevator)

        self.timer = timer

        self.discharge_enabled = False
        self.discharge_start_time = 0
        self.discharge_direction = 0  # 0: None | -1: Left | 1: Right
        
        self.left_solenoid_channel = 0
        self.right_solenoid_channel = 1

    def execute(self):
        """Defualt Command for Coral Track"""
        
        # If Discharge Sequence Disables
        if not self.discharge_enabled:
            self.coral_track.center_coral()
            return

        # If Discharge Sequence Over
        if self.timer.get() - self.discharge_start_time > 1.5:
            self.disable_discharge()
            return

        # If Not Targeting L4
        L4_height = 10  # TODO Find height for when targeting L4
        if self.elevator.get_position() < L4_height:
            return
        
        # If Center Detector Enabled
        if self.coral_track.get_detectors()[1]:
            return
        
        self.pneumatic_hub.pulse_solenoid(self.right_solenoid_channel, 1)
        self.pneumatic_hub.pulse_solenoid(self.left_solenoid_channel, 1)
        
    def enable_discharge(self):
        '''Enables the Discharge Sequence'''
        self.discharge_direction = self.get_direction()
        self.discharge_start_time = self.timer.get()
        self.discharge_enabled = True
        
        discharge_speed = 0.08
        self.coral_track.set_motor(discharge_speed*self.discharge_direction)


    def disable_discharge(self):
        """Disables Discharge Sequence"""
        self.discharge_direction = 0
        self.discharge_enabled = False
        self.coral_track.disable_motor()
    
    def get_direction(self):
        """Get Direction of Discharge using April Tags"""
        # TODO Use April Tags to automatically identify the needed direction for discharge
        return 1
