from commands2 import Command
from wpilib import XboxController, SmartDashboard
from wpimath.controller import PIDController
from wpimath.units import meters, inches, seconds, metersToInches, inchesToMeters
from phoenix6 import hardware
from constants import ElevatorConstants
from math import pi
import numpy as np
import time
from robotUtils import controlAugment

class LiftElevatorCommand(Command):
    def __init__(self, sub_elevator, joystick: XboxController):
        super().__init__()
        self.sub_elevator = sub_elevator
        self.joystick = joystick
        # self.setpoint = self.sub_elevator.setpoint
        self.addRequirements(sub_elevator)
        SmartDashboard.putNumber("Elevator/Move Increment", ElevatorConstants.kincrement_m_per_sec_held)
        self.increment_m_per_sec_held = ElevatorConstants.kincrement_m_per_sec_held
        self.previous_time = None


    def execute(self):
        if not self.previous_time:
            self.previous_time = time.time()
        time_delta = np.min([time.time()-self.previous_time, .02])
        self.increment_m_per_sec_held =  SmartDashboard.getNumber("Elevator/Move Increment", ElevatorConstants.kincrement_m_per_sec_held)

        right_trigger_value = controlAugment.smooth(self.joystick.getRightTriggerAxis(), exponential_for_curving=3, blend=0.5)
        left_trigger_value = controlAugment.smooth(self.joystick.getLeftTriggerAxis(), exponential_for_curving=3, blend=0.5)

        if self.joystick.rightTrigger(threshold=.15):
            self.sub_elevator.update_setpoint( self.increment_m_per_sec_held * right_trigger_value * time_delta)
            self.sub_elevator.moveElevator()
        elif self.joystick.leftTrigger(threshold=.15):
            self.sub_elevator.update_setpoint(- self.increment_m_per_sec_held * left_trigger_value* time_delta)
            self.sub_elevator.moveElevator()
        

        elif self.joystick.a():
            ( self.sub_elevator.update_setpoint((ElevatorConstants.kCoralLv1), incremental=False))
            self.sub_elevator.moveElevator()
        elif self.joystick.b():
            ( self.sub_elevator.update_setpoint((ElevatorConstants.kCoralLv2), incremental=False))
            self.sub_elevator.moveElevator()
        elif self.joystick.y():
            ( self.sub_elevator.update_setpoint((ElevatorConstants.kCoralLv3), incremental=False, constrain=False))
            self.sub_elevator.moveElevator()
        self.previous_time = time.time()
        
        # self.sub_elevator.elevmotor_left.set(.25* controlAugment.smooth(-self.joystick.getRightY()))



    def end(self, interrupted: bool):
        self.sub_elevator.moveElevator(0)