from commands2 import Command
from wpilib import XboxController
from wpimath.controller import PIDController
from phoenix6 import hardware

class DriveOneMotorCommand(Command):
    def __init__(self,sub_one_motor, joystick: XboxController):
        super().__init__()
        self.sub_one_motor = sub_one_motor
        self.joystick = joystick
        self.addRequirements(sub_one_motor)
        self.limit = 0.1

    def execute(self):
        right_y = self.joystick.getRightY()
        self.sub_one_motor.set(-right_y*self.limit)

    def end(self, interrupted: bool):
        self.sub_one_motor.set(0)