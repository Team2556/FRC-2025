from commands2 import Command
from phoenix6 import utils
from phoenix6.swerve import SwerveModule
from phoenix6.swerve.requests import RobotCentric, Idle
from wpilib import SmartDashboard
from wpimath._controls._controls.controller import PIDController

from lib.limelight import LimelightHelpers
from robotUtils.limelight import RawFiducial
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.vison import VisionSubsystem



class AutoAlign(Command):
    def __init__(self, drivetrain: CommandSwerveDrivetrain, vision: VisionSubsystem):
        super().__init__()

        self.drivetrain = drivetrain
        self.vision = vision

        self.rotational_pid = PIDController(0.05000, 0.000000, 0.001000)
        self.x_pid = PIDController(2.7, 0.004, 0.02)

        self.align_request = RobotCentric().with_drive_request_type(SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        self.idle_request = Idle()

        self.rotational_rate = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0

        self.addRequirements(self.vision)
        self.addRequirements(self.drivetrain)

        SmartDashboard.putBoolean("Seen Reef", False)



    def initialize(self):
        pass


    def execute(self, *cameras: str):
        LimelightHelpers.set_LED_to_force_on("limelight")
        fiducial = self.vision.get_reef_fiducial()

        if fiducial is None:
            if not utils.is_simulation():
                SmartDashboard.putBoolean("Seen Reef", False)
                self.drivetrain.set_control(self.align_request.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0.0))
            else:
                # Random sim values
                fiducial = RawFiducial()
                fiducial.txyc = 10
                fiducial.tync = 1
                fiducial.ta = 1.5
                fiducial.dist_to_camera = 1
                fiducial.dist_to_robot = 1
                fiducial.ambiguity = 0.1

        # self.rotational_rate = self.rotational_pid.calculate(2 * -1 * fiducial.txyc, 0.0)
        self.velocity_x = self.x_pid.calculate(fiducial.dist_to_robot, 0.65) * -1
        SmartDashboard.putBoolean("Seen Reef", True)

        if self.x_pid.atSetpoint():
            LimelightHelpers.set_LED_to_force_off("limelight")
            self.end(True)
            return

        self.drivetrain.set_control(
            # self.align_request.with_rotational_rate(-self.rotational_rate)
            self.align_request.with_velocity_x(self.velocity_x)
        )

        SmartDashboard.putNumber("txyc", fiducial.txyc)
        # SmartDashboard.putNumber("rotationalPidController", self.rotational_rate)
        SmartDashboard.putNumber("xPidController", self.velocity_x)





    def isFinished(self):
        return self.x_pid.atSetpoint()
3

    def end(self, interrupted: bool):
        self.drivetrain.set_control(self.align_request.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0))