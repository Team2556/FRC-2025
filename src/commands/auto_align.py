import wpilib
from commands2 import Command
from phoenix6.swerve import SwerveModule
from phoenix6.swerve.requests import RobotCentric, Idle
from wpilib import SmartDashboard
# from wpimath._controls._controls.controller import PIDController
from wpimath.controller import PIDController

from robotUtils.limelight import RawFiducial
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.vison import VisionSubsystem

class AutoAlign(Command):
    def __init__(self, drivetrain: CommandSwerveDrivetrain, vision: VisionSubsystem):
        super().__init__()

        # self.printTimer = wpilib.Timer()
        # self.printTimer.start()
        self.drivetrain = drivetrain
        self.vision = vision
        self.tag_id = 22  # Default tag ID

        self.rotational_pid = PIDController(0.05000, 0.000000, 0.001000)

        self.y_pid = PIDController(2.7, 0.004, 0.02)

        self.align_request = RobotCentric().with_drive_request_type(SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        self.idle_request = Idle()

        self.rotational_rate = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0

        self.addRequirements(self.vision)
        self.addRequirements(self.drivetrain)

        SmartDashboard.putNumber("Tag being tracked", 22)
        SmartDashboard.setPersistent("Tag being tracked")


    def initialize(self):
        pass


    def execute(self):
        self.tag_id = SmartDashboard.getNumber("Tag being tracked", 22)
        fiducial = self.vision.get_fiducial_with_id(self.tag_id)

        if fiducial is None:
            self.drivetrain.set_control(self.align_request.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0.0))
            return
            fiducial = RawFiducial()
            fiducial.id = 7  # Example tag ID
            fiducial.txyc = 10  # Random normalized x position
            fiducial.tync = 1  # Random normalized y position
            fiducial.ta = 1.5  # Random target area
            fiducial.dist_to_camera = 1  # 2 meters away from the camera
            fiducial.dist_to_robot = 1  # Slightly farther from the robot center
            fiducial.ambiguity = 0.1  # Low ambiguity (good detection)

        self.rotational_rate = self.rotational_pid.calculate(2 * -1 * fiducial.txyc, 0.0) * 0.675
        self.velocity_y = self.y_pid.calculate(fiducial.dist_to_robot, 0.65) * -1 * 0.7 # Scaled speed factor

        if self.rotational_pid.atSetpoint() and self.y_pid.atSetpoint():
            self.end(True)
            return

        self.drivetrain.set_control(
            self.align_request.with_rotational_rate(-self.rotational_rate)
            .with_velocity_y(self.velocity_y)
        )

        SmartDashboard.putNumber("txyc", fiducial.txyc)
        #print(fiducial.dist_to_robot)
        SmartDashboard.putNumber("rotationalPidController", self.rotational_rate)
        SmartDashboard.putNumber("xPidController", self.velocity_x)

        self.drivetrain.set_control(
            self.align_request.with_rotational_rate(-self.rotational_rate)
            .with_velocity_x(-self.velocity_x)
        )


    def isFinished(self):
        return self.rotational_pid.atSetpoint() and self.y_pid.atSetpoint()


    def end(self, interrupted: bool):
        self.drivetrain.set_control(self.align_request.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0))