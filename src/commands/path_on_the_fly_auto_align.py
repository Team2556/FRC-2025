from typing import List

from commands2 import Command
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState
from phoenix6 import utils
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Transform2d
import math

from robotUtils.limelight import LimelightHelpers

from commands2 import Command
from phoenix6.swerve import SwerveModule
from phoenix6.swerve.requests import RobotCentric, Idle, FieldCentric
from wpilib import SmartDashboard
from wpimath._controls._controls.controller import PIDController

from lib.limelight import RawFiducial
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.vison import VisionSubsystem

def get_fiducial_id(limelight_name):
    pass


class PathOnTheFlyAutoAlign(Command):
    def __init__(self, drivetrain: CommandSwerveDrivetrain, vision: VisionSubsystem):
        super().__init__()

        self.rotational_pid = PIDController(0.05000, 0.000000, 0.001000)
        self.x_pid = PIDController(1.5, 0.004, 0.0)
        self.y_pid = PIDController(1.5, 0.004, 0.0)
        self.rotational_pid.enableContinuousInput(0, 360)
        self.rotational_rate = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.vision = vision
        self.swerve = drivetrain
        self.seen_tag_ID = 22
        self.align_request = FieldCentric().with_drive_request_type(SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
        self.endpose = Pose2d(0,0,0)
        self.addRequirements(self.vision)
        self.addRequirements(self.swerve)
        self.initial_offset = 0.5
        self.initialReached = False

        self.tagID = [17, 18 , 19, 20, 21, 22]
        #17 back right, 18 back, 19 back left, 20 front right, 21 front, 22 front right

        poseList = [
            Pose2d(3.776, 2.970, Rotation2d.fromDegrees(60-90)),
            Pose2d(3.249, 4.049, Rotation2d.fromDegrees(0-90)),
            Pose2d(3.944, 5.176, Rotation2d.fromDegrees(-60-90)),
            Pose2d(5.227, 5.080, Rotation2d.fromDegrees(-120-90)),
            Pose2d(5.754, 3.905, Rotation2d.fromDegrees(180-90)),
            Pose2d(5.06+0.05, 2.96+0.05, Rotation2d.fromDegrees(33.84))
        ]

        initialPoseList = [
            pose+(Transform2d(Translation2d(0, -self.initial_offset), Rotation2d()))
            for pose in poseList
        ]

        self.reefWaypoints = {
            i:j for i,j in zip(self.tagID,poseList)
        }

        self.initiallReefWaypoints = {
            i: j for i, j in zip(self.tagID, initialPoseList)
        }


    def initialize(self):
        self.seen_tag_ID = int(LimelightHelpers.get_fiducial_id("limelight-four"))
        self.initialReached = False
        if utils.is_simulation():
            self.seen_tag_ID = SmartDashboard.getNumber("Seen Tag", 18)
        if not self.seen_tag_ID in self.tagID or self.seen_tag_ID is None:
            SmartDashboard.putNumber("Seen Tag", 0)
            self.initialReached = True
            self.end(True)
        else:
            SmartDashboard.putNumber("Seen Tag", self.seen_tag_ID)
            self.endpose = self.initiallReefWaypoints[self.seen_tag_ID]
            self.rotational_pid.setTolerance(1)
            self.x_pid.setTolerance(0.1)
            self.y_pid.setTolerance(0.1)

    def execute(self):
        if not self.seen_tag_ID in self.tagID or self.seen_tag_ID is None:
            self.end(True)
            return
        currentPose = self.swerve.get_state().pose
        self.rotational_rate = self.rotational_pid.calculate(currentPose.rotation().degrees(), self.endpose.rotation().degrees())
        self.velocity_y = self.y_pid.calculate(currentPose.y, self.endpose.y)
        self.velocity_x = self.x_pid.calculate(currentPose.x, self.endpose.x)

        if self.rotational_pid.atSetpoint() and self.y_pid.atSetpoint() and self.x_pid.atSetpoint():
            if not self.initialReached:
                self.initialReached = True
                self.endpose = self.reefWaypoints[self.seen_tag_ID]
                self.rotational_pid.setTolerance(SmartDashboard.getNumber("rotationalPID set tolerance", 0.2))
                self.x_pid.setTolerance(SmartDashboard.getNumber("xPID set tolerance", 0.025))
                self.y_pid.setTolerance(SmartDashboard.getNumber("yPID set tolerance", 0.025))
                self.rotational_rate = self.rotational_pid.calculate(currentPose.rotation().degrees(),self.endpose.rotation().degrees())
                self.velocity_y = self.y_pid.calculate(currentPose.y, self.endpose.y)
                self.velocity_x = self.x_pid.calculate(currentPose.x, self.endpose.x)
            else:
                self.end(True)
                return

        SmartDashboard.putBoolean("initialReached", self.initialReached)
        SmartDashboard.putNumber("rotationalPidController", self.rotational_rate)
        SmartDashboard.putNumber("xPidController", self.velocity_x)
        SmartDashboard.putNumber("yPidController", self.velocity_y)
        SmartDashboard.putNumber("rotationalPID set tolerance", 0.2)
        SmartDashboard.putNumber("xPID set tolerance", 0.025)
        SmartDashboard.putNumber("yPID set tolerance", 0.025)

        self.swerve.set_control(
            self.align_request.with_rotational_rate(self.rotational_rate)
            .with_velocity_x(self.velocity_x)
            .with_velocity_y(self.velocity_y)
        )

    def isFinished(self):
        return self.rotational_pid.atSetpoint() and self.y_pid.atSetpoint() and self.x_pid.atSetpoint() and self.initialReached == True

    def end(self, interrupted: bool):
        self.swerve.set_control(self.align_request.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0))