from commands2 import Command
from phoenix6 import utils
from phoenix6.swerve import SwerveModule
from phoenix6.swerve.requests import FieldCentric
from wpilib import SmartDashboard
from wpimath._controls._controls.controller import PIDController
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Transform2d

from robotUtils.limelight import LimelightHelpers
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from subsystems.vison import VisionSubsystem

from robotUtils.reefOffsets import ReefOffsets
from constants import AprilTagConstants


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
        self.initial_offset = AprilTagConstants.kOrigStandoff #0.5 
        self.initialReached = False

        self.tagID = [17, 18 , 19, 20, 21, 22]
        #17 back right, 18 back, 19 back left, 20 front right, 21 front, 22 front right

        poseList = [
            Pose2d(3.86, 2.9, Rotation2d.fromDegrees(-27)),
            Pose2d(3.26, 3.9, Rotation2d.fromDegrees(-88)),
            Pose2d(3.9, 5.15, Rotation2d.fromDegrees(-145.79)),
            Pose2d(5.1, 4.91, Rotation2d.fromDegrees(152.94)),
            Pose2d(5.93, 3.96, Rotation2d.fromDegrees(94.58)),
            Pose2d(5.06, 2.96, Rotation2d.fromDegrees(33.84))
        ]
        #only shooting from left side currently
        calc_pose_dict = ReefOffsets(extra_left_offset=0,extra_right_offset=0).tag_alignment_poses['robot_left']['poleRight']
        #this "inital pose" is offset in a direction perpendicular to the wall
        useCalcPoseList = [6, 7, 8, 9, 10, 11, 17,18,19,20,21,22]
        calc_reefWaypoints = {tag:calc_pose_dict[tag] for tag in useCalcPoseList}# if tag not in self.tagID}
        # calc_initialReefWaypoints = {tag:calc_initial_pose_dict['poleRight'][tag] for tag in useCalcPoseList}
    




        self.reefWaypoints = {
            i:j for i,j in zip(self.tagID,poseList)
        }
        #merge the dictionaries
        self.reefWaypoints.update(calc_reefWaypoints)
        # self.initialReefWaypoints.update(calc_initialReefWaypoints)
        initialPoseList = [
            pose+(Transform2d(Translation2d(0, -self.initial_offset), Rotation2d())) #looks to shift all to the right (from blue perspective), Why?
            for pose in self.reefWaypoints.values()
        ]

        self.initialReefWaypoints = {
            i: j for i, j in zip(self.reefWaypoints.keys(), initialPoseList)
        }



    def initialize(self):
        self.seen_tag_ID = int(LimelightHelpers.get_fiducial_id("limelight-four"))
        self.initialReached = False
        if utils.is_simulation():
            self.seen_tag_ID = SmartDashboard.getNumber("Seen Tag", 19)
        if not self.seen_tag_ID in self.tagID or self.seen_tag_ID is None:
            SmartDashboard.putNumber("Seen Tag", 0)
            self.initialReached = True
            self.end(True)
        else:
            SmartDashboard.putNumber("Seen Tag", self.seen_tag_ID)
            self.endpose = self.initialReefWaypoints[self.seen_tag_ID]
            self.rotational_pid.setTolerance(1)
            self.x_pid.setTolerance(0.1)
            self.y_pid.setTolerance(0.1)

    def execute(self):
        if not self.seen_tag_ID in self.tagID or self.seen_tag_ID is None:
            self.end(True)
            return
        current_pose = self.swerve.get_state().pose
        self.rotational_rate = self.rotational_pid.calculate(current_pose.rotation().degrees(), self.endpose.rotation().degrees())
        self.velocity_y = self.y_pid.calculate(current_pose.y, self.endpose.y)
        self.velocity_x = self.x_pid.calculate(current_pose.x, self.endpose.x)

        if self.rotational_pid.atSetpoint() and self.y_pid.atSetpoint() and self.x_pid.atSetpoint():
            if not self.initialReached:
                self.initialReached = True
                self.endpose = self.reefWaypoints[self.seen_tag_ID]
                self.rotational_pid.setTolerance(SmartDashboard.getNumber("rotationalPID set tolerance", 0.2))
                self.x_pid.setTolerance(SmartDashboard.getNumber("xPID set tolerance", 0.025))
                self.y_pid.setTolerance(SmartDashboard.getNumber("yPID set tolerance", 0.025))
                self.rotational_rate = self.rotational_pid.calculate(current_pose.rotation().degrees(),self.endpose.rotation().degrees())
                self.velocity_y = self.y_pid.calculate(current_pose.y, self.endpose.y)
                self.velocity_x = self.x_pid.calculate(current_pose.x, self.endpose.x)
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