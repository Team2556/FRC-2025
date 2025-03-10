#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2

import commands2.button, commands2.cmd
import numpy as np
from commands2.sysid import SysIdRoutine

from constants import ClimbConstants
from generated.tuner_constants import TunerConstants
from constants import RobotDimensions, ElevatorConstants
from subsystems import (
    ElevatorSubsystem,
    #coralSubsystem,
    limelightSubsystem,
    pneumaticSubsystem,
    ultrasonic, #ClimbSubsystem
)
from telemetry import Telemetry
from robotUtils import controlAugment

from pathplannerlib.auto import AutoBuilder, PathfindThenFollowPath, PathPlannerAuto
from pathplannerlib.path import PathPlannerPath, PathConstraints
from phoenix6 import swerve
#from phoenix6.hardware import TalonFX
import wpilib
from wpilib import SmartDashboard, DriverStation
from wpimath.geometry import Rotation2d, Translation2d, Transform2d, Pose2d, Rectangle2d
from wpimath.units import (
    rotationsToRadians,
    degrees,
    radians,
    degreesToRadians,
    radiansToDegrees,
    metersToInches,
    inchesToMeters,
)
import wpinet
import math
from commands.odometrySnap2Line import SnapToLineCommand
from commands.liftElevator import LiftElevatorCommand
# from commands import coralCommand
import networktables as nt
from networktables import util as ntutil



# from subsystems import algae


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self.robotWidthBumpered = inchesToMeters(RobotDimensions.WIDTH_w_bumpers)
        SmartDashboard.putNumber("Max Speed", TunerConstants.speed_at_12_volts)
        SmartDashboard.putNumber("Elevator/Kp", ElevatorConstants.kElevatorKp)
        SmartDashboard.putNumber("Elevator/Ki", ElevatorConstants.kElevatorKi)
        SmartDashboard.putNumber("Elevator/Kd", ElevatorConstants.kElevatorKd)
        SmartDashboard.putNumber("Elevator/Kg", ElevatorConstants.kGVolts)
        # SmartDashboard.putNumber("Elevator/Kf",0.0)

        self.timer = wpilib.Timer()
        self.timer.start()

        self._max_speed = SmartDashboard.getNumber(
            "Max Speed", TunerConstants.speed_at_12_volts
        )
        """speed_at_12_volts desired top speed"""
        self._max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.05)
            .with_rotational_deadband(
                self._max_angular_rate * 0.05
            )  # Add a 5% deadband on output
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY #OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive
        )
        """
        Control the drive motor using a velocity closed-loop request.
        The control output type is determined by SwerveModuleConstants.DriveMotorClosedLoopOutput
        """


        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._forward_straight = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY #.OPEN_LOOP_VOLTAGE
            )
        )

        # self.algae = algae.AlgaeHandler()
        self.ultrasonicENABLE = False
        if self.ultrasonicENABLE:
            self.ultrasonic = ultrasonic.Ultrasonic()

        self._logger = Telemetry(self._max_speed)

        # This one's probably used for moving
        self._joystick = commands2.button.CommandXboxController(0)
        # This one's probably used for scoring stuff
        self._joystick2 = commands2.button.CommandXboxController(1)

        # Using NetworkButton with a USB keyboard will require running a seperate python program on the driver's station
        # python ../DriverstationUtils/keyboard_to_nt.py
        # pressing {CTRL+C} will stop the program

        self.drivetrain = TunerConstants.create_drivetrain()

        self.coralENABLE = False
        if self.coralENABLE:
            self.coral_track = coralSubsystem.CoralTrack()
            
        pneumaticENABLE = False
        if pneumaticENABLE:
            self.pneumaticsHub = pneumaticSubsystem.PneumaticSubsystem()

        # self.climb = ClimbSubsystem.ClimbSubsystem()
        # self.one_motor = oneMotor.OneMotor(
        #     motor=[TalonFX(constants.CAN_Address.FOURTEEN),TalonFX(constants.CAN_Address.FIFTEEN)]   )
        # section elevator
        self.ENABLE_ELEVATOR = True
        if self.ENABLE_ELEVATOR:
            self.elevator = ElevatorSubsystem.ElevatorSubsystem()
            self._reset_zero_point_here = self.elevator.reset_zero_point_here()
            self._elevator_motors_break = self.elevator.elevator_motors_break
        # endsection elevator

        # Vision
        self.limelight = limelightSubsystem.LimelightSubsystem()
        for port in np.arange(start=5800, stop=5809):
            wpinet.PortForwarder.getInstance().add(port, "limelight.local", port)

        # self.coral_command = coralCommand.CoralCommand(
        #     self.coral_track, self.pneumaticsHub, self.elevator, self.timer
        # )

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("SetOdo_DriverWallRtFeeder")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

        

        # Configure the button bindings
        self.configureButtonBindings()


    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        # Drivetrain will execute this command periodically

        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -controlAugment.smooth(
                            self._joystick.getLeftY(), exponential_for_curving=3
                        )
                        * self._max_speed
                        * self.invertBlueRedDrive
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -controlAugment.smooth(
                            self._joystick.getLeftX(), exponential_for_curving=3
                        )
                        * self._max_speed
                        * self.invertBlueRedDrive
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -controlAugment.smooth(self._joystick.getRightX())
                        * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                    .with_center_of_rotation(
                        Translation2d(
                            x=self.robotWidthBumpered
                            * 0.2
                            * (
                                controlAugment.smooth(
                                    controlAugment.one_side_control_only(
                                        self._joystick.getRightY(), "Pos"
                                    )
                                )
                            ),
                            # want y translation to depend on direction of turn
                            y=math.copysign(1, self._joystick.getRightX()),
                        )
                        * self.robotWidthBumpered
                        * 0.2
                        * (
                            controlAugment.smooth(
                                controlAugment.one_side_control_only(
                                    self._joystick.getRightY(), "Pos"
                                )
                            )
                        )
                    )
                    # shift the center of rotation to opposite front corner, if the driver pulls down on the right stick in addition to the side.
                    # This should allow some nice defensive roll-off maneuvers
                )
            )
        )

        # self.one_motor.setDefaultCommand(DriveOneMotorCommand(self.one_motor, self._joystick2))
        if self.ENABLE_ELEVATOR:
            self.elevator.setDefaultCommand(
                LiftElevatorCommand(self.elevator, self._joystick2)
            )
            (self._joystick2.start() & self._joystick2.a()).whileTrue(
                self._reset_zero_point_here
            )  # .onFalse(lambda: self._elevator_motors_break) #TODO: fix this to not crash :)
            # (self._joystick2.start() & self._joystick2.x()).whileTrue(lambda: self._reset_zero_point_here) #TODO: fix this to not crash :)

        # section vision related commands
                    
        self._joystick.x().onTrue(SnapToLineCommand(self.drivetrain))

        # endsection vision related commands

        

        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(
                        -self._joystick.getLeftY() * self.invertBlueRedDrive,
                        -self._joystick.getLeftX() * self.invertBlueRedDrive,
                    )
                )
            )
        )
        # trim out the gyro drift; if press POV 0 and move right stick update the drivetrain rotation, but conditional that the right stick input is more than .1
        self._joystick.pov(0).whileTrue(
            commands2.ConditionalCommand(
                self.drivetrain.apply_request(
                    lambda: self.drivetrain.reset_rotation(
                        self.drivetrain.get_rotation3d().toRotation2d()
                        + Rotation2d(
                            0, 0, degreesToRadians(-self._joystick.getRightX())
                        )
                    )
                ),
                self.drivetrain.apply_request(
                    lambda: self._forward_straight.with_velocity_x(0.5).with_velocity_y(
                        0
                    )
                ),
                lambda: self._joystick.getRightX().__abs__() > 0.1,
            )
        )
        self._joystick.pov(180).whileTrue(
            commands2.ConditionalCommand(
                self.drivetrain.apply_request(
                    lambda: self.drivetrain.reset_rotation(
                        self.drivetrain.get_rotation3d().toRotation2d()
                        + Rotation2d(0, 0, degreesToRadians(self._joystick.getRightX()))
                    )
                ),
                self.drivetrain.apply_request(
                    lambda: self._forward_straight.with_velocity_x(
                        -0.5
                    ).with_velocity_y(0)
                ),
                lambda: self._joystick.getRightX().__abs__() > 0.1,
            )
        )

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self._joystick.back() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.back() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._joystick.start() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.start() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        # reset the field-centric heading on left bumper press
        self._joystick.leftBumper().onTrue(
            self.drivetrain.runOnce(
                lambda: self.drivetrain.reset_rotation(
                    Rotation2d(
                        np.pi
                        * (DriverStation.getAlliance() == DriverStation.Alliance.kRed)
                    )
                )
            )
        )
        # self.drivetrain.seed_field_centric()))
        (self._joystick.leftBumper() & self._joystick.a()).onTrue(
            self.drivetrain.runOnce(
                lambda: self.drivetrain.reset_pose_by_zone(zone="a")
            )
        )
        (self._joystick.leftBumper() & self._joystick.b()).onTrue(
            self.drivetrain.runOnce(
                lambda: self.drivetrain.reset_pose_by_zone(zone="b")
            )
        )
        (self._joystick.leftBumper() & self._joystick.x()).onTrue(
            self.drivetrain.runOnce(
                lambda: self.drivetrain.reset_pose_by_zone(zone="x")
            )
        )
        (self._joystick.leftBumper() & self._joystick.y()).onTrue(
            self.drivetrain.runOnce(
                lambda: self.drivetrain.reset_pose_by_zone(zone="y")
            )
        )

       

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return self._auto_chooser.getSelected()
