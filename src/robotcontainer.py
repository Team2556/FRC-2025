#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import math

import commands2
import commands2.button
import commands2.cmd
from commands2.cmd import runOnce
from pathplannerlib.auto import AutoBuilder
from phoenix6 import swerve
from phoenix6.swerve import Pose2d, SwerveModule
from phoenix6.swerve.requests import FieldCentricFacingAngle
from wpilib import SmartDashboard, DriverStation, CameraServer
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

# NOTE: THIS IS THE OFFICIAL LOCATION FOR IMPORTING COMMANDS AND SUBSYSTEMS AND CONSTANTS
from subsystems import (
    algaeSubsystem,
    elevatorSubsystem,
    coralSubsystem,
    climbSubsystem,
    pneumaticSubsystem,
)

from commands import (
    algaeCommands,
    coralCommands,
    elevatorCommands,
    climbCommands,
    pneumaticCommands
)

from pathplannerlib.auto import AutoBuilder, PathfindThenFollowPath, PathPlannerAuto, NamedCommands
from pathplannerlib.path import PathPlannerPath, PathConstraints

from commands.path_on_the_fly_auto_align import PathOnTheFlyAutoAlign
from constants import ElevatorConstants, AlgaeConstants, Override_DriveConstant
from generated.tuner_constants import TunerConstants
from robotUtils.adjustJoystick import adjust_jostick

from subsystems import vison
from subsystems.vison import VisionSubsystem
from telemetry import Telemetry

from phoenix6 import swerve
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians
from wpilib import SmartDashboard
from wpilib.cameraserver import CameraServer


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        AutoBuilder._configured = False
        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._field_centric_drive = (
            swerve.requests.FieldCentric()
            .with_deadband(0.01)
            .with_rotational_deadband(0.01)  # Add a 1% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._robot_centric_drive = (
            swerve.requests.RobotCentric()
            .with_deadband(0.001)
            .with_rotational_deadband(0.001)  # Add a 1% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self.field_centric_angle_lock = (FieldCentricFacingAngle()
                                         .with_heading_pid(5, 0, 0)
                                         .with_max_abs_rotational_rate(2 * math.pi)
                                         .with_drive_request_type(SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
                                         .with_deadband(0.01)
                                         )
        # self.slow_mode_multiplier = 1.0 # 1 or kSlowMode constant

        self._logger = Telemetry(self._max_speed)

        self._joystick = commands2.button.CommandXboxController(0)
        self._joystick2 = commands2.button.CommandXboxController(1)  # FOR TESTING

        self.drivetrain = TunerConstants.create_drivetrain()

        self.vision = VisionSubsystem(self.drivetrain, "limelight-four")
        # NOTE: HAVE ALL THE ENABLY THINGS HERE (and change them all to true when actually playing)

        self.ENABLE_ALGAE = True
        self.ENABLE_ELEVATOR = True
        self.ENABLE_CORAL = True
        self.ENABLE_CLIMB = True
        self.ENABLE_PNEUMATIC = True
        self.ENABLE_VISON = True

        # NOTE: DECLARE ALL SUBSYSTEMS HERE AND NOWHERE ELSE PLEASE

        if self.ENABLE_ALGAE:
            self.algaeSubsystem = algaeSubsystem.AlgaeSubsystem()

        if self.ENABLE_ELEVATOR:
            self.elevatorSubsystem = elevatorSubsystem.ElevatorSubsystem()

        if self.ENABLE_CORAL:
            self.coralSubsystem = coralSubsystem.CoralTrack()

        if self.ENABLE_CLIMB:
            self.climbSubsystem = climbSubsystem.ClimbSubsystem()
        
        if self.ENABLE_PNEUMATIC:
            self.pneumaticSubsystem = pneumaticSubsystem.PneumaticSubsystem()

        if self.ENABLE_VISON:
            self.vision = vison.VisionSubsystem(self.drivetrain)

        # Configure the button bindings
        self.configureButtonBindings()

        # Make Auto Command so it actually corals
        self.Coral1 = (coralCommands.DischargeCoralCommand(
                self.coralSubsystem,
                # self.elevatorSubsystem,
                direction = -1,  # Left is -1, Right is 1
            ))

        # Path follower
        NamedCommands.registerCommand("dischargeCoralLeftCommand", 
            coralCommands.DischargeCoralCommand(
                self.coralSubsystem,
                # self.elevatorSubsystem,
                direction = -1,  
            )
        )
        NamedCommands.registerCommand("dischargeCoralRightCommand", 
            coralCommands.DischargeCoralCommand(
                self.coralSubsystem,
                # self.elevatorSubsystem,
                direction = 1, 
            )
        )
        NamedCommands.registerCommand("RightPoleAutoAlign", 
            PathOnTheFlyAutoAlign(
                self.drivetrain, 
                self.vision, 
                False
            )
        )
        NamedCommands.registerCommand("LeftPoleAutoAlign", 
            PathOnTheFlyAutoAlign(
                self.drivetrain, 
                self.vision, 
                True
            )
        )

        #After registering named commands, we can build the auto chooser
        self._auto_chooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

        # commands2.SequentialCommandGroup

        CameraServer().launch()

    def getAutonomousCommand(self):
        return self._auto_chooser.getSelected()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        self._joystick.rightBumper().whileTrue(self.drivetrain.apply_request(lambda:(self._robot_centric_drive.with_velocity_x(
                        # self._robot_centric_drive.with_velocity_x(
                        adjust_jostick(-self._joystick.getLeftY(), smooth=True)
                        * self._max_speed * Override_DriveConstant.kSlowMove
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        adjust_jostick(-self._joystick.getLeftX(), smooth=True)
                        * self._max_speed * Override_DriveConstant.kSlowMove
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        adjust_jostick(-self._joystick.getRightX(), smooth=True)
                        * self._max_angular_rate * Override_DriveConstant.kSlowRotate
                        )
                    )
                )
           )

        self._joystick.b().whileTrue(self.drivetrain.apply_request(lambda:(self._robot_centric_drive.with_velocity_x(
                        # self._robot_centric_drive.with_velocity_x(
                        adjust_jostick(self._joystick.getLeftX(), smooth=False)
                        * self._max_speed * Override_DriveConstant.kSuperSlowMove
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(adjust_jostick(self._joystick.getLeftY(), smooth=False)
                        * self._max_speed * Override_DriveConstant.kSuperSlowMove)
                    .with_rotational_rate(
                        adjust_jostick(-self._joystick.getRightX(), smooth=False)
                        * self._max_angular_rate * Override_DriveConstant.kSuperSlowRotate
                        )
                    )
                )
           )

        self.enableFastRobotCentric = False
        SmartDashboard.putBoolean("ROBOT CENTRIC", self.enableFastRobotCentric)

        def toggleRobotCentric():
            self.enableFastRobotCentric = not self.enableFastRobotCentric
            SmartDashboard.putBoolean("ROBOT CENTRIC", self.enableFastRobotCentric)

        self._joystick.start().onTrue(commands2.cmd.runOnce(toggleRobotCentric))

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    # FIELD CENTRIC
                    (self._field_centric_drive.with_velocity_x(
                        # self._robot_centric_drive.with_velocity_x(
                        -adjust_jostick(self._joystick.getLeftY(), smooth=True)
                        * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        adjust_jostick(-self._joystick.getLeftX(), smooth=True)
                        * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        adjust_jostick(-self._joystick.getRightX(), smooth=True)
                        * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                    ) if not self.enableFastRobotCentric else (
                        # ROBOT CENTRIC
                        self._robot_centric_drive.with_velocity_x(
                            # self._robot_centric_drive.with_velocity_x(
                            adjust_jostick(-self._joystick.getLeftY(), smooth=True)
                            * self._max_speed
                        )  # Drive forward with negative Y (forward)
                        .with_velocity_y(
                            adjust_jostick(-self._joystick.getLeftX(), smooth=True)
                            * self._max_speed
                        )  # Drive left with negative X (left)
                        .with_rotational_rate(
                            adjust_jostick(-self._joystick.getRightX(), smooth=True)
                            * self._max_angular_rate
                        )
                    )
                )
            )
        )

        self._joystick.leftBumper().whileTrue(
            self.drivetrain.apply_request(
                lambda: self.field_centric_angle_lock
                .with_velocity_x((adjust_jostick(-self._joystick.getLeftY(), smooth=True)* self._max_speed))
                .with_velocity_y(adjust_jostick(-self._joystick.getLeftX(), smooth=True)* self._max_speed)
                .with_target_direction(Rotation2d.fromDegrees(self.getHumanPlayerAngle()))
            )
        )

        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))

        # reset the field-centric heading on left stick press
        self._joystick.leftStick().onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

        # WARNING: ONLY ADD COMMAND INPUT STUFF TO ROBOT CONTAINER
        # NOTE: ITS UP TO YOU TO DECIDE WHETHER ANYTHING BELONGS HERE OR SOMEWHERE ELSE

        if self.ENABLE_ALGAE:
            # TODO: Make these sequential commands when you actually have time

            # ALGAE REEF INTAKE COMMAND
            algaeReefIntakeCommand = algaeCommands.AlgaeInstantCommand(
                self.algaeSubsystem,
                AlgaeConstants.kPivotReefIntakingValue,
                -1 * AlgaeConstants.kIntakeMultiplier,
            )

            # ALGAE PROCESS COMMAND
            algaeProcessCommand = algaeCommands.AlgaeCommand(
                self.algaeSubsystem,
                AlgaeConstants.kPivotProcessingValue,
                -1 * AlgaeConstants.kIntakeMultiplier * 0.7, # The 0.7 is so it won't bounce out
            )

            # ALGAE GROUND INTAKE COMMAND
            algaeGroundIntakeCommand = algaeCommands.AlgaeCommand(
                self.algaeSubsystem,
                AlgaeConstants.kPivotGroundIntakingValue,
                1 * AlgaeConstants.kIntakeMultiplier,
            )
            
            # ALGAE AFTER GROUND INTAKE COMMAND
            algaeAfterGroundIntakeCommand = algaeCommands.AlgaeCommand(
                self.algaeSubsystem,
                AlgaeConstants.kPivotAfterGroundIntakingValue,
                0 * AlgaeConstants.kIntakeMultiplier,
            )

            algaeHomeCommand = algaeCommands.AlgaeHomeCommand(self.algaeSubsystem)

            self._joystick.povUp().onTrue(algaeReefIntakeCommand) # Reef Intake
            self._joystick.rightTrigger().onTrue(algaeGroundIntakeCommand) # Reef Ground
            self._joystick.leftTrigger().onTrue(algaeProcessCommand) # Reef Process
            self._joystick.leftStick().onTrue(algaeHomeCommand) # Home
            
            # For testing so I don't have to hit the joystick perfectly
            self._joystick.povDown().onTrue(algaeHomeCommand)

            self._joystick.povUp().onFalse(algaeAfterGroundIntakeCommand) # algaeAfterGroundIntakeCommand)
            self._joystick.rightTrigger().onFalse(algaeAfterGroundIntakeCommand)
            self._joystick.leftTrigger().onFalse(algaeHomeCommand)
            # self._joystick.leftStick().onFalse()


        self._joystick.y().whileTrue(PathOnTheFlyAutoAlign(self.drivetrain, self.vision, False))
        self._joystick.x().whileTrue(PathOnTheFlyAutoAlign(self.drivetrain, self.vision, True))
        # self._joystick.b() # B is being used for reef robot centric

        if self.ENABLE_CORAL:
            # Declare Coral Sequential Commands
            defaultCoralCommand = coralCommands.CoralDefaultCommand(self.coralSubsystem)
            
            self.dischargeCoralRightCommand = coralCommands.DischargeCoralCommand(
                self.coralSubsystem,
                # self.elevatorSubsystem,
                direction = -1,  
            )

            self.dischargeCoralLeftCommand = coralCommands.DischargeCoralCommand(
                self.coralSubsystem,
                # self.elevatorSubsystem,
                direction = 1, 
            )
            
            # 0.53 0.27
            self.coralSubsystem.setDefaultCommand(defaultCoralCommand)
            self._joystick2.rightBumper().whileTrue(self.dischargeCoralRightCommand)
            self._joystick2.leftBumper().whileTrue(self.dischargeCoralLeftCommand)

        if self.ENABLE_ELEVATOR:
            # Home elevator at the start
            commands2.CommandScheduler.getInstance().schedule(
                elevatorCommands.HomeElevatorCommand(self.elevatorSubsystem)
            )

            SC = elevatorCommands.SetElevatorCommand
            
            self._joystick2.a().onTrue(elevatorCommands.HomeElevatorCommand(self.elevatorSubsystem))
            self._joystick2.x().onTrue(SC(self.elevatorSubsystem, ElevatorConstants.kCoralLv3))
            self._joystick2.b().onTrue(SC(self.elevatorSubsystem, ElevatorConstants.kAlgaeLv3))
            self._joystick2.y().onTrue(SC(self.elevatorSubsystem, ElevatorConstants.kCoralLv4))

            
            def doDeadband(num):
                return 0 if num <= 0.1 and num >= -0.1 else num
            
            def getElevatorIncrement():
                speed = (
                    (0.9 * (self._joystick2.getRightTriggerAxis() - self._joystick2.getLeftTriggerAxis())
                    + 0.3 * (-1 * doDeadband(self._joystick2.getLeftY()))
                    ) * ElevatorConstants.kElevatorIncrementalStep
                )
                return speed
            
            self.continuousElevatorCommand = (
                elevatorCommands.ContinuousIncrementCommand(
                    self.elevatorSubsystem, getElevatorIncrement
                )
            )

            self.elevatorSubsystem.setDefaultCommand(self.continuousElevatorCommand)

        if self.ENABLE_CLIMB:

            # Reeling command
            self.forwardCommand = climbCommands.Forward(self.climbSubsystem)

            # Unreeling command
            self.backwardCommand = climbCommands.Backward(self.climbSubsystem)

            # Button detections:
            # TODO: consider auto trigger when latched on (with debounce of course)
            # sensing_cage_in_hand = commands2.button.Trigger(self.climbSubsystem.cageInGripSwitch.get())
            self._joystick.povLeft().whileTrue(self.forwardCommand)
            self._joystick2.back().whileTrue(self.backwardCommand)
            
        if self.ENABLE_PNEUMATIC:
            defaultPneumaticCommand = pneumaticCommands.DefaultPneumaticCommand(
                self.pneumaticSubsystem,
                self.elevatorSubsystem,
                self.coralSubsystem,
            )
            
            # TODO: removed timer may need new command or time put on via factory here

            self.pneumaticSubsystem.setDefaultCommand(defaultPneumaticCommand)
            # self._joystick2.povUp().whileTrue(pneumaticCommands.PulseFlippersCommand(self.pneumaticSubsystem))
            
            testPneumaticCommand = pneumaticCommands.PulseFlippersCommand(self.pneumaticSubsystem)
            
            self._joystick2.povUp().whileTrue(testPneumaticCommand)

    def getHumanPlayerAngle(self)-> float:
        offset = 0
        if DriverStation.getAlliance() and (DriverStation.getAlliance() == DriverStation.Alliance.kRed):
            offset = 108
        return (126 + offset) if self.drivetrain.get_state().pose.y >= 3.8  else (-126 - offset)
