#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

# NOTE: Please use the following naming conventions:
# Subsystems are: name + "Subsystem"
# Commands are: name + "Command"
# Command modules are: name + "Commands" (in camel case)
# Enable variables are in MACRO_CASE just like constants
# Constant classes are: name + "Constants" (also in camel case)

# If you wish to change any of these, be sure to change all
# instances of the rule unless you're really desperate on time

# from imaplib import Commands
import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine

from pathplannerlib.auto import AutoBuilder, PathfindThenFollowPath, PathPlannerAuto
from pathplannerlib.path import PathPlannerPath, PathConstraints

# NOTE: THIS IS THE OFFICIAL LOCATION FOR IMPORTING COMMANDS AND SUBSYSTEMS AND CONSTANTS
from subsystems import (
    algaeSubsystem,
    coralSubsystem,
    elevatorSubsystem,
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

from constants import ElevatorConstants, AlgaeConstants, CoralConstants

from robotUtils.adjustJoystick import adjust_jostick

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians
from wpilib import SmartDashboard

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
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(0.1)
            .with_rotational_deadband(0.1)  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        # self._robot_centric_drive = (
        #     swerve.requests.RobotCentric()
        #     .with_deadband(0.1)
        #     .with_rotational_deadband(0.1)  # Add a 10% deadband
        #     .with_drive_request_type(
        #         swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        #     )  # Use open-loop control for drive motors
        # )

        self._logger = Telemetry(self._max_speed)

        self._joystick = commands2.button.CommandXboxController(0)
        self._joystick2 = commands2.button.CommandXboxController(1)  # FOR TESTING

        self.drivetrain = TunerConstants.create_drivetrain()

        # NOTE: HAVE ALL THE ENABLY THINGS HERE (and change them all to true when actually playing)

        self.ENABLE_ALGAE = True
        self.ENABLE_ELEVATOR = True
        self.ENABLE_CORAL = True
        self.ENABLE_CLIMB = True
        self.ENABLE_PNEUMATIC = True

        # Command Scheduler is needed to run periodic() function on subsystems
        # self.scheduler = commands2.CommandScheduler()

        # NOTE: DECLARE ALL SUBSYSTEMS HERE AND NOWHERE ELSE PLS

        if self.ENABLE_ALGAE:
            self.algaeSubsystem = algaeSubsystem.AlgaeSubsystem()
            # self.scheduler.registerSubsystem(self.algaeSubsystem)

        if self.ENABLE_ELEVATOR:
            self.elevatorSubsystem = elevatorSubsystem.ElevatorSubsystem()

        if self.ENABLE_CORAL:
            self.coralSubsystem = coralSubsystem.CoralTrack()

        if self.ENABLE_CLIMB:
            self.climbSubsystem = climbSubsystem.ClimbSubsystem()
            # self.scheduler.registerSubsystem(self.climbSubsystem)
        
        if self.ENABLE_PNEUMATIC:
            self.pneumaticSubsystem = pneumaticSubsystem.PneumaticSubsystem()
            # self.scheduler.registerSubsystem(self.pneumaticSubsystem)

        # Configure the button bindings
        self.configureButtonBindings()

    def getAutonomousCommand():
        # Load the path you want to follow using its name in the GUI
        path = PathPlannerPath.fromPathFile("moveForeward")
        # Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path)

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        
        # Slow Down on right bumper stuff
        self.slowSpeedMultiplier = 1
        self.slowRotationMultiplier = 1
        
        def changeSlowMultipliers(speed = 1, rotation = 1): 
            self.slowSpeedMultiplier = speed
            self.slowRotationMultiplier = rotation

        self._joystick.rightBumper().onTrue(
            # THESE HERE ARE THE VALUES TO TUNE YAY     vvv  vvvv
            commands2.cmd.runOnce(changeSlowMultipliers(0.2, 0.35))
            # THESE HERE ARE THE VALUES TO TUNE YAY     ^^^  ^^^^
        )

        self._joystick.rightBumper().onFalse(
            commands2.cmd.runOnce(changeSlowMultipliers(1, 1))
        )

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        # self._robot_centric_drive.with_velocity_x(
                        -adjust_jostick(self._joystick.getLeftY(), smooth=True)
                        * self._max_speed * self.slowSpeedMultiplier
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        adjust_jostick(-self._joystick.getLeftX(), smooth=True)
                        * self._max_speed * self.slowSpeedMultiplier
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        adjust_jostick(-self._joystick.getRightX(), smooth=True)
                        * self._max_angular_rate * self.slowRotationMultiplier
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        # We don't need this I think
        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )

        robotCentricSpeedMultiplier = 0.2

        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        # (self._joystick.back() & self._joystick.y()).whileTrue(
        #     self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        # )
        # (self._joystick.back() & self._joystick.x()).whileTrue(
        #     self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse) # Commented out by Aida for climb - disable climb if needed.
        # )
        # (self._joystick.start() & self._joystick.y()).whileTrue(
        #     self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        # )
        # (self._joystick.start() & self._joystick.x()).whileTrue(
        #     self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse) # Commented out by Aidan for climb - disable climb if needed.
        # )

        # reset the field-centric heading on left bumper press
        self._joystick.leftBumper().onTrue(
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
                1 * AlgaeConstants.kIntakeMultiplier,
            )

            # ALGAE PROCESS COMMAND
            algaeProcessCommand = algaeCommands.AlgaeCommand(
                self.algaeSubsystem,
                AlgaeConstants.kPivotProcessingValue,
                -1 * AlgaeConstants.kIntakeMultiplier,
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

            self._joystick.y().onTrue(algaeReefIntakeCommand) # Reef Intake
            self._joystick.rightTrigger().onTrue(algaeGroundIntakeCommand) # Reef Ground
            self._joystick.leftTrigger().onTrue(algaeProcessCommand) # Reef Process
            self._joystick.leftStick().onTrue(algaeHomeCommand) # Home
            
            # For testing so I don't have to hit the joystick perfectly
            self._joystick.povDown().onTrue(algaeHomeCommand)

            self._joystick.y().onFalse(algaeAfterGroundIntakeCommand) # algaeAfterGroundIntakeCommand)
            self._joystick.rightTrigger().onFalse(algaeAfterGroundIntakeCommand)
            self._joystick.leftTrigger().onFalse(algaeHomeCommand)
            # self._joystick.leftStick().onFalse()

        if self.ENABLE_CORAL:
            # Declare Coral Sequential Commands
            defaultCoralCommand = coralCommands.CoralDefaultCommand(self.coralSubsystem)

            dischargeCoralLeftCommand = coralCommands.DischargeCoralCommand(
                self.coralSubsystem,
                # self.elevatorSubsystem,
                direction = -1,  # Left is -1, Right is 1
            )

            dischargeCoralRightCommand = coralCommands.DischargeCoralCommand(
                self.coralSubsystem,
                # self.elevatorSubsystem,
                direction = 1,  # Left is -1, Right is 1
            )
            
            # 0.53 0.27
            self.coralSubsystem.setDefaultCommand(defaultCoralCommand)
            # Cayden said to invert these
            self._joystick2.rightBumper().whileTrue(dischargeCoralLeftCommand)
            self._joystick2.leftBumper().whileTrue(dischargeCoralRightCommand)

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

            # self._joystick2.povLeft().onTrue(JS_left)
            # self._joystick2.povRight().onTrue(JS_right)

            # *NEW* Elevator Increment
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
            self._joystick.y().whileTrue(self.forwardCommand)
            self._joystick.x().whileTrue(self.backwardCommand)

        if self.ENABLE_PNEUMATIC:
            defaultPneumaticCommand = pneumaticCommands.DefaultPneumaticCommand(
                self.pneumaticSubsystem,
                self.elevatorSubsystem,
                self.coralSubsystem,
            )
            
            # TODO: removed timer may need new command or time put on via factory here
            testPneumaticCommand = pneumaticCommands.PulseFlippersCommand(self.pneumaticSubsystem)

            self.pneumaticSubsystem.setDefaultCommand(defaultPneumaticCommand)
            self._joystick2.povUp().whileTrue(testPneumaticCommand)
