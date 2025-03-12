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

import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine

# NOTE: THIS IS THE OFFICIAL LOCATION FOR IMPORTING COMMANDS AND SUBSYSTEMS AND CONSTANTS
from subsystems import (
    algaeSubsystem,
    coralSubsystem,
    pneumaticSubsystem,
    elevatorSubsystem,
)

from commands import (
    algaeCommands,
    coralCommands,
    elevatorCommands,
)

from constants import (
    ElevatorConstants, 
    AlgaeConstants, 
    CoralConstants
)

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from phoenix6 import swerve
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._robot_centric_drive = (
            swerve.requests.RobotCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )

        self._logger = Telemetry(self._max_speed)

        self._joystick = commands2.button.CommandXboxController(0)
        self._joystick2 = commands2.button.CommandXboxController(1) # FOR TESTING

        self.drivetrain = TunerConstants.create_drivetrain()
        
        # NOTE: HAVE ALL THE ENABLY THINGS HERE (and change them all to true when actually playing)
        
        self.ENABLE_ALGAE = True
        self.ENABLE_ELEVATOR = True
        self.ENABLE_CORAL = True
        self.ENABLE_CLIMB = True
        
        # Command Scheduler is needed to run periodic() function on subsystems
        self.scheduler = commands2.CommandScheduler()
        
        # NOTE: DECLARE ALL SUBSYSTEMS HERE AND NOWHERE ELSE PLS
        
        if self.ENABLE_ALGAE:
            self.algaeSubsystem = algaeSubsystem.AlgaeSubsystem()
            self.scheduler.registerSubsystem(self.algaeSubsystem)
            
        if self.ENABLE_ELEVATOR:
            self.elevatorSubsystem = elevatorSubsystem.ElevatorSubsystem()
            
        if self.ENABLE_CORAL:
            self.coralSubsystem = coralSubsystem.CoralTrack()
            self.scheduler.registerSubsystem(self.coralSubsystem)
            # self.scheduler.schedule()
            self.pneumaticSubsystem = pneumaticSubsystem.PneumaticSubsystem()
            
        if self.ENABLE_CLIMB:
            ...

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
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(  
                    #self._robot_centric_drive.with_velocity_x(
                        -self._joystick.getLeftY() * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -self._joystick.getLeftX() * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self._joystick.getRightX() * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )

        self._joystick.rightBumper().whileTrue(
            self.drivetrain.apply_request(
                lambda: (
                    self._robot_centric_drive.with_velocity_x(
                        -self._joystick.getLeftY() * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -self._joystick.getLeftX() * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self._joystick.getRightX() * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

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
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )
        
        # WARNING: ONLY ADD COMMAND INPUT STUFF TO ROBOT CONTAINER
        # NOTE: ITS UP TO YOU TO DECIDE WHETHER ANYTHING BELONGS HERE OR SOMEWHERE ELSE
        
        if self.ENABLE_ALGAE:
            # TODO: Make these sequential commands when you actually have time
            
            # ALGAE INTAKE COMMAND
            algaeIntakeCommand = algaeCommands.AlgaeInstantCommand(
                self.algaeSubsystem, AlgaeConstants.kPivotReefIntakingValue, 1 * AlgaeConstants.kIntakeMultiplier
            )

            # ALGAE PROCESS COMMAND
            algaeProcessCommand = algaeCommands.AlgaeInstantCommand(
                self.algaeSubsystem, AlgaeConstants.kPivotProcessingValue, -1 * AlgaeConstants.kIntakeMultiplier
            )
             
            # ALGAE IDLE COMMAND
            algaeIdleCommand = algaeCommands.AlgaeInstantCommand(
                self.algaeSubsystem, AlgaeConstants.kPivotIdleValue, 0 * AlgaeConstants.kIntakeMultiplier
            )
            
            # self._joystick2.y().onTrue(algaeIntakeCommand)
            # self._joystick2.povDown().onTrue(algaeProcessCommand)
            # self._joystick2.a().onTrue(algaeIdleCommand)
            
            # self._joystick2.povUp().onTrue(algaeCommands.AlgaeHomeCommand(self.algaeSubsystem))
        
        if self.ENABLE_CORAL:
            # Declare Coral Sequential Commands
            defaultCoralCommand = coralCommands.CoralDefaultCommand(self.coralSubsystem)
            
            dischargeCoralLeftCommand = coralCommands.DischargeCoralCommand(
                self.coralSubsystem,
                self.elevatorSubsystem,
                self.pneumaticSubsystem,
                direction = -1 # Left is -1, Right is 1
            )
            
            dischargeCoralRightCommand = coralCommands.DischargeCoralCommand(
                self.coralSubsystem,
                self.elevatorSubsystem,
                self.pneumaticSubsystem,
                direction = 1 # Left is -1, Right is 1
            )
            
            self.coralSubsystem.setDefaultCommand(defaultCoralCommand)
            self._joystick2.leftBumper().whileTrue(dischargeCoralLeftCommand)
            self._joystick2.rightBumper().whileTrue(dischargeCoralRightCommand)
            
            
        if self.ENABLE_ELEVATOR:
            IC = elevatorCommands.InstantSetElevatorCommand # So I can actually see all the code
            # self._joystick2.x().onTrue(IC(self.elevatorSubsystem, ElevatorConstants.kCoralIntakePosition))
            self._joystick2.povUp().onTrue(IC(self.elevatorSubsystem, ElevatorConstants.kCoralLv2))
            self._joystick2.povRight().onTrue(IC(self.elevatorSubsystem, ElevatorConstants.kCoralLv3))
            self._joystick2.povDown().onTrue(IC(self.elevatorSubsystem, ElevatorConstants.kCoralLv4))
            # self._joystick2.x().onTrue(IC(self.elevatorSubsystem, ElevatorConstants.kAlgaeGroundIntake))
            # self._joystick2.x().onTrue(IC(self.elevatorSubsystem, ElevatorConstants.kAlgaeLv2))
            # self._joystick2.x().onTrue(IC(self.elevatorSubsystem, ElevatorConstants.kAlgaeLv3))
            # self._joystick2.x().onTrue(IC(self.elevatorSubsystem, ElevatorConstants.kAlgaeProcess))

        if self.ENABLE_CLIMB:
            ...

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command
        to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return commands2.cmd.print_("No autonomous command configured")
