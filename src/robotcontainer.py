#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import math

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
import wpilib
from commands2.sysid import SysIdRoutine

from pathplannerlib.auto import AutoBuilder, PathfindThenFollowPath, PathPlannerAuto
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState
from phoenix6.swerve import Pose2d
from wpilib import SmartDashboard

from subsystems.vison import VisionSubsystem
from commands.auto_align import AutoAlign
# NOTE: THIS IS THE OFFICIAL LOCATION FOR IMPORTING COMMANDS AND SUBSYSTEMS AND CONSTANTS
from subsystems import (
    algaeSubsystem,
    coralSubsystem,
    elevatorSubsystem,
    climbSubsystem
)

from commands import (
    algaeCommands,
    coralCommands,
    elevatorCommands,
    climbCommands
)

from constants import ElevatorConstants, AlgaeConstants, CoralConstants

from robotUtils.adjustJoystick import adjust_jostick

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
        self._robot_centric_drive = (
            swerve.requests.RobotCentric()
            .with_deadband(0.1)
            .with_rotational_deadband(0.1)  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )

        self._logger = Telemetry(self._max_speed)

        self._joystick = commands2.button.CommandXboxController(0)
        self._joystick2 = commands2.button.CommandXboxController(1)  # FOR TESTING

        self.drivetrain = TunerConstants.create_drivetrain()

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

        self.vision = VisionSubsystem(self.drivetrain, "limelight-four")
        #self.auto_align = AutoAlign(self.drivetrain, self.vision)
        # NOTE: HAVE ALL THE ENABLY THINGS HERE (and change them all to true when actually playing)

        self.ENABLE_ALGAE = True
        self.ENABLE_ELEVATOR = True
        self.ENABLE_CORAL = True
        self.ENABLE_CLIMB = True

        # NOTE: DECLARE ALL SUBSYSTEMS HERE AND NOWHERE ELSE PLEASE

        if self.ENABLE_ALGAE:
            self.algaeSubsystem = algaeSubsystem.AlgaeSubsystem()


        if self.ENABLE_ELEVATOR:
            self.elevatorSubsystem = elevatorSubsystem.ElevatorSubsystem()

        if self.ENABLE_CORAL:
            self.coralSubsystem = coralSubsystem.CoralTrack()


        if self.ENABLE_CLIMB:
            self.climbSubsystem = climbSubsystem.ClimbSubsystem()
            # self.scheduler.registerSubsystem(self.climbSubsystem)

        # Configure the button bindings
        self.configureButtonBindings()


    def getAutonomousCommand(self):
        return self._auto_chooser.getSelected()

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
                )
            )
        )

        self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        # self._joystick.b().whileTrue(
        #     self.drivetrain.apply_request(
        #         lambda: self._point.with_module_direction(
        #             Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
        #         )
        #     )
        # )

        self._joystick.b().onTrue(self.drivetrain.runOnce(lambda: self.drivetrain.reset_pose(Pose2d(0.485676,1.585252,0.0))))
        #self._joystick.x().whileTrue(self.auto_align)

        self._joystick.rightBumper().whileTrue(
            self.drivetrain.apply_request(
                lambda: (
                    self._robot_centric_drive.with_velocity_x(
                        (-1 * adjust_jostick(self._joystick.getLeftY()))
                        * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        (-1 * adjust_jostick(self._joystick.getLeftX()))
                        * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        (-1 * adjust_jostick(self._joystick.getRightX()))
                        * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
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


        # Elevator button press detection:

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
                AlgaeConstants.kPivotReefIntakingValue,
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
                AlgaeConstants.kPivotGroundIntakingValue,
                0 * AlgaeConstants.kIntakeMultiplier,
            )

            algaeHomeCommand = algaeCommands.AlgaeSetPivotSpeedCommand(self.algaeSubsystem, speed = -0.05)

            self._joystick.povUp().onTrue(algaeReefIntakeCommand)
            self._joystick.rightTrigger().onTrue(algaeGroundIntakeCommand)
            self._joystick.leftTrigger().onTrue(algaeProcessCommand)
            self._joystick.povDown().onTrue(algaeHomeCommand)

            self._joystick.povUp().onFalse(algaeAfterGroundIntakeCommand)
            self._joystick.rightTrigger().onFalse(algaeAfterGroundIntakeCommand)
            self._joystick.leftTrigger().onFalse(algaeHomeCommand)
            # self._joystick.povDown().onFalse()

            # WE NEED
            # Ground intake
            # Reef intake
            # Process
            # Home

        reefWaypoints = PathPlannerPath.waypointsFromPoses([
            Pose2d(2, 2.0, Rotation2d.fromDegrees(90)),
            Pose2d(2, 6.0, Rotation2d.fromDegrees(90))
        ])
        """
            Kinematic path following constraints

            Args:
                maxVelocityMps (float): Max linear velocity (M/S)
                maxAccelerationMpsSq (float): Max linear acceleration (M/S^2)
                maxAngularVelocityRps (float): Max angular velocity (Rad/S)
                maxAngularAccelerationRpsSq (float): Max angular acceleration (Rad/S^2)
                nominalVoltage (float): Nominal battery voltage (Volts)
                unlimited (bool): Should the constraints be unlimited
            """
        reefConstraints = PathConstraints(1.0, 1.0, (2 * math.pi) / 2,
                                          (4 * math.pi) / 2)  # The constraints for this path.
        # constraints = PathConstraints.unlimitedConstraints(12.0) # You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
        # Create the path using the waypoints created above
        testPath = PathPlannerPath(
            reefWaypoints,
            reefConstraints,
            None,
            # The ideal starting state, this is only relevant for pre-planned paths, so can be None for on-the-fly paths.
            GoalEndState(0.0, Rotation2d.fromDegrees(90))
            # Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        )

        # Prevent the path from being flipped if the coordina6es are already correct
        testPath.preventFlipping = False

        self._joystick.y().onTrue(AutoBuilder.pathfindThenFollowPath(
            testPath,reefConstraints
        ))

        if self.ENABLE_CORAL:
            # Declare Coral Sequential Commands
            defaultCoralCommand = coralCommands.CoralDefaultCommand(self.coralSubsystem)

            dischargeCoralLeftCommand = coralCommands.DischargeCoralCommand(
                self.coralSubsystem,
                self.elevatorSubsystem,
                direction = -1,  # Left is -1, Right is 1
            )

            dischargeCoralRightCommand = coralCommands.DischargeCoralCommand(
                self.coralSubsystem,
                self.elevatorSubsystem,
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
            IC = elevatorCommands.IncrementElevatorCommand
            self._joystick2.povUp().whileTrue(commands2.RepeatCommand(IC(self.elevatorSubsystem, ElevatorConstants.kElevatorIncrementalStep)))
            # self._joystick2.povRight().onTrue(IC(self.elevatorSubsystem, ElevatorConstants.kCoralLv3))
            self._joystick2.povDown().whileTrue(commands2.RepeatCommand(IC(self.elevatorSubsystem, -1 * ElevatorConstants.kElevatorIncrementalStep)))

            SC = elevatorCommands.SetElevatorCommand
            self._joystick2.a().onTrue(elevatorCommands.HomeElevatorCommand(self.elevatorSubsystem))
            self._joystick2.x().onTrue(SC(self.elevatorSubsystem, ElevatorConstants.kCoralLv3))
            self._joystick2.b().onTrue(SC(self.elevatorSubsystem, ElevatorConstants.kAlgaeLv3))
            self._joystick2.y().onTrue(SC(self.elevatorSubsystem, ElevatorConstants.kCoralLv4))

            JS_right = commands2.SequentialCommandGroup(
                SC(self.elevatorSubsystem, (lambda: ElevatorConstants.kCoralLv4)()),
                commands2.ParallelRaceGroup(
                    dischargeCoralLeftCommand,
                    commands2.SequentialCommandGroup(
                        commands2.WaitCommand(0.5),
                        SC(self.elevatorSubsystem, (lambda: ElevatorConstants.kCoralLv4_JumpScore)()),
                        commands2.WaitCommand(0.5),
                    )
                )
            )

            JS_left = commands2.SequentialCommandGroup(
                SC(self.elevatorSubsystem, (lambda: ElevatorConstants.kCoralLv4)()),
                commands2.ParallelRaceGroup(
                    dischargeCoralRightCommand,
                    commands2.SequentialCommandGroup(
                        commands2.WaitCommand(0.5),
                        SC(self.elevatorSubsystem, (lambda: ElevatorConstants.kCoralLv4_JumpScore)()),
                        commands2.WaitCommand(0.5),
                    )
                )
            )

            (self._joystick2.b() & self._joystick2.rightTrigger()).onTrue(JS_right)
            (self._joystick2.b() & self._joystick2.leftTrigger()).onTrue(JS_left)

            def doDeadband(num):
                return 0 if num <= 0.08 and num >= -0.08 else num
            
            def getElevatorIncrement():
                speed = (0.3 * (self._joystick2.getRightTriggerAxis() - self._joystick2.getLeftTriggerAxis())
                        + 0.1 * (-1 * doDeadband(self._joystick2.getLeftY()))
                        # - 0.05 * (self._joystick2.getRightY())
                        + 0.03)
                if (self.elevatorSubsystem.get_position() < ElevatorConstants.kLowEnoughToSlowDown
                    and self.elevatorSubsystem.elevmotor_left.get_velocity().value < 0):
                    # Lower speed if position is low enough and going down
                    speed *= ElevatorConstants.kLowEnoughSpeedMultiplier
                return speed

            # self.continuousElevatorCommand = (
            #     elevatorCommands.ContinuousIncrementCommand(
            #         self.elevatorSubsystem, getElevatorIncrement
            #     )
            # )

            # self.elevatorSubsystem.setDefaultCommand(self.continuousElevatorCommand)

            # self._joystick2.povLeft().onTrue(elevatorCommands.InstantTestFlipperCommand(
            #     self.pneumaticSubsystem
            # ))

        if self.ENABLE_CLIMB:

            # Reeling command:
            self.forwardCommand = climbCommands.Forward(self.climbSubsystem)

            # Unreeling command:
            self.backwardCommand = climbCommands.Backward(self.climbSubsystem)

            # Button detections:
            # TODO: consider auto trigger 
            # sensing_cage_in_hand = commands2.button.Trigger(self.climbSubsystem.cageInGripSwitch.get())
            self._joystick.y().whileTrue(self.forwardCommand)
            self._joystick.x().whileTrue(self.backwardCommand)
