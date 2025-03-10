from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
import math
from pathplannerlib.auto import AutoBuilder, RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from phoenix6 import SignalLogger, swerve, units, utils
from typing import Callable, overload
from wpilib import DriverStation, Notifier, RobotController, SmartDashboard, SendableChooser
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Rotation2d, Pose2d


from robotUtils.limelight import LimelightHelpers
import numpy as np
from constants import AprilTags, LimelightConstants
# from archive import odometry_fuse


class CommandSwerveDrivetrain(Subsystem, swerve.SwerveDrivetrain):
    """
    Class that extends the Phoenix 6 SwerveDrivetrain class and implements
    Subsystem so it can easily be used in command-based projects.
    """

    _SIM_LOOP_PERIOD: units.second = 0.005

    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    """Blue alliance sees forward as 0 degrees (toward red alliance wall)"""
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)
    """Red alliance sees forward as 180 degrees (toward blue alliance wall)"""

    @overload
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:     Type of the drive motor
        :type drive_motor_type:      type
        :param steer_motor_type:     Type of the steer motor
        :type steer_motor_type:      type
        :param encoder_type:         Type of the azimuth encoder
        :type encoder_type:          type
        :param drivetrain_constants: Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:  swerve.SwerveDrivetrainConstants
        :param modules:              Constants for each specific module
        :type modules:               list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:            Type of the drive motor
        :type drive_motor_type:             type
        :param steer_motor_type:            Type of the steer motor
        :type steer_motor_type:             type
        :param encoder_type:                Type of the azimuth encoder
        :type encoder_type:                 type
        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:         swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    @overload
    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        odometry_update_frequency: units.hertz,
        odometry_standard_deviation: tuple[float, float, float],
        vision_standard_deviation: tuple[float, float, float],
        modules: list[swerve.SwerveModuleConstants],
    ) -> None:
        """
        Constructs a CTRE SwerveDrivetrain using the specified constants.

        This constructs the underlying hardware devices, so users should not construct
        the devices themselves. If they need the devices, they can access them through
        getters in the classes.

        :param drive_motor_type:            Type of the drive motor
        :type drive_motor_type:             type
        :param steer_motor_type:            Type of the steer motor
        :type steer_motor_type:             type
        :param encoder_type:                Type of the azimuth encoder
        :type encoder_type:                 type
        :param drivetrain_constants:        Drivetrain-wide constants for the swerve drive
        :type drivetrain_constants:         swerve.SwerveDrivetrainConstants
        :param odometry_update_frequency:   The frequency to run the odometry loop. If
                                            unspecified or set to 0 Hz, this is 250 Hz on
                                            CAN FD, and 100 Hz on CAN 2.0.
        :type odometry_update_frequency:    units.hertz
        :param odometry_standard_deviation: The standard deviation for odometry calculation
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians
        :type odometry_standard_deviation:  tuple[float, float, float]
        :param vision_standard_deviation:   The standard deviation for vision calculation
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians
        :type vision_standard_deviation:    tuple[float, float, float]
        :param modules:                     Constants for each specific module
        :type modules:                      list[swerve.SwerveModuleConstants]
        """
        ...

    def __init__(
        self,
        drive_motor_type: type,
        steer_motor_type: type,
        encoder_type: type,
        drivetrain_constants: swerve.SwerveDrivetrainConstants,
        arg0=None,
        arg1=None,
        arg2=None,
        arg3=None,
    ):
        Subsystem.__init__(self)
        swerve.SwerveDrivetrain.__init__(
            self, drive_motor_type, steer_motor_type, encoder_type,
            drivetrain_constants, arg0, arg1, arg2, arg3
        )


        self.pigeon2.reset()
        self.skip_counter = 0 # increment with ever iteration, use to set frequency of certain actions

        self._sim_notifier: Notifier | None = None
        self._last_sim_time: units.second = 0.0

        self._has_applied_operator_perspective = False
        """Keep track if we've ever applied the operator perspective before or not"""

        SmartDashboard.putBoolean("HighConfidence_VisionUpdate", False)
        SmartDashboard.putNumber("VisionUpdate_StdDev_XandY", 0.07)
        SmartDashboard.putNumber("VisionUpdate_StdDev_Theta", 9999999)
        self.megatag_chooser = SendableChooser()
        self.megatag_chooser.setDefaultOption('MegaTag1', "MegaTag1")
        self.megatag_chooser.addOption('MegaTag2', "MegaTag2")
        SmartDashboard.putData("MegaTag Chooser", self.megatag_chooser)

        # Swerve request to apply during path following
        self._apply_robot_speeds = swerve.requests.ApplyRobotSpeeds()
        
        # Swerve requests to apply during SysId characterization
        self._translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self._steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self._rotation_characterization = swerve.requests.SysIdSwerveRotation()

        self._sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V to prevent brownout
                stepVoltage=4.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdTranslation_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._translation_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """SysId routine for characterizing translation. This is used to find PID gains for the drive motors."""

        self._sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._steer_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """SysId routine for characterizing steer. This is used to find PID gains for the steer motors."""

        self._sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                # This is in radians per second², but SysId only supports "volts per second"
                rampRate=math.pi / 6,
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Use default timeout (10 s)
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                ),
            ),
            SysIdRoutine.Mechanism(
                lambda output: (
                    # output is actually radians per second, but SysId only supports "volts"
                    self.set_control(
                        self._rotation_characterization.with_rotational_rate(output)
                    ),
                    # also log the requested output for SysId
                    SignalLogger.write_double("Rotational_Rate", output),
                ),
                lambda log: None,
                self,
            ),
        )
        """
        SysId routine for characterizing rotation.
        This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
        See the documentation of swerve.requests.SysIdSwerveRotation for info on importing the log to SysId.
        """

        self._sys_id_routine_to_apply = self._sys_id_routine_translation
        """The SysId routine to test"""

        if utils.is_simulation():
            self._start_sim_thread()
        self._configure_auto_builder()
    
    def _configure_auto_builder(self):
        config = RobotConfig.fromGUISettings()
        if AutoBuilder.isConfigured():
            print("AutoBuilder is already configured, Double Configuring XXXXXXXXXXX!!!!!!!!!!!!!!!!!!!!!!!XXXXXXXXXXXXXXX")
    
        AutoBuilder.configure(
            lambda: self.get_state().pose,   # Supplier of current robot pose
            self.reset_pose,                 # Consumer for seeding pose against auto
            lambda: self.get_state().speeds, # Supplier of current robot speeds
            # Consumer of ChassisSpeeds and feedforwards to drive the robot
            lambda speeds, feedforwards: self.set_control(
                self._apply_robot_speeds
                .with_speeds(speeds)
                .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
            ),
            PPHolonomicDriveController(
                # PID constants for translation
                PIDConstants(1.0, 0.0, 0.0),
                # PID constants for rotation
                PIDConstants(.004, 0.0, 0.0)
            ),
            config,
            # Assume the path needs to be flipped for Red vs Blue, this is normally the case
            #  don't know what the 'or' is for ... lambda: (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed,
            lambda: DriverStation.getAlliance()  == DriverStation.Alliance.kRed, #has no effect in sim as it is init happens before color selection
            self # Subsystem for requirements
        )
        
    def apply_request(
        self, request: Callable[[], swerve.requests.SwerveRequest]
    ) -> Command:
        """
        Returns a command that applies the specified control request to this swerve drivetrain.

        :param request: Lambda returning the request to apply
        :type request: Callable[[], swerve.requests.SwerveRequest]
        :returns: Command to run
        :rtype: Command
        """
        return self.run(lambda: self.set_control(request()))

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Quasistatic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Quasistatic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Dynamic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Dynamic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.dynamic(direction)

    def periodic(self):
        # Periodically try to apply the operator perspective.
        # If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
        # This allows us to correct the perspective in case the robot code restarts mid-match.
        # Otherwise, only check and apply the operator perspective if the DS is disabled.
        # This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
        self.skip_counter +=1

        LimelightHelpers.set_robot_orientation(
            'limelight',
            # self.get_state().pose.rotation().degrees(),  # OR
            self.pigeon2.get_yaw().value + LimelightConstants.kLL3yaw,
            # 0,  #
            self.pigeon2.get_angular_velocity_z_world().value,
            # 0,  #
            self.pigeon2.get_pitch().value + LimelightConstants.kLL3pitch,
            0,  #
            # self.pigeon2.get_angular_velocity_y_world().value,
            # 0,  #
            self.pigeon2.get_roll().value + LimelightConstants.kLL3roll,
            0,  #
            # self.pigeon2.get_angular_velocity_x_world().value
        )
        LimelightHelpers.set_robot_orientation(
            'limelight-four',
            # self.get_state().pose.rotation().degrees(),  # OR
            self.pigeon2.get_yaw().value + LimelightConstants.kLL4yaw,
            # 0,  #
            self.pigeon2.get_angular_velocity_z_world().value,
            # 0,  #
            self.pigeon2.get_pitch().value + LimelightConstants.kLL4pitch,
            0,  #
            # self.pigeon2.get_angular_velocity_y_world().value,
            # 0,  #
            self.pigeon2.get_roll().value + LimelightConstants.kLL4roll,
            0,  #
            # self.pigeon2.get_angular_velocity_x_world().value
        )

        if not self._has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color is not None:
                self.set_operator_perspective_forward(
                    self._RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == DriverStation.Alliance.kRed
                    else self._BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True
        
        skip_tripper_base = 50


        if self.skip_counter % skip_tripper_base ==0:
            # with this and all sub part of use_vision..., robot is still responsive when skipped at 1/1000
            self.use_vision_odometry_update(limelight_to_use='limelight')

        if (self.skip_counter+skip_tripper_base/2 + 2) % skip_tripper_base ==0:
            self.use_vision_odometry_update(limelight_to_use='limelight-four')

        #TODO:  is this taking too long ? yeah, sort of:: Try with skipping
        if (self.skip_counter + skip_tripper_base/4 + 3) % skip_tripper_base ==0:
            self.ignore_backs_of_AprilTags('limelight')
            self.ignore_backs_of_AprilTags('limelight-four')


    def _start_sim_thread(self):
        def _sim_periodic():
            current_time = utils.get_current_time_seconds()
            delta_time = current_time - self._last_sim_time
            self._last_sim_time = current_time

            # use the measured time delta, get battery voltage from WPILib
            self.update_sim_state(delta_time, RobotController.getBatteryVoltage())

        self._last_sim_time = utils.get_current_time_seconds()
        self._sim_notifier = Notifier(_sim_periodic)
        self._sim_notifier.startPeriodic(self._SIM_LOOP_PERIOD)
    
    def use_vision_odometry_update(self, limelight_to_use= "limelight"):
        """
        Add vision measurement to MegaTag 1 or 2
        """
        # with this, robot is still responsive when skipped at 1/1000
        #have the pigon her so not moving this to limelight subsytem periodic
        # Rather move it to periodic here, needs to happen every frame
        #  LimelightHelpers.set_robot_orientation(
        #     limelight_to_use,
        #     # self.get_state().pose.rotation().degrees(),  # OR
        #     self.pigeon2.get_yaw().value % (360),
        #     # 0,  #
        #     self.pigeon2.get_angular_velocity_z_world().value,
        #     0,  #
        #     # self.pigeon2.get_pitch().value,
        #     0,  #
        #     # self.pigeon2.get_angular_velocity_y_world().value,
        #     0,  #
        #     # self.pigeon2.get_roll().value,
        #     0,  #
        #     # self.pigeon2.get_angular_velocity_x_world().value
        # )
        
        # get botpose estimate with origin on blue side of field; MegaTag1 seems to be working better
        mega_tag_choice = {'MegaTag2': LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(limelight_to_use),
        'MegaTag1': LimelightHelpers.get_botpose_estimate_wpiblue(limelight_to_use)}
        mega_tag = mega_tag_choice[self.megatag_chooser.getSelected()]

        #if we are spinning slower than 720 deg/sec and we see tags    
        if abs(self.pigeon2.get_angular_velocity_z_world().value) <= 720 and mega_tag.tag_count > 0:
            self.VisionUpdateOn_bool = SmartDashboard.getBoolean("HighConfidence_VisionUpdate", True)
            self.VisionUncertainty_XY_low_confidence = SmartDashboard.getNumber("VisionUpdate_StdDev_XandY", 0.07)
            self.VisionUncertainty_Theta_low_confidence = SmartDashboard.getNumber("VisionUpdate_StdDev_Theta", 9999999)
            # set and add vision measurement
            if self.VisionUpdateOn_bool:
                self.set_vision_measurement_std_devs((0.0095, 0.0095, 100)) #originally: (0.7, 0.7, 9999999)
            else:
                self.set_vision_measurement_std_devs((self.VisionUncertainty_XY_low_confidence, 
                                                      self.VisionUncertainty_XY_low_confidence, 
                                                      self.VisionUncertainty_Theta_low_confidence)) #originally: (0.7, 0.7, 9999999)

            self.add_vision_measurement(mega_tag.pose, utils.fpga_to_current_time(mega_tag.timestamp_seconds))
        
    def ignore_backs_of_AprilTags(self, limelight_to_use='limelight') -> None:
        """
        Ignore the backs of AprilTags in vision measurements.
        Make a list of april tags that do not have backs towards robot's position.
        Set the fiducial id filters override to the list of april tags that do not have backs towards robot's position.
        """
        # get botpose estimate with origin on blue side of field
        bot_x, bot_y = self.get_state().pose.translation().x, self.get_state().pose.translation().y
        tag_poses = [tag.pose for tag in AprilTags]
        tag_ids = np.array([tag.ID for tag in AprilTags])
        tag_x, tag_y, tag_theta = [pose.x for pose in tag_poses], [pose.y for pose in tag_poses], [pose.rotation().z for pose in tag_poses]

        theta_t_b = np.arctan2(np.subtract(bot_y, tag_y ), np.subtract(bot_x,tag_x ))
        #normalize values to -pi yo +pi
        theta_delta = (tag_theta - theta_t_b + np.pi) % (2 * np.pi) - np.pi            
        # if less than 90deg difference, then the tag is facing the robot
        facing_robot_bool = np.abs(theta_delta) <= np.pi /2
        facing_robot_ids = tag_ids[facing_robot_bool]
        #tags not on this list will be ignored
        LimelightHelpers.set_fiducial_id_filters_override(limelight_to_use, facing_robot_ids.tolist())

    def reset_pose_by_zone(self, zone='b') -> None:
        """Resets the pose of the robot by zone. values change based on Alliance color
        The 4 zones are about 2 meters off the cardnal points of the reef"""
        # get the alliance color
        alliance_color = DriverStation.getAlliance()
        # use a match statement to get the correct pose
        if alliance_color == DriverStation.Alliance.kRed:
            straight_ahead = Rotation2d.fromDegrees(180)
            
            if zone == 'y':
                set_pose = Pose2d(10.75, 4, straight_ahead)
            elif zone == 'b':
                set_pose = Pose2d(13.0, 6.0,straight_ahead)
            elif zone == 'a':
                set_pose = Pose2d(16, 4, straight_ahead)
            elif zone == 'x':
                set_pose = Pose2d(13, 1.7, straight_ahead)
        else:
            straight_ahead = Rotation2d.fromDegrees(0)
            if zone == 'y':
                set_pose = Pose2d(6.5, 4, straight_ahead)
            elif zone == 'b':
                set_pose = Pose2d(4.5, 1.7, Rotation2d.fromDegrees(0))
            elif zone == 'a':
                set_pose = Pose2d(2, 4, Rotation2d.fromDegrees(0))
            elif zone == 'x':
                set_pose = Pose2d(4.5, 6, Rotation2d.fromDegrees(0))
        self.reset_pose(set_pose)
        # Should happen each itteration (with propper angles)
        # LimelightHelpers.set_robot_orientation('limelight', yaw=straight_ahead.degrees(), 
        #                                        yaw_rate=0, pitch=0, pitch_rate=0, roll=0, roll_rate=0)
        # LimelightHelpers.set_robot_orientation('limelight-four', yaw=straight_ahead.degrees(),
        #                                        yaw_rate=0, pitch=0, pitch_rate=0, roll=0, roll_rate=0)
        # self.pigeon2.reset()
        self.pigeon2.set_yaw(straight_ahead.degrees())


       
        # self.tare_everything()
        # self.reset_pose(After_viz_update_odo_pose)
        # self.container.drivetrain.odometry_thread.set_thread_priority(99)
        # self.container.drivetrain.OdometryThread.stop(self.container.drivetrain.odometry_thread)
        # self.container.drivetrain.reset_pose(After_viz_update_odo_pose)
        # self.container.drivetrain.OdometryThread.start(self.container.drivetrain.odometry_thread)
        # self.container.drivetrain.odometry_thread.set_thread_priority(20)
        # After_viz_update_odo_pose = self.container.drivetrain.get_state().pose
        # self.container.drivetrain.reset_pose(After_viz_update_odo_pose)
        # self.odometry_thread.set_thread_priority(99)
        # self.container.drivetrain.OdometryThread.stop(self.container.drivetrain.odometry_thread)
