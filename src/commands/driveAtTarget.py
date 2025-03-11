from commands2 import Command
from wpimath.geometry import Translation2d, Pose2d, Translation3d, Pose3d, Rotation2d, Transform2d, Transform3d, Rotation3d
from wpimath.units import degrees, radians, degreesToRadians, radiansToDegrees
import limelight
import limelightresults
from phoenix6 import swerve
from phoenix6.swerve.utility import phoenix_pid_controller,geometry,kinematics

# create a class that extends Command names DriveAtTargetCommand. 
# It should use the limelight target latest results (and [maybe] the drivetrain pose) to move the robot to a specific orientation relative to a target.
# The command should be interruptable and should not end on its own.
# The command should require the drivetrain subsystem.


# def add_transforms(transform1, transform2):
#     '''takes two 3d transforms and returns the sum of the two'''
#     # Adding the translations and rotations separately
#     result_translation = transform1.translation() + transform2.translation()
#     result_rotation = Rotation3d(
#         transform1.rotation().x + transform2.rotation().x,
#         transform1.rotation().y + transform2.rotation().y,
#         transform1.rotation().z + transform2.rotation().z
#     )
#     return Transform3d(result_translation, result_rotation)
    

class DriveAtTargetCommand(Command):
    def __init__(self, setpointTranslation: Translation2d, drivetrain, latest_parsed_result):
        super().__init__()
        self.latest_parsed_result=latest_parsed_result #TODO later; change to using general results so don't wait on the parsing
        # limelight JSON parsed spec: https://docs.limelightvision.io/docs/docs-limelight/apis/json-results-specification
        self.vision_valid = latest_parsed_result.validity
        self.vision_horz_angle = latest_parsed_result.target_x_degrees
        self.vision_vert_angle = latest_parsed_result.target_y_degrees
        self.vision_target_consumed_area = latest_parsed_result.target_area

        # self.Kp_angle = 0.01
        # self.Kp_area = 0.01
        self.setpoint_angle_x = degreesToRadians(0) #TODO: should this be input?
        self.setpoint_angle_y = degreesToRadians(0) #TODO: should this be input?
        self.setpoint_distance = 2 #meters
        # self.setpoint_area = 0.25 #TODO: should this be input?
        #not used currently by limelight self.vision_target_skew = latest_parsed_result.target_skew
        # self.control_rotation = self.Kp_angle * (self.vision_horz_angle - self.setpoint_angle_x)
        # self.control_forward = (self.Kp_area * (self.vision_target_consumed_area - self.setpoint_area)
        #                         + self.Kp_angle * (self.vision_vert_angle - self.setpoint_angle_y))
        dist_side_at_100pct = .5 #meters
        dist_side_at_10pct = 15.5 #meters
        def get_distance_from_area(area):
            b=(1)
            y=area**0.5
            m=(.10 - 1.0)/(dist_side_at_10pct - dist_side_at_100pct )           
            return (y-b)/m + dist_side_at_100pct
        self.current_distance = get_distance_from_area(self.vision_target_consumed_area)


        '''robot_center = Translation3d(0, 0, .05)
        camera_location_on_robot = Translation3d(.15, 0.0, .5) #TODO: get this from the robot's configuration
        camera_rotation_on_robot = Rotation3d(0,degreesToRadians(-5),0)
        camera_transform = Transform3d(camera_location_on_robot, camera_rotation_on_robot)
        camera_translation_from_XY_plane = add_transforms(camera_transform, Transform3d(robot_center, Rotation3d()))
        camera_loc_from_XY_plane = Pose3d(camera_translation_from_XY_plane.translation(), camera_translation_from_XY_plane.rotation())
        print(f'camera location off ground: {camera_loc_from_XY_plane}')
        target_setpoint_pose = Pose3d(Translation3d(x=2, y=0, z=1.25), Rotation3d(0,0,degreesToRadians(10)))
        err = Transform3d(camera_loc_from_XY_plane, target_setpoint_pose)
        print(f'error in camera location: {err}')'''

        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self.current_pose_swerve = self.drivetrain.get_state_copy().pose2d

    def execute(self):
        # self.drivetrain.add_vision_measurement(self.latest_parsed_result.robot_pose, self.latest_parsed_result.timestamp, self.latest_parsed_result.vision_measurement_std_devs)
        if self.vision_valid:
            turn_effort = (phoenix_pid_controller()
            .setPID(0.1, 0.05, 0.05)
            .setTolerance(degreesToRadians(5))
            .calculate(self.vision_horz_angle, self.setpoint_angle_x)
            )
            forward_effort = (phoenix_pid_controller()
            .setPID(0.1, 0.05, 0.05)
            .setTolerance(0.05)
            .calculate(self.current_distance, self.setpoint_distance)
            )
            (swerve.requests.RobotCentric()
            .with_rotational_rate(turn_effort)
            .with_velocity_x(forward_effort)
            .with_velocity_y(-self.control_rotation)
            )#.schedule())

    def isFinished(self):
        return False

    def end(self):
        pass

    def interrupted(self):
        self.end()
