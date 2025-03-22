
import robotpy_apriltag as apriltag
from constants import AprilTagConstants, RobotDimensions
from wpimath.geometry import Translation2d, Pose2d, Translation3d, Pose3d, Rotation2d, Transform2d, Transform3d, Rotation3d
from wpimath.units import degrees, radians, degreesToRadians, radiansToDegrees, inchesToMeters, inches, meters


# create a class to store and supply the offsets for the reef, based on the AprilTag locations.
#will use may of the same methods as odometrySnap2Line.py

class ReefOffsets:
    # need to initalize the reef offsets and create agetter for the offests from a provided apriltag
    #The offsets will be based on the hard walls around the april tags at feed stations and reef. 
    # The offests to the left and right of the april tag along the wall need to be inidpendently adjustable.
    # The location inputs are from the wall by the robot's width, but there needs to be an alowable adjustment for the width.
    # There returned offsets should be adjusted based on if the left or right side of the robot is going to be against the wall.

    def __init__(self, drivetrain, extra_left_offset: meters=0, extra_right_offset: meters=0):
        self.drivetrain = drivetrain
        self.extra_left_offset = extra_left_offset
        self.extra_right_offset = extra_right_offset
        self.aprilTags = apriltag.AprilTagFieldLayout.loadField(AprilTagConstants.kCompFieldType).getTags()
        self.sidePoleOffest = AprilTagConstants.kPoleOffset
        self.wallOffset = inchesToMeters(RobotDimensions.WIDTH_w_bumpers/2 + AprilTagConstants.kOrigStandoff) 
 
    def getOffsetPathPoints(self, reef_tag, pole_side, robot_side):
        apriltag_center_pose = self.aprilTags[reef_tag].pose
        apriltag_translation = apriltag_center_pose.translation()
        apriltag_rotation = apriltag_center_pose.rotation()
        reef_wall_rotation = apriltag_rotation.rotateBy(Rotation2d(degreesToRadians(-90)))
        pole_side_sign = -1 if pole_side == 'left' else 1
        extra_pole_offset = self.extra_left_offset if pole_side == 'left' else self.extra_right_offset
        self.sidePoleOffest = self.sidePoleOffest + extra_pole_offset
        shooter_path_end = apriltag_translation + Translation2d(distance=pole_side_sign*self.sidePoleOffest,angle= reef_wall_rotation)
        shooter_path_start = shooter_path_end + Translation2d(distance=self.wallOffset,angle= apriltag_rotation)
        # reverse the translation of the robot center to shooter location for the chosen side
        shooter_in_robot_space = RobotDimensions.LEFT_SHOOTER_ROBOT_SPACE if robot_side == 'left' else RobotDimensions.RIGHT_SHOOTER_ROBOT_SPACE
        translation_to_robot_center = Translation2d(distance=shooter_in_robot_space.translation().norm(),angle= apriltag_rotation+shooter_in_robot_space.rotation())
        robot_path_end = shooter_path_end + translation_to_robot_center
        robot_path_start = shooter_path_start + translation_to_robot_center
        # return (robot_path_start, robot_path_end)
        robot_path_end_pose = Pose2d(robot_path_end, apriltag_rotation.rotateBy(degreesToRadians(180)))
        robot_path_start_pose = Pose2d(robot_path_start, apriltag_rotation.rotateBy(degreesToRadians(180)))
        return (robot_path_start_pose, robot_path_end_pose)

