
import robotpy_apriltag as apriltag
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from wpimath.units import degreesToRadians, radiansToDegrees, inchesToMeters, meters

from constants import AprilTagConstants, RobotDimensions


# create a class to store and supply the offsets for the reef, based on the AprilTag locations.
#will use may of the same methods as odometrySnap2Line.py

class ReefOffsets:
    # need to initalize the reef offsets and create agetter for the offests from a provided apriltag
    #The offsets will be based on the hard walls around the april tags at feed stations and reef. 
    # The offests to the left and right of the april tag along the wall need to be inidpendently adjustable.
    # The location inputs are from the wall by the robot's width, but there needs to be an alowable adjustment for the width.
    # There returned offsets should be adjusted based on if the left or right side of the robot is going to be against the wall.

    def __init__(self, extra_left_offset: meters=0, extra_right_offset: meters=0):
        self.extra_left_offset = extra_left_offset
        self.extra_right_offset = extra_right_offset
        self.aprilTags = apriltag.AprilTagFieldLayout.loadField(AprilTagConstants.kCompFieldType).getTags()
        # print(f'Initalizing april tags {self.aprilTags=}')
        # self.sidePoleOffest = AprilTagConstants.kPoleOffset
        self.wallOffset = inchesToMeters(RobotDimensions.WIDTH_w_bumpers/2 + AprilTagConstants.kOrigStandoff) 

        self.tag_alignment_poses = {'robot_left':{'poleLeft':{tag.ID: self.getOffsetPathPoints(tag.ID, 'left', 'left') for tag in self.aprilTags},
                                        'poleRight':{tag.ID: self.getOffsetPathPoints(tag.ID, 'right', 'left') for tag in self.aprilTags}},
                          'robot_right':{'poleLeft':{tag.ID: self.getOffsetPathPoints(tag.ID, 'left', 'right') for tag in self.aprilTags},
                                         'poleRight':{tag.ID: self.getOffsetPathPoints(tag.ID, 'right', 'right') for tag in self.aprilTags}}}
        
        self.tag_alignment_inital_poses = {'robot_left':{'poleLeft':{tag.ID: self.getOffsetPathPoints(tag.ID, 'left', 'left', return_path_start=True) for tag in self.aprilTags},
                                                         'poleRight':{tag.ID: self.getOffsetPathPoints(tag.ID, 'right', 'left', return_path_start=True) for tag in self.aprilTags}},
                                            'robot_right':{'poleLeft':{tag.ID: self.getOffsetPathPoints(tag.ID, 'left', 'right', return_path_start=True) for tag in self.aprilTags},
                                                        'poleRight':{tag.ID: self.getOffsetPathPoints(tag.ID, 'right', 'right', return_path_start=True) for tag in self.aprilTags}}}
        # , 'poleRight':{}}
    
    def pose_extract_array(self, pose):
            try:
                array = [
                pose.translation().X(),
                pose.translation().Y(),
                radiansToDegrees(pose.rotation().radians())
            ]
            except:
                array = [
                pose.translation().X(),
                pose.translation().Y(),
                radiansToDegrees(pose.rotation().z)
            ]
            return array
 
    def getOffsetPathPoints(self, reef_tag, pole_side, robot_side, return_path_start=False):
        reef_tag_index = reef_tag-1
        transpose_steps =[]
        apriltag_center_pose = self.aprilTags[reef_tag_index].pose
        transpose_steps.append(self.pose_extract_array(apriltag_center_pose))
        apriltag_translation = Translation2d(apriltag_center_pose.translation().x, apriltag_center_pose.translation().y)
        apriltag_rotation = Rotation2d(apriltag_center_pose.rotation().z)
        reef_wall_rotation = apriltag_rotation + Rotation2d(degreesToRadians(-90))# apriltag_center_pose.rotateBy(Rotation3d(0,0, degreesToRadians(-90)))
        # reef_wall_rotation = Rotation2d(reef_wall_rotation.z)
        #Pose2d(apriltag_center_pose.translation(), apriltag_center_pose.rotation() Rotation2d(degreesToRadians(-90)))
        #
        pole_side_sign = 1 if pole_side == 'left' else -1
        extra_pole_offset = self.extra_left_offset if pole_side == 'left' else self.extra_right_offset
        self.sidePoleOffest = AprilTagConstants.kPoleOffset + extra_pole_offset
        shooter_path_end = apriltag_translation + Translation2d(distance=pole_side_sign*self.sidePoleOffest,angle= reef_wall_rotation)
        transpose_steps.append(self.pose_extract_array(Pose2d(shooter_path_end, reef_wall_rotation)))
        shooter_path_start = shooter_path_end + Translation2d(distance=self.wallOffset,angle= apriltag_rotation)
        transpose_steps.append(self.pose_extract_array(Pose2d(shooter_path_start, reef_wall_rotation)))
        # reverse the translation of the robot center to shooter location for the chosen side
        # shooter_in_robot_space = RobotDimensions.ROBOT_CENTER_FROM_LEFT_SHOOTER if robot_side == 'left' else RobotDimensions.ROBOT_CENTER_FROM_RIGHT_SHOOTER
        shooter_in_robot_space_with_Angle = RobotDimensions.LEFT_SHOOTER_ROBOT_SPACE if robot_side == 'left' else RobotDimensions.RIGHT_SHOOTER_ROBOT_SPACE
        shooter_in_robot_space = shooter_in_robot_space_with_Angle.translation()
        shooter_angel_on_robot = shooter_in_robot_space_with_Angle.rotation()
        translation_to_robot_center = Translation2d(distance=shooter_in_robot_space.norm(),
                                                    angle= apriltag_rotation
                                                    +shooter_in_robot_space.angle()
                                                    -shooter_angel_on_robot
                                                    -Rotation2d(degreesToRadians(-180))
                                                    )
        robot_path_end = shooter_path_end + translation_to_robot_center
        robot_path_start = shooter_path_start + translation_to_robot_center
        # return (robot_path_start, robot_path_end)
        robot_path_end_pose = Pose2d(robot_path_end, apriltag_rotation + (Rotation2d(shooter_angel_on_robot.radians())))
        robot_path_start_pose = Pose2d(robot_path_start, apriltag_rotation + (Rotation2d(shooter_angel_on_robot.radians())))
        transpose_steps.append(self.pose_extract_array(robot_path_start_pose))
        transpose_steps.append(self.pose_extract_array(robot_path_end_pose))
        # return (robot_path_start_pose,shooter_path_start, robot_path_end_pose,shooter_path_end, apriltag_translation, apriltag_rotation, self.sidePoleOffest)
        # return ( robot_path_end_pose, shooter_path_end, apriltag_translation, apriltag_rotation, self.sidePoleOffest, transpose_steps)
        if return_path_start:
            return (robot_path_start_pose)
        else:
            return ( robot_path_end_pose)

