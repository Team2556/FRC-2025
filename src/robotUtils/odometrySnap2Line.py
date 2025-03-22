from commands2 import Command
from wpimath.geometry import Translation2d, Pose2d, Translation3d, Pose3d, Rotation2d, Transform2d, Transform3d, Rotation3d
from wpimath.units import degrees, radians, degreesToRadians, radiansToDegrees, inchesToMeters, inches, meters
from constants import AprilTagField, AprilTags, RobotDimensions
import numpy as np
from phoenix6.utils import get_current_time_seconds
class SnapToLineCommand(Command):
    '''This command does not fully snap the robot to the closest point on a line segment. It actually uses the closest point a a nav update. The error distance si passed as the uncertainty.
    The line segments are based on the hard walls around the april tags at feed stations and reef. There the robot could potentially push against the hard walls.
    The location inputs are from the wall by the robot's width and length. !!! The robot is currently assumed to be square. !!!'''

    def __init__(self, drivetrain, segments: list[tuple[Translation2d, Translation2d]]=None):
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        
        if not segments:
            AprilTags_care_about_short = [tag for  tag in AprilTags if tag.ID  in [ 6,7,8,9,10,11,17,18,19,20,21,22]] 
            AprilTags_care_about_long = [tag for  tag in AprilTags if tag.ID  in [ 1,2,12,13]]
            wall_length_short = inchesToMeters(37)
            wall_length_long = inchesToMeters(76)
            segments = ([self.create_wall_from_center_pose(tag.pose,wall_length_short ) for tag in AprilTags_care_about_short] + 
                        [self.create_wall_from_center_pose(tag.pose,wall_length_long) for tag in AprilTags_care_about_long])
            self.segments = segments #Done: make into real segments, import form constants(could use apriltaglayout to help generate)


    def execute(self):
        current_pose = self.drivetrain.get_state_copy().pose
        square_robot_key_size = inchesToMeters(RobotDimensions.WIDTH_w_bumpers/2)
        offset_segments = [self.rightHand_offset_segment(start, end, square_robot_key_size, square_robot_key_size/2) for start, end in self.segments]
        # print(f'segment compare: {[(f"wall:{wall}",f"offset:{off}") for wall, off in zip(self.segments,offset_segments)]}')
        (trust_snap, snap_dist, closest_point) = self.get_closest_point_on_segments(current_pose.translation(), segments=offset_segments)
        #TODO: add way to address the rotation of the robot or use self.drivetrain.reset_translation(); not sure what sid of the robot is against the wall
        if trust_snap:
            self.drivetrain.add_vision_measurement(Pose2d(closest_point, current_pose.rotation()), timestamp=get_current_time_seconds(), vision_measurement_std_devs=(snap_dist, snap_dist, 3.14))

        else:
            print(f'Not trusting snap, snap_dist: {snap_dist} \n-\n-\n-\n--------------------------------')


    def get_closest_point_on_segments(self, point: Translation2d, segments: list) -> tuple[bool, float, Translation2d]:
        min_snap_dist = float('inf')
        closest_point = None
        for start, end in segments:
            line_vec = end - start
            point_vec = point - start
            line_len = line_vec.norm()
            line_unitvec = line_vec / line_len
            projection_length = np.dot(point_vec.toVector(), line_unitvec.toVector())
            projection_length = max(0, min(line_len, projection_length))
            candidate_point = start + line_unitvec * projection_length
            snap_dist = (point - candidate_point).norm()
            if snap_dist < min_snap_dist:
                min_snap_dist = snap_dist
                closest_point = candidate_point
        snap_trust_limit = 1.0
        trust_snap = min_snap_dist < snap_trust_limit
        return (trust_snap, min_snap_dist, closest_point)

    
    def rightHand_offset_segment(self, start: Translation2d, end: Translation2d, offset: meters, shorten: meters) -> tuple[Translation2d, Translation2d]:
        '''Returns a new segment that is offset to the right of the original segment by the given distance (e.g., width of robot). And shortened ON EACH END by the given distance.'''
        line_vec = end - start
        line_len = line_vec.norm()
        line_unitvec = line_vec / line_len
        # print(f'{line_unitvec=}')
        perp_unitvec = line_unitvec.rotateBy(Rotation2d(degreesToRadians(90)))
        # print(f'{perp_unitvec=}')
        offset_vec = perp_unitvec * offset
        shorten_vec = line_unitvec * shorten
        new_start = start + offset_vec + shorten_vec
        new_end = end + offset_vec - shorten_vec
        return (new_start, new_end)
    
    def create_wall_from_center_pose(self, center_pose: Pose3d, length: float) -> tuple[Translation2d, Translation2d]:
        center_pose = center_pose.toPose2d()
        center_translation = center_pose.translation()
        center_rotation = center_pose.rotation()
        center_wall_rotation = center_rotation.rotateBy(Rotation2d(degreesToRadians(-90)))
        start = center_translation + Translation2d(distance=-inches(length)/2,angle= center_wall_rotation)
        end = center_translation + Translation2d(distance=inches(length)/2,angle= center_wall_rotation)
        return (start, end)


    def isFinished(self):
        return True

    def end(self, somthingelse):
        (print(f' what is second term passed to end of a command? {somthingelse=} \n        ???????????????\n        ????????????????????????\n'))
        pass

    def interrupted(self):
        self.end()