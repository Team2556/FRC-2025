import self
from commands2 import Command
from pathplannerlib.path import PathPlannerPath, PathConstraints, GoalEndState
from wpimath.geometry import Pose2d, Rotation2d
import math

from src.subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
from src.subsystems.vison import VisionSubsystem


def get_fiducial_id(limelight_name):
    pass


class PathOnTheFlyAutoAlign(Command):
    def __init__(self, drivetrain: CommandSwerveDrivetrain, vision: VisionSubsystem):
        super().__init__()

    #self.target_tag = get_fiducial_id(limelight_name: str)

    # Create a list of waypoints from poses. Each pose represents one waypoint.
    # The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
    reefWaypoints = PathPlannerPath.waypointsFromPoses([
        Pose2d(6.4, 4.0, Rotation2d.fromDegrees(180)),
        Pose2d(6.0, 4.0, Rotation2d.fromDegrees(180))
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
    reefConstraints = PathConstraints(0.5, 0.5, (2 * math.pi)/2, (4 * math.pi)/2) # The constraints for this path.
    # constraints = PathConstraints.unlimitedConstraints(12.0) # You can also use unlimited constraints, only limited by motor torque and nominal battery voltage
    # Create the path using the waypoints created above
    testPath = PathPlannerPath(
        reefWaypoints,
        reefConstraints,
        None, # The ideal starting state, this is only relevant for pre-planned paths, so can be None for on-the-fly paths.
        GoalEndState(0.0, Rotation2d.fromDegrees(90)) # Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    )

    # Prevent the path from being flipped if the coordina6es are already correct
    testPath.preventFlipping = True