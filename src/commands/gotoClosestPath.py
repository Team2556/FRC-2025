from commands2 import Command
from pathplannerlib.path import PathPlannerPath, PathConstraints
from pathplannerlib.auto import AutoBuilder
# from wpimath.geometry import Pose2d, Translation2d, Rotation2d, Transform2d
from wpimath.units import degreesToRadians
import math
from phoenix6 import swerve


class GotoClosestPath(Command):
    def __init__(self, drivetrain, paths: list[PathPlannerPath]):
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self.paths = paths
        self._brake = swerve.requests.SwerveDriveBrake()
        # self.current_pose = self.drivetrain.get_state_copy().pose
        # self.closest_path = self.closest_path_to_robot()
        print(f'Initiallizing GotoClosestPath  {AutoBuilder.isConfigured()=} {AutoBuilder._getPose()=}!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')


    def closest_path_to_robot(paths) -> PathPlannerPath:
        closest_path_to_robot_lam = lambda paths: min(paths, key=lambda path: path._waypoints[0].anchor.distance(AutoBuilder._getPose().translation()))
        close_one = closest_path_to_robot_lam(paths)
        print(f'{close_one=} FROM the Commands file m55555555555555555555555555555555555555555555555555555555555555555555555555555555555555555')
        return close_one

    def initialize(self):
        print(f'Initializing GotoClosestPath with def initialize.......................-----------------------------------------------')
        closest_path_to_robot_lam = lambda paths: min(paths, key=lambda path: path._waypoints[0].anchor.distance(self.drivetrain.get_state_copy().pose.translation()))
        self.closest_path = closest_path_to_robot_lam(self.paths)
        print(f'{self.closest_path=}')

    def execute(self):
        self.current_pose = self.drivetrain.get_state_copy().pose
        print(f'Executing GotoClosestPath at translation {self.current_pose} {AutoBuilder._getPose()=}within def execute./n.{self.closest_path=}.....................-----------------------------------------------')
        # print(f'{self.drivetrain.isScheduled()=}')
            #lambda: self._brake)
        
        # self._brake
        
        AutoBuilder.pathfindThenFollowPath(
            goal_path= self.closest_path,
            pathfinding_constraints=PathConstraints(3.0, 4.0, degreesToRadians(540), degreesToRadians(720),12,False),
        )
        print(f' got past the  [SELF._BRAKE] Autobuilder Executing GotoClosestPath with def execute.......................-----------------------------------------------')
    
    def isFinished(self):
        return False

    def end(self, something): 
        pass

    def interrupted(self):
        self.end()