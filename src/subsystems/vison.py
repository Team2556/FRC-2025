import concurrent.futures
import math

from commands2 import Subsystem
from phoenix6 import utils
from wpilib import SmartDashboard

from lib.limelight import PoseEstimate, LimelightHelpers
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain


class VisionSubsystem(Subsystem):
    """
    Handles all camera calculations on the robot.
    This is primarily used for combining MegaTag pose estimates and ensuring no conflicts between Limelights.

    We use the starting position in auto to determine our robot heading to calibrate our cameras.
    """

    def __init__(self, swerve: CommandSwerveDrivetrain, *cameras: str):
        super().__init__()

        self._swerve = swerve
        self._cameras = tuple(cameras)

        if not all(isinstance(cam, str) for cam in self._cameras):
            raise TypeError(f"All cameras must be strings! Given: {self._cameras}")

        self._executor = concurrent.futures.ThreadPoolExecutor()

    def periodic(self):
        super().periodic()

        if abs(self._swerve.pigeon2.get_angular_velocity_z_world().value) > 720:
            return

        futures = [
            self._executor.submit(self._process_camera, cam)
            for cam in self._cameras
        ]

        for future in concurrent.futures.as_completed(futures):
            estimate = future.result()
            if estimate and estimate.tag_count > 0:
                SmartDashboard.putData("Number of Tags", estimate.tag_count)
                SmartDashboard.putData("Pose", estimate.pose)
                self._swerve.add_vision_measurement(
                    estimate.pose,
                    utils.fpga_to_current_time(estimate.timestamp_seconds),
                    self._get_dynamic_std_devs(estimate),
                )

    def _process_camera(self, camera: str) -> PoseEstimate | None:
        """ Retrieves pose estimate for a single camera and ensures it's closer to expected than the last one. """
        state = self._swerve.get_state_copy().pose.rotation()
        LimelightHelpers.set_robot_orientation(
            camera,
            state.degrees(),
            0,0, 0, 0, 0
        )
        pose = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(camera)

        if pose is None or pose.tag_count == 0:
            return None  # Reject immediately if invalid
        return pose

    @staticmethod
    def _get_dynamic_std_devs(estimate: PoseEstimate) -> tuple[float, float, float]:
        """ Computes dynamic standard deviations based on tag count and distance. """
        if estimate.tag_count == 0:
            return 0.5, 0.5, 0.5

        avg_dist = sum(f.dist_to_camera for f in estimate.raw_fiducials) / estimate.tag_count
        factor = 1 + (avg_dist ** 2 / 30)

        return 0.5 * factor, 0.5 * factor, math.inf if estimate.is_megatag_2 else (0.5 * factor)

    def get_fiducial_with_id(self, target_id: int):
        tags = LimelightHelpers.get_raw_fiducials(self._cameras[0])

        for tag in tags:
            if tag.id == target_id:
                return tag

        return None


