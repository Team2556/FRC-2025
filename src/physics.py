import phoenix6
import wpilib
from wpilib import RobotController, DriverStation
import wpilib.simulation as sim
import wpimath.controller
import wpimath.estimator
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Transform2d, Pose3d, Rotation3d, Transform3d
from wpimath.units import degrees, radians, degreesToRadians, radiansToDegrees, inchesToMeters, inches, meters, feetToMeters, metersToFeet, radiansToRotations
from wpimath.system.plant import DCMotor, LinearSystemId

import robotpy_apriltag as apriltag
import ntcore
from photonlibpy import PhotonCamera
from photonlibpy.simulation import VisionSystemSim, VisionTargetSim, PhotonCameraSim
from constants import AprilTags, AprilTagField
# TODO: add the camera location to constants and import


from phoenix6.unmanaged import feed_enable
from phoenix6.hardware import TalonFX
import phoenix6.sim as phoenix6_sim
import typing
import wpimath.system.plant


from constants import ElevatorConstants
import subsystems
# import subsystems.elevatorSubsystem

if typing.TYPE_CHECKING:
    from robot import MyRobot

# import generated.tuner_constants as tc
# from wpimath.units import inchesToMeters, feetToMeters, metersToFeet
# from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain


class PhysicsEngine:
    def __init__(self, physics_controller, robot: "MyRobot"):
        self.physics_controller = physics_controller
        self.robot = robot
        self.container = self.robot.container
        self.drivetrain = self.container.drivetrain
        self.prev_pose = self.drivetrain.get_state().pose
        self.elevatorGearbox = wpimath.system.plant.DCMotor.krakenX60(2)
        self.sim_elevator_model =  sim.DCMotorSim(
            plant= LinearSystemId.DCMotorSystem( self.elevatorGearbox,.0001, ElevatorConstants.kElevatorGearing),
            gearbox=self.elevatorGearbox, measurementStdDevs= [0.01,0]
            )
        self.elevator_physics_model = sim.ElevatorSim(
            plant=LinearSystemId.elevatorSystem(motor=self.elevatorGearbox,
                                                mass= ElevatorConstants.kCarriageMass,
                                                 radius=ElevatorConstants.kElevatorDrumRadius,
                                                  gearing= ElevatorConstants.kElevatorGearing ),#  LinearSystemId.DCMotorSystem( self.elevatorGearbox,.0001, ElevatorConstants.kElevatorGearing),
            gearbox=self.elevatorGearbox,
            minHeight=ElevatorConstants.kMinElevatorHeight,
            maxHeight=ElevatorConstants.kMaxElevatorHeight,
            simulateGravity=True,
            startingHeight=.2,
            measurementStdDevs=[0.0, 0.0]
            )
        # def __init__(self, plant: wpimath._controls._controls.system.LinearSystem_2_1_2, gearbox: wpimath._controls._controls.plant.DCMotor, minHeight: wpimath.units.meters, maxHeight: wpimath.units.meters, simulateGravity: bool, startingHeight: wpimath.units.meters, measurementStdDevs: typing.Annotated[list[float], pybind11_stubgen.typing_ext.FixedSize(2)] = [0.0, 0.0]) -> None:
        """
        Constructs a simulated elevator mechanism.
        
        :param plant:              The linear system that represents the elevator.
                                   This system can be created with
                                   LinearSystemId::ElevatorSystem().
        :param gearbox:            The type of and number of motors in your
                                   elevator gearbox.
        :param minHeight:          The minimum allowed height of the elevator.
        :param maxHeight:          The maximum allowed height of the elevator.
        :param simulateGravity:    Whether gravity should be simulated or not.
        :param startingHeight:     The starting height of the elevator.
        :param measurementStdDevs: The standard deviation of the measurements.
        """


        # self.sim_oneMotor = phoenix6.sim.TalonFXSimState(self.container.one_motor.motor)
        if self.container.ENABLE_ELEVATOR:
            self.sim_elevator_motor_left = self.container.elevator.elevmotor_left.sim_state#phoenix6.sim.TalonFXSimState(self.container.elevator.elevmotor_left)
            self.sim_elevator_motor_right = self.container.elevator.elevmotor_right.sim_state#phoenix6.sim.TalonFXSimState(self.container.elevator.elevmotor_right)
            self.sim_elevator_encoder_left = self.container.elevator.elevCANcoder_left.sim_state #phoenix6.sim.CANcoderSimState(self.container.elevator.elevCANcoder_left)
            self.sim_elevator_encoder_right = self.container.elevator.elevCANcoder_right.sim_state #phoenix6.sim.CANcoderSimState(self.container.elevator.elevCANcoder_right)
            self.sim_elevator_motor_left.orientation = phoenix6_sim.ChassisReference.CounterClockwise_Positive
            self.sim_elevator_motor_right.orientation = phoenix6_sim.ChassisReference.CounterClockwise_Positive
        
        self.field = self.robot.field #wpilib.Field2d()#already put a field on smartdashboard from robot or container..
        # SmartDashboard.putData("Field-Vision", self.field)

        #init photon camera
        self.photon_camera_left = PhotonCamera('photonvision_left')
        self.photon_camera_right = PhotonCamera('photonvision_right')
        #setup photon camera physical location on robot
        #init simvision 
        self.photon_camera_sim_left =PhotonCameraSim(self.photon_camera_left)
        self.photon_camera_sim_right =PhotonCameraSim(self.photon_camera_right)
        self.photon_camera_sim_left.setMaxSightRange(4)#meters
        self.photon_camera_sim_right.setMaxSightRange(4)#meters
        self.sim_vision = VisionSystemSim("SimPhoton_system")
        self.sim_vision.addCamera(self.photon_camera_sim_left, Transform3d(x=inchesToMeters(4), 
                                                                           y=inchesToMeters((28/2)-2), 
                                                                           z=inchesToMeters(8), 
                                                                           rotation=Rotation3d(0, 0, degreesToRadians(90))))
        self.sim_vision.addCamera(self.photon_camera_sim_right, Transform3d(x=inchesToMeters(4), 
                                                                            y=inchesToMeters(-((28/2)-2)), 
                                                                            z=inchesToMeters(8), 
                                                                            rotation=Rotation3d(0, 0, degreesToRadians(-90))))
        self.sim_vision.addAprilTags(AprilTagField)

        
        #doesn't work ... temp = self.container.drivetrain.get_module(0).sim_state
        # for each motor in each module in the drivetrain, create a simulated motor and encoder
        # for module in self.drivetrain.modules:
        #     for motor in module:
        #         temp = motor.sim_state



        '''
        # self.leftelevmotor = self.container.leftelevmotor  TalonFX(robot.leftelevmotor.get())
        # self.rightelevmotor = TalonFX(robot.rightelevmotor.get())
        self.sim_leftelevmotor = phoenix6.sim.TalonFXSimState(self.elevator.elevmotor_left)
        self.sim_rightelevmotor = phoenix6.sim.TalonFXSimState(self.elevator.elevmotor_right)
        # self.sim_leftelevencoder = phoenix6.sim.EncoderSimState(self.container.leftelevencoder)
        # self.sim_rightelevencoder = phoenix6.sim.EncoderSimState(self.container.rightelevencoder)
        # wpilib.Encoder(robot.elevencoder) #not sure if i need to add another encoder, will find out later
        '''
        # Create a Mechanism2d display of an elevator
        if self.container.ENABLE_ELEVATOR:
            self.mech2d = wpilib.Mechanism2d(20, 50)
            self.elevatorRoot = self.mech2d.getRoot("Elevator Root", 10, 0)
            self.elevatorMech2d = self.elevatorRoot.appendLigament(
                "Elevator", self.elevator_physics_model.getPositionInches(), 90
            )
            # Put Mechanism to SmartDashboard
            wpilib.SmartDashboard.putData("Elevator", self.mech2d)

            
        
    def update_sim(self, now, tm_diff):
        # Currently, the Python API for CTRE doesn't automatically detect the the
        # Sim driverstation status and enable the signals. So, for now, manually
        # feed the enable signal for double the set robot period.
        feed_enable(0.020 * 2)

        robot_pose = self.drivetrain.get_state().pose
        self.field.setRobotPose(robot_pose) 
        # self.photon_camera_sim.  .setRobotPose(robot_pose)
        self.sim_vision.update(robotPose=robot_pose)
        if self.container.ENABLE_ELEVATOR:
            self.sim_elevator_motor_left.set_supply_voltage(RobotController.getBatteryVoltage())
            self.sim_elevator_motor_right.set_supply_voltage(RobotController.getBatteryVoltage())
            self.sim_elevator_model.setInputVoltage(self.sim_elevator_motor_left.motor_voltage)
            self.sim_elevator_model.update(tm_diff)
            self.sim_elevator_motor_left.set_raw_rotor_position(radiansToRotations(self.sim_elevator_model.getAngularPosition()))
            self.sim_elevator_motor_left.set_rotor_velocity(radiansToRotations(self.sim_elevator_model.getAngularVelocity()))
            self.sim_elevator_motor_right.set_raw_rotor_position(radiansToRotations(self.sim_elevator_model.getAngularPosition()))
            self.sim_elevator_motor_right.set_rotor_velocity(radiansToRotations(self.sim_elevator_model.getAngularVelocity()))

        #refer to https://api.ctr-electronics.com/phoenix6/release/python/autoapi/phoenix6/sim/talon_fx_sim_state/index.html

        # drivetrainKinematics = SwerveDrive4Kinematics([module.pose for module in self.drivetrain.modules])
        # self.physics_controller.setPose(self.drivetrain.pose)
        # print("------------------------------------------------\n",f'{self.sim_front_left_drive.motor_voltage=}',
        #       "------------------------------------------------\n",f'{self.sim_front_left_steer.motor_voltage=}/n/n')
        # for module in self.drivetrain.modules:
        #     for module_locations in module.module_locations:
        #         print("------------------------------------------------\n",f'{module}at{module_locations=}',
        #         "------------------------------------------------\n")
        # print("------------------------------------------------\n",f'{feetToMeters(self.drivetrain.get_state().speeds)=}',"------------------------------------------------\n",f'{(self.drivetrain.get_state().speeds)=}')  
        #may need unit conversion, may not..? metersToFeet or feetToMeters
        # self.physics_controller.drive(-(self.drivetrain.get_state().speeds), 0.020)
        # self.physics_controller.drive(-(self.drivetrain.get_state().speeds), 0.020)
        if self.container.ENABLE_ELEVATOR:
            self.elevator_physics_model.setInput(
                0, 
                self.sim_elevator_motor_left.motor_voltage
                # self.sim_leftelevmotor.setVoltage() * phoenix6.hardware.TalonFX.setVoltage()
            )
            self.elevator_physics_model.setInput(
                1, 
                self.sim_elevator_motor_right.motor_voltage
                # self.sim_rightelevmotor.setVoltage() * phoenix6.hardware.TalonFX.setVoltage()
            )

            # Next, we update it
            self.elevator_physics_model.update(tm_diff)
            # Update the Elevator length based on the simulated elevator height
            self.elevatorMech2d.setLength(self.elevator_physics_model.getPositionInches())
            '''

            # Finally, we set our simulated encoder's readings and simulated battery
            # voltage
            self.elevencoder.getDistance(self.elevator.getPosition())
            # SimBattery estimates loaded battery voltage
            # wpilib.simulation.RoboRioSim.setVInVoltage(
            #     wpilib.simulation.BatterySim
            # )


        do_nothing = True
        if do_nothing:
            pass
        else:
            self.physics_controller.move_elevator(Transform2d(self.prev_pose, self.drivetrain.get_state().pose))
            self.prev_pose = self.drivetrain.get_state().pose'''

            

