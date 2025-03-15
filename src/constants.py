import numpy
import robotpy_apriltag as apriltag
from enum import IntEnum, auto
import math
from phoenix6.units import *
from wpimath.units import (
    degrees,
    radians,
    degreesToRadians,
    radiansToDegrees,
    inchesToMeters,
    inches,
)
from wpimath.trajectory import TrapezoidProfile
from wpilib import SmartDashboard
from phoenix6.configs.config_groups import Slot0Configs
from pathlib import Path

class AprilTags_height:
    def tag_heights():  # height of apriltags by order of number, in centimeters
        heights = numpy.array(
            [
                135,
                135,
                117,
                178,
                178,
                17,
                17,
                17,
                17,
                17,
                17,
                135,
                135,
                117,
                178,
                178,
                17,
                17,
                17,
                17,
                17,
                17,
            ]
        )
        heights = numpy.array(
            # FIXME: "AprilTags" is not defined   vvvvvvvvv that one
            #[numpy.nan] + [tag.pose.Z for tag in AprilTags]
        )  # put nan at front so index from 1 works better
        return heights

""" ID X Y Z Z-Rotation Y-Rotation (in inches)
 1 657.37 25.80 58.50 126 0
 2 657.37 291.20 58.50 234 0
 3 455.15 317.15 51.25 270 0
 4 365.20 241.64 73.54 0 30
 5 365.20 75.39 73.54 0 30
 6 530.49 130.17 12.13 300 0
 7 546.87 158.50 12.13 0 0
 8 530.49 186.83 12.13 60 0
 9 497.77 186.83 12.13 120 0
 10 481.39 158.50 12.13 180 0
 11 497.77 130.17 12.13 240 0
 12 33.51 25.80 58.50 54 0
 13 33.51 291.20 58.50 306 0
 14 325.68 241.64 73.54 180 30
 15 325.68 75.39 73.54 180 30
 16 235.73-0.15 51.25 90 0
 17 160.39 130.17 12.13 240 0
 18 144.00 158.50 12.13 180 0
 19 160.39 186.83 12.13 120 0
 20 193.10 186.83 12.13 60 0
 21 209.49 158.50 12.13 0 0
 22 193.10 130.17 12.13 300 0"""


# region RoboRio Constants
# included to help with communication and readability
class Rio_DIO(IntEnum):
    ZERO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()
    EIGHT = auto()
    NINE = auto()
    TEN = auto()
    ELEVEN = auto()
    TWELVE = auto()
    THIRTEEN = auto()
    FOURTEEN = auto()
    FIFTEEN = auto()
    SIXTEEN = auto()
    SEVENTEEN = auto()


class Rio_Pnue(IntEnum):
    ZERO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()


class Rio_PWM(IntEnum):
    ONE = 0
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()
    EIGHT = auto()
    NINE = auto()
    TEN = auto()


class Rio_Relay(IntEnum):
    ZERO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()


class Rio_Analog(IntEnum):
    ZERO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()

# TODO: Do we really need this (i guess so?)
class CAN_Address(IntEnum):
    ZERO = 0
    ONE = auto()
    TWO = auto()
    THREE = auto()
    FOUR = auto()
    FIVE = auto()
    SIX = auto()
    SEVEN = auto()
    EIGHT = auto()
    NINE = auto()
    TEN = auto()
    ELEVEN = auto()
    TWELVE = auto()
    THIRTEEN = auto()
    FOURTEEN = auto()
    FIFTEEN = auto()
    SIXTEEN = auto()
    SEVENTEEN = auto()
    EIGHTEEN = auto()
    NINETEEN = auto()
    TWENTY = auto()
    TWENTYONE = auto()
    TWENTYTWO = auto()
    TWENTYTHREE = auto()
    TWENTYFOUR = auto()
    TWENTYFIVE = auto()
    TWENTYSIX = auto()
    TWENTYSEVEN = auto()
    TWENTYEIGHT = auto()
    TWENTYNINE = auto()
    THIRTY = auto()
    THIRTYONE = auto()
    THIRTYTWO = auto()
    THIRTYTHREE = auto()
    THIRTYFOUR = auto()
    THIRTYFIVE = auto()
    THIRTYSIX = auto()
    THIRTYSEVEN = auto()
    THIRTYEIGHT = auto()
    THIRTYNINE = auto()
    FORTY = auto()
    FORTYONE = auto()
    FORTYTWO = auto()
    FORTYTHREE = auto()
    FORTYFOUR = auto()
    FORTYFIVE = auto()
    FORTYSIX = auto()
    FORTYSEVEN = auto()
    FORTYEIGHT = auto()
    FORTYNINE = auto()
    FIFTY = auto()
    FIFTYONE = auto()
    
class RobotDimensions:
    WIDTH_w_bumpers = inches(36)  # inches inchesToMeters(36)#(26+2*3.25)

class Override_DriveConstant: ...

# NOTE: ALL OF THE BELOW CLASSES ARE FOR SUBSYSTEMS

class ElevatorConstants:
    kLeftMotorPort = CAN_Address.THIRTEEN
    kRightMotorPort = CAN_Address.FOURTEEN
    kJoystickPort = 0
    kpeak_forward_torque_current = 35  # 120
    kpeak_reverse_torque_current = -35  # -120
    kincrement_m_per_sec_held = 0.25
    
    # All the speed stuff
    kHomingRate = 1 / 10  # 1 meter every 10 seconds
    kElevatorSpeed = 1 / 10 # 1 meter every 10 seconds

    kElevatorKp = 5.0
    kElevatorKi = 0.0
    kElevatorKd = 0.0
    kElevatorGearing = 6  # 10.0
    kElevatorDrumRadius = 0.035 / 2  # half of 35mm in meters
    kCarriageMass = 22.68  # 4 kg

    kMinElevatorHeight = 0.00 #0.0508  # 2 inches
    kMaxElevatorHeight = inchesToMeters(26)  # 50 inches TODO: make this smaller
    # kElevatorDistanceMovedAfterContactWithLimitSwitch = 0.00002 sorry Jack
    
    ScaredSafetyFactor = 5 # Set ScaredSafetyFactor to 1 once we get SUPER confident
    # All the elevator levels to do stuff
    kCoralIntakePosition = -0.05 #/ ScaredSafetyFactor
    # kCoralLv1 = 0.1 / ScaredSafetyFactor # Height in meters
    kCoralLv2 = 0.1 #/ ScaredSafetyFactor # 556
    kCoralLv3 = 0.2 #/ ScaredSafetyFactor
    kCoralLv4 = 0.25 #/ ScaredSafetyFactor # All the elevator levels below aren't tuned
    kAlgaeProcess = 0.15 #/ ScaredSafetyFactor
    kAlgaeGroundIntake = 0.21 #/ ScaredSafetyFactor
    kAlgaeLv2 = 0.2 #/ ScaredSafetyFactor
    kAlgaeLv3 = 0.4 #/ ScaredSafetyFactor
    kZero = 0

    # The command decides the position's close enough if it's within this range (in meters?)
    kTargetValueAccuracy = 0.02
    kTargetValueAdder = 0 # If it setpoints to a value a bit more or less than you want to to

    kMaxVelocityMetersPerSecond = 1.5/ScaredSafetyFactor
    kMaxAccelerationMetersPerSecSquared = 0.5/ScaredSafetyFactor

    kSVolts = 0
    kGVolts = 0.01

    kVVoltSecondPerMeter = 0 #1.5
    kAVoltSecondSquaredPerMeter = 0 #0.75

    kElevatorOffsetMeters = 0 #Used in softlimit minimum

    kBottomLeftLimitSwitchChannel = Rio_DIO.EIGHT
    kBottomRightLimitSwitchChannel = Rio_DIO.NINE
    # kTopLeftLimitSwitchChannel = Rio_DIO.TWO
    kTopRightLimitSwitchChannel = Rio_DIO.SEVEN  # TODO: ? two on top also?


# endregion
class Override_DriveConstant: ...


class AlgaeConstants:
    # Motor Channels
    kPivotMotorChannel = CAN_Address.TWENTYTHREE
    kIntakeWheelsChannel = CAN_Address.TWENTYTWO
    
    # Limit Switch channel (So it doesn't input when limit switch activated)
    kBottomLimitSwitchChannel = Rio_DIO.FIVE # TODO: Add more actual CAN IDs
    kTopLimitSwitchChannel = Rio_DIO.SIX
    
    # This is so it doesn't move too fast in one way? 
    # (to disable just set to super high positive/negative numbers)
    kPeakForwardTorqueCurrent = 20
    kPeakReverseTorqueCurrent = -20
    
    # All the following stuff are tunable in SmartDashboard
    kPivotp = 2
    kPivoti = 0
    kPivotd = 0
    kPivotg = 0
    
    # If the motor is stalled then it's trying to intake an algae more than it can
    # So this detects if it shouldn't spin anymore
    kAmpValueToDetectIfMotorStalled = 90
    
    # The command decides the position's close enough if it's within this range (in rotations)
    kTargetValueAccuracy = 0.01
    
    kPivotMaxHeight = 0.25
    kPivotMinHeight = 0
    
    # Values to set pivoting motor to
    kPivotReefIntakingValue = 2.1 # Pivot position when grabbing algae
    kPivotGroundIntakingValue = 1.3 # Pivot position when grabbing algae from the FLOOR (not being used)
    kPivotAfterGroundIntakingValue = 1
    kPivotProcessingValue = 1.2 # Pivot position when about to send to processor
    kPivotIdleValue = 0.05 # Pivot position when idle
    # The time it takes to switch between pivoting positions
    kPivotRotationsPerSecond = 2
    
    # Intake wheels multiply by this speed
    kIntakeMultiplier = 0.5
    
    # The code waits this many seconds between intaking/processing
    kTimeItTakesToIntake = 1
    kTimeItTakesToProcess = 1

class CoralConstants:
    kCoralMotorPort = CAN_Address.THIRTY
    
    kLeftBreakerLight = Rio_DIO.ZERO # TODO: Get the actual IDs
    kRightBreakerLight = Rio_DIO.ONE
    
    kLeftFlipper = 1 # IDs for the pneumatics flippers
    kRightFlipper = 2
    
    kIntakeMultiplier = 0.15
    kDischargeMultiplier = 0.25
    
    kFlipperPulseDuration = 1
    
    # Not currently being used
    kDelayBetweenLeavingBeamBreaksAndActivatingFlippers = 0.3
    
    kHighEnoughToActivateFlippers = 0.8 # I think it's in meters (where 0 is lowest physical point on elevator)

class ClimbConstants:
    kClimbMotorPort = CAN_Address.TWENTYSEVEN
    kTopLimitSwitchChannel = Rio_DIO.FIVE
    kBottomLimitSwitchChannel = Rio_DIO.SIX
    kClimbHookZeroEntry = 1
    kSpeed = 1.0
    kMaxAccelerationMetersPerSecondSquared = 0.5
    #kPClimbController = 1
    #kIClimbController = 0
    #kDClimbController = 0
    kClimbMaxHeight = 1.6081375
    kClimbMinHeight = 0.815975
    # kClimbSlightlyExtendedHeight = 0.9
    # kClimbMinHeightEncoderEstimate = -3583.037109375
    kS = 0
    kV = 0
    kA = 0
    kClimbMaxPosConfirmationExtraHeight = 0.1
    GEAR_RATIO = 15376/45
    kPositionConversionFactor = 0.1965
    GAINS = (Slot0Configs()
                .with_k_p(1.0)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.0)
                .with_k_v(0.0)
                .with_k_a(0.0)
            )

class PneumaticConstants:
    kHub = CAN_Address.FORTY

class PowerDistributionConstants:
    kPDP = CAN_Address.FIFTY

class UltrasonicConstants:
    frontLeft = Rio_DIO.TWO
    frontRight = Rio_DIO.THREE
    # backLeft = Rio_DIO.FOUR
    # backRight = Rio_DIO.THREE


class LimelightConstants:
    # for field map replacr src with /home/lvuser/py/
    '''if Path('/').resolve() == Path('/'):  # Check if root is actually root (Linux/RoboRIO)
        field_map_folder = Path('/home/lvuser/py/WPIcalFieldToUse/')
    else:  # We're on Windows
        field_map_folder = Path('WPIcalFieldToUse/')
    # field_map_folder = Path('/home/lvuser/py/WPIcalFieldToUse/')
    field_map_address = str([i for i in field_map_folder.glob(pattern='*fmap', case_sensitive=False)][0])'''
    #TODO Update the location values
    kLL3forward = -0.383
    kLL3side = 0.0
    kLL3up = 0.167
    kLL3roll = 0.0
    kLL3pitch = 25.1
    kLL3yaw = 180
    
    kLL4forward = 0.0
    kLL4side = -0.365
    kLL4up = 0.217
    kLL4roll = 2.8
    kLL4pitch = 37.6
    kLL4yaw = -90

    