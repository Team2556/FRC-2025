import math
from enum import IntEnum, auto

import robotpy_apriltag as apriltag
from wpimath.geometry import (
    Translation2d,
    Rotation2d,
    Transform2d,
)
from wpimath.units import (
    degreesToRadians,
    inchesToMeters,
    inches,
)


class AprilTagConstants:
    kCompFieldType = apriltag.AprilTagField.k2025ReefscapeWelded
    kPoleOffset = inchesToMeters(inches(12.94)/2  )
    kOrigStandoff = 1 # meter  inchesToMeters(inches(16) ) # 12.94

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
    WIDTH_w_bumpers = inches(28+2*3.25)  # inches inchesToMeters(36)#(26+2*3.25)
    ROBOT_CENTER_FROM_LEFT_SHOOTER = Translation2d(inchesToMeters(inches(-9)), inchesToMeters(inches(-18.5)))
    ROBOT_CENTER_FROM_RIGHT_SHOOTER = Translation2d(inchesToMeters(inches(-9)), inchesToMeters(inches(18.5)))
    LEFT_SHOOTER_ROBOT_SPACE = Transform2d(ROBOT_CENTER_FROM_LEFT_SHOOTER, Rotation2d(degreesToRadians(90)))
    RIGHT_SHOOTER_ROBOT_SPACE = Transform2d(ROBOT_CENTER_FROM_RIGHT_SHOOTER, Rotation2d(degreesToRadians(-90)))


# NOTE: ALL OF THE BELOW CLASSES ARE FOR SUBSYSTEMS

class ElevatorConstants:
    kLeftMotorPort = CAN_Address.THIRTEEN
    kRightMotorPort = CAN_Address.FOURTEEN

    kElevatorSensorToMech = (1 / 6) * 1.4397 * math.pi
    
    kpeak_forward_torque_current = 35  # 120
    kpeak_reverse_torque_current = -35  # -120
    # kincrement_m_per_sec_held = 0.25
    
    # THIS ONE has a max apeed of 1, so 0.10 is 10% of elevator's max speed
    kHomingRate = 0.2
    # So the robot doesn't slam into the ground
    kLowEnoughToSlowDown = 6
    kLowEnoughSpeedMultiplier = 0.39
    
    # All the speed stuff (in rotations per second)
    kElevatorSpeed = 1.5 # was too fast 2.5 # 10
    
    kElevatorKp = 0.4
    kElevatorKi = 0.0
    kElevatorKd = 0.0
    kElevatorKg = 0.4

    kMinElevatorHeight = 0
    kMaxElevatorHeight = 38 # 35
    # kElevatorDistanceMovedAfterContactWithLimitSwitch = 0.2 poor Jack
    
    kCoralLv3 = 15.75 #14.25 #was 16, why different # 11.2
    kAlgaeLv3 = 25
    kCoralLv4 = 36 # 25.5 # All the elevator levels below aren't tuned
    # kCoralLv4_JumpScore = 37

    # Goes up of down by this much every 50th of a second
    kElevatorIncrementalStep = 0.1
    
    # The command decides the position's close enough if it's within this range (in rotations of a sort)
    # This doesn't delete the setpoint, it just declared the command's finished
    kTargetValueAccuracy = 0.65
    kTargetValueAdder = 0.35 # If it setpoints to a value a bit more or less than you want to to

    kVVoltSecondPerMeter = 0 # 1.5
    kAVoltSecondSquaredPerMeter = 0 # 0.75

    kElevatorOffsetMeters = 0 #Used in softlimit minimum

    kBottomLeftLimitSwitchChannel = Rio_DIO.EIGHT
    kBottomRightLimitSwitchChannel = Rio_DIO.NINE
    
    kTopLimitSwitchChannel = Rio_DIO.SEVEN  # TODO: ? two on top also?

class Override_DriveConstant:
    kSlowMove = 0.3 # Percent speed for slow mode movement
    kSlowRotate = 0.3 # Percent speed for slow mode rotation

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
    kPivotp = 1.3
    kPivoti = 0
    kPivotd = 0
    kPivotg = 0.3
    
    # If the motor is stalled then it's trying to intake an algae more than it can
    # So this detects if it shouldn't spin anymore
    kAmpValueToDetectIfMotorStalled = 90
    
    # The command decides the position's close enough if it's within this range (in rotations)
    kTargetValueAccuracy = 0.1
    kTargetValueAdder = 0 # If it setpoints to a value a bit more or less than you want to to
    
    kPivotMaxHeight = 0.25
    kPivotMinHeight = 0
    
    # Values to set pivoting motor to
    kPivotReefIntakingValue = 2.75 # Pivot position when grabbing algae
    kPivotGroundIntakingValue = 2 # Pivot position when grabbing algae from the FLOOR (not being used)
    kPivotAfterGroundIntakingValue = 1.7
    kPivotProcessingValue = 2 # Pivot position when about to send to processor
    
    # The time it takes to switch between pivoting positions
    kPivotRotationsPerSecond = 2
    kPivotHomingRate = -0.05
    
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
    kDischargeMultiplier = 0.30
    
    kFlipperPulseDuration = 1
    
    # Not currently being used
    kDelayBetweenLeavingBeamBreaksAndActivatingFlippers = 0.3
    
    kHighEnoughToActivateFlippers = 0.8 # I think it's in meters (where 0 is lowest physical point on elevator)
    

class ClimbConstants:
    kClimbMotorPort = CAN_Address.TWENTYSEVEN

    kCaptureCageSwitchChannel = Rio_DIO.TWO

    kSpeedForwardIn = 0.30
    kSpeedForwardIn_gripped = 0.99
    kSpeedBackOut = -0.20
    

    # PID stuff for coral:
    kMotorKp = 1
    kMotorKi = 0
    kMotorKd = 0

class PneumaticConstants:
    kHub = CAN_Address.FORTY

    kLeftScoreSolenoid = 12
    kLeftRetractSolenoid = 13
    kRightScoreSolenoid = 14
    kRightRetractSolenoid = 15
    
    # For getting delay between coral leaving beam breaks and activatings
    kScoreDelay = 0.2

class PowerDistributionConstants:
    kPDP = CAN_Address.FIFTY

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