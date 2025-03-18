from phoenix6.hardware import TalonFX
import wpilib
from constants import ClimbConstants

class ClimbSubsystem():
    ''' Subsystem that handles the climb mechanic '''

    def __init__(self):
        self.climbConstants = ClimbConstants()
        self.climbMotor = TalonFX(self.climbConstants.kClimbMotorPort, "rio")
        self.isReadyToGrabOntoCage = False
        self.bottomLimitSwitch = wpilib.DigitalInput(self.climbConstants.kBottomLimitSwitchChannel)
        self.topLimitSwitch = wpilib.DigitalInput(self.climbConstants.kTopLimitSwitchChannel)

    def periodic(self):
        ...

    def reelRobot(self):
        if self.topLimitSwitch.get():
            self.climbMotor.set(self.climbConstants.kSpeed)
        else:
            self.climbMotor.set(0)

    def unreelRobot(self):
        if self.bottomLimitSwitch.get():
            self.climbMotor.set(-self.climbConstants.kSpeed)
        else:
            self.climbMotor.set(0)
