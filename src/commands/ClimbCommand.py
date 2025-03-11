from abc import abstractmethod
import commands2
from subsystems.ClimbSubsystem import ClimbSubsystem
from constants import ClimbConstants

class ClimbCommand(commands2.Command):
    """
    Command to control the climbing mechanism using button inputs.
    """

    def __init__(self, climb: ClimbSubsystem, 
                 #Armvert: Callable[[], bool], Armdown: Callable[[], bool], 
                 state: ClimbSubsystem.SubsystemState): #joystick : XboxController):
        super().__init__()
        self.climb = climb
        # self.deploy = deploy
        # self.down = climbdown
        #self.Armvert = Armvert
        #self.Armdown = Armdown
        #self.isFullPower = isFullPower
        #self.joystick = joystick
        self.new_state = state
        self.addRequirements(climb)

    def initialize(self, mode):
        """Called when the command is initially scheduled."""
        self.state = ClimbSubsystem.SubsystemState.INITIAL

    @abstractmethod
    def execute(self):
        """Called every time the scheduler runs while the command is scheduled."""
        # self.speed = ClimbConstants.kSpeed if self.isFullPower() else 0.75 * ClimbConstants.kSpeed

        # # Manually controlling climb arm height
        # if self.deploy():
        #     self.climb.climbDeploy(self.isFullPower)
        # elif self.down() and not self.climb.isReset():
        #     self.climb.climbPull(-self.speed)
        # else:
        #     self.climb.climbRetract(0)

        raise NotImplementedError()
    
        
    @abstractmethod
    def isFinished(self) -> bool:
        """Returns true when the command should end."""
        raise NotImplementedError()

    def end(self, interrupted: bool):
        """Called once the command ends or is interrupted."""
        self.climb.climbDeploy(0)

        if interrupted:
            self.state = ClimbSubsystem.SubsystemState.UNKNOWN
        else:
            self.state = self.new_state
        
class ReadyClimber(ClimbCommand):
    position = ClimbConstants.kClimbMinHeight

    def __init__(self, climber: ClimbSubsystem):
        super().__init__(climber=climber, state=ClimbSubsystem.SubsystemState.INITIAL)

    def execute(self):
        self.climb.climbDeploy()

    def isFinished(self) -> bool:
        return self.climb.getPosition() >= self.position


class Climb(ClimbCommand):
    def __init__(self, climber: ClimbSubsystem):
        super().__init__(climber=climber, state=ClimbSubsystem.SubsystemState.CLIMB_POSITIVE)

    def execute(self):
        self.climb.climbPull()

    def isFinished(self) -> bool:
        return self.climb.isClimbed()


class ReleaseClimber(ClimbCommand):
    def __init__(self, climber: ClimbSubsystem):
        super().__init__(climber=climber, state=ClimbSubsystem.SubsystemState.CLIMB_NEGATIVE)

    def execute(self):
        self.climb.climbRetract()

    def isFinished(self) -> bool:
        return self.climb.getPosition() <= 0.0
    
class ResetClimber(ClimbCommand):
    def __init__(self, climber: ClimbSubsystem):
        super().__init__(climber = climber, state = ClimbSubsystem.SubsystemState.RESET)
        self.touched_switch = False

    def initialize(self):
        self.touched_switch = False
        self.climb.SubsystemState = self.climb.SubsystemState.CLIMB_NEGATIVE

    def execute(self):
        if self.climb.isClimbed():  # If the switch is pressed move up.
            self.climb.climbDeploy()
            self.touched_switch = True
        elif self.touched_switch:
            self.climb.climbRetract()
        else:
            self.climb.climbPull()

    def isFinished(self) -> bool:
        return self.touched_switch and self.climb.getPosition() <= 0.0

    def end(self, interrupted: bool):
        if interrupted:
            self.climb.SubsystemState = self.climb.SubsystemState.UNKNOWN
        else:
            self.climb.SubsystemState = self.climb.SubsystemState.INITIAL
        self.climb.stop()
       
