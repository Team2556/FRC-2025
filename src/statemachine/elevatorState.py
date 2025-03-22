from statemachine import StateMachine, State
from subsystems import coralSubsystem, elevatorSubsystem, pneumaticSubsystem
from constants import CoralConstants, PneumaticConstants, ElevatorConstants
from commands import coralCommands, pneumaticCommands, elevatorCommands

# Define the states:

# Coral States:
# FLaps down Coral Track Centering
# Flaps up Coral Track Centering
# Flaps down Coral Track Score Left
# Flaps down Coral Track Score Right
# Flaps move durring (with delay) score Right
# Flaps move durring (with delay)score Left
# Flaps down no centering

class CoralStateMachine(StateMachine):
    def __init__(self, coralSubsystem: coralSubsystem.CoralTrack, elevatorSubsystem: elevatorSubsystem.ElevatorSubsystem):
        self.coralSubsystem = coralSubsystem
        self.elevatorSubsystem = elevatorSubsystem
        self.coralFiring = False
        self.coralStateMachine = StateMachine()
        # Create list of our states
        self.coralStateList = [ 'CoralCentering', 'CoralScoreLeft', 'CoralScoreRight']#'CoralDefault', ,  'CoralNoCentering', 'CoralPneumatics', 'DischargeCoral', 'PulseFlippers']
        CoralCentering = State(initial=True)
        # CoralDeafult = State(intial=True)
        CoralScoreLeft = State()
        CoralScoreRight = State()
        # CoralNoCentering = State()
        # CoralPneumatics = State()
        transitions =  (CoralCentering.to('CoralScoreLeft',) | 
                        CoralCentering.to('CoralScoreRight',) | 
                        CoralScoreLeft.to('CoralCentering',) | 
                        CoralScoreRight.to('CoralCentering',) )
    
    
        


