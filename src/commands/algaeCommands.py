'''Has a command for changing the position of the pivot motor and the speed of the intake motor'''

from wpilib import Timer, SmartDashboard
from commands2 import Command, InterruptionBehavior
from subsystems import elevatorSubsystem, algaeSubsystem
from constants import AlgaeConstants
    
class AlgaePivotCommand(Command):
    '''Sets algae pivot motor to any position in rotations'''
    def __init__(self, position, algaeSubsystem: algaeSubsystem.AlgaeSubsystem):
        self.algaeSubsystem = algaeSubsystem
        self.addRequirements(self.algaeSubsystem)
        self.InterruptionBehavior = InterruptionBehavior.kCancelSelf
        self.position = position
        self.timer = Timer()
        self.timer.start()
    
    def initialize(self):
        pass

    def execute(self):
        self.algaeSubsystem.updatePivotSetpoint(self.position)
        self.algaeSubsystem.changePivotPosition()
        
    def isFinished(self):
        return self.timer.get() > 1
        # value = self.algaeSubsystem.pivotMotor.get_position().value
        # if self.position == AlgaeConstants.kPivotIdleValue:
        #     return True # Automatically finish the command if it's being told to set to idle value
        # else:
        #     return (value <= self.position + AlgaeConstants.kTargetValueAccuracy
        #         and value >= self.position - AlgaeConstants.kTargetValueAccuracy)
        
    def end(self): pass

class AlgaeIntakeCommand(Command):
    '''Super simple command that sets intake motors and that's it'''
    def __init__(self, algaeSubsystem: algaeSubsystem.AlgaeSubsystem, speed):
        # Doesn't add requirements so it can be run at the same time as tha AlgaePivotCommand
        self.algaeSubsystem = algaeSubsystem
        self.speed = speed
        
    def initialize(self):
        self.algaeSubsystem.spinIntakeMotor(self.speed)
        
    def isFinished(self): return True
    
class AlgaeInstantCommand(Command):
    '''Sets pivot position and intake motor speed and ends the command'''
    def __init__(self, algaeSubsystem: algaeSubsystem.AlgaeSubsystem, pivotPosition, intakeSpeed):
        self.algaeSubsystem = algaeSubsystem
        self.position = pivotPosition
        self.speed = intakeSpeed
        self.addRequirements(self.algaeSubsystem)
        self.InterruptionBehavior = InterruptionBehavior.kCancelSelf
    
    def initialize(self): # TODO: WORK ON THIS
        self.algaeSubsystem.updatePivotSetpoint(self.position)
        self.algaeSubsystem.changePivotPosition()
        self.algaeSubsystem.spinIntakeMotor(self.speed)
    
    # def execute(self):
    #     self.algaeSubsystem.updatePivotSetpoint(self.position)
    #     self.algaeSubsystem.changePivotPosition(2)#self.algaeSubsystem.setpoint)
    
    def isFinished(self): return True
    
class AlgaeHomeCommand(Command):
    def __init__(self, algaeSubsystem: algaeSubsystem.AlgaeSubsystem):
        self.algaeSubsystem = algaeSubsystem
        self.addRequirements(self.algaeSubsystem)
        self.InterruptionBehavior = InterruptionBehavior.kCancelSelf
    
    def initialize(self):
        self.algaeSubsystem.spinPivotMotor(-0.15)
        self.algaeSubsystem.spinIntakeMotor(0)
        
    def isFinished(self): 
        if self.algaeSubsystem.getBottomLimitSwitchActive():
            self.algaeSubsystem.pivotMotor.set_position(0)
            self.algaeSubsystem.spinPivotMotor(0)
            return True
    
    def end(self):
        self.algaeSubsystem.spinPivotMotor(0)

class AlgaeLiftArmCommand(Command):
    def __init__(self, algaeSubsystem: algaeSubsystem.AlgaeSubsystem):
        self.algaeSubsystem = algaeSubsystem
        self.addRequirements(self.algaeSubsystem)
        self.InterruptionBehavior = InterruptionBehavior.kCancelIncoming
    
    def initialize(self):
        self.algaeSubsystem.spinPivotMotor(0.075)
        
    def isFinished(self):
        if self.algaeSubsystem.getBottomLimitSwitchActive():
            self.algaeSubsystem.setpoint = 0
            self.algaeSubsystem.spinPivotMotor(0)
            return True

# WE DONT NEED THIS ANYMORE YAY
class AlgaeManualPIDCommand(Command):
    def __init__(self, algaeSubsystem: algaeSubsystem.AlgaeSubsystem, position, p, g):
        self.algaeSubsystem = algaeSubsystem
        self.addRequirements(self.algaeSubsystem)
        self.InterruptionBehavior = InterruptionBehavior.kCancelIncoming
        self.setpoint = position
        self.kp = p
        self.kg = g
        self.timer = Timer()
        self.timer.start()
        
    def execute(self):
        self.algaeSubsystem.spinPivotMotor(
            (self.setpoint - self.algaeSubsystem.pivotMotor.get_position().value)
            * self.kp + self.kg
        )
    
    def isFinished(self):
        return self.timer.get() > 1
        # return (self.setpoint <= self.position + AlgaeConstants.kTargetValueAccuracy
        #         and self.setpoint >= self.position - AlgaeConstants.kTargetValueAccuracy)
    
class AlgaeSetPivotSpeedCommand(Command):
    def __init__(self, algaeSubsystem: algaeSubsystem.AlgaeSubsystem, speed):
        self.algaeSubsystem = algaeSubsystem
        self.addRequirements(self.algaeSubsystem)
        self.speed = speed

    def initialize(self):
        self.algaeSubsystem.spinPivotMotor(self.speed)

    def isFinished(self): return True

'''
MANUAL PID
get current position and setpoint
set speed to: kp * (setpoint - position) + kg
see if this works
'''