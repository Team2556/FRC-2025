'''Has a command for changing the position of the pivot motor and the speed of the intake motor'''

from commands2 import Command, InterruptionBehavior
from wpilib import Timer

from constants import AlgaeConstants
from subsystems import algaeSubsystem


class AlgaeCommand(Command):
    '''THE algae command that sets a pivot and rotation value'''
    def __init__(self, algaeSubsystem: algaeSubsystem.AlgaeSubsystem, position, intakeSpeed):
        self.algaeSubsystem = algaeSubsystem
        self.addRequirements(self.algaeSubsystem)
        self.InterruptionBehavior = InterruptionBehavior.kCancelSelf
        
        self.position = position
        self.intakeSpeed = intakeSpeed
    
    def initialize(self):
        self.algaeSubsystem.spinIntakeMotor(self.intakeSpeed)
        self.algaeSubsystem.updatePivotSetpoint(self.position)
        self.algaeSubsystem.changePivotPosition()
        
    def isFinished(self):
        value = self.algaeSubsystem.getPivotPosition()
        return (value <= self.position + AlgaeConstants.kTargetValueAdder + AlgaeConstants.kTargetValueAccuracy
            and value >= self.position + AlgaeConstants.kTargetValueAdder - AlgaeConstants.kTargetValueAccuracy)
        
    def end(self): 
        print("Algae Command Finished!") # NOTE ITS TEMPORARY
        pass

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