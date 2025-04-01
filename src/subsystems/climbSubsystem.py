import math

import phoenix6
import wpilib
from phoenix6.hardware import TalonFX
import wpilib
from wpilib import Timer, SmartDashboard
from constants import ClimbConstants, CoralConstants

from constants import ClimbConstants
from commands2 import Subsystem

class ClimbSubsystem(Subsystem):
    ''' Subsystem that handles the climb mechanic '''

    def __init__(self):
        self.climbConstants = ClimbConstants()
        self.climbMotor = TalonFX(self.climbConstants.kClimbMotorPort, "rio")
        self.isReadyToGrabOntoCage = False
        self.cageInGripSwitch = wpilib.DigitalInput(self.climbConstants.kCaptureCageSwitchChannel)
        # self.bottomLimitSwitch = wpilib.DigitalInput(self.climbConstants.kBottomLimitSwitchChannel)
        # self.topLimitSwitch = wpilib.DigitalInput(self.climbConstants.kTopLimitSwitchChannel)
        cfgs = phoenix6.configs.TalonFXConfiguration()
        cfgs.current_limits.with_stator_current_limit_enable(False).with_supply_current_limit_enable(False)


        cfgs.slot0.k_p = ClimbConstants.kMotorKp
        cfgs.slot0.k_i = ClimbConstants.kMotorKi
        cfgs.slot0.k_d = ClimbConstants.kMotorKd

        hardwareLimitSwitchConfig = (phoenix6.configs.config_groups.HardwareLimitSwitchConfigs()
                        .with_forward_limit_enable(True)
                        .with_forward_limit_type(phoenix6.configs.config_groups.ForwardLimitTypeValue.NORMALLY_OPEN)
                        .with_reverse_limit_enable(True)
                        .with_reverse_limit_type(phoenix6.configs.config_groups.ForwardLimitTypeValue.NORMALLY_OPEN))
        
        motorControllerFeedbackConfig = (phoenix6.configs.config_groups.FeedbackConfigs()
                           .with_sensor_to_mechanism_ratio((1/(4*5*3)) * math.pi * 2.375))
        

        cfgs.motor_output.with_inverted(phoenix6.configs.config_groups.InvertedValue.CLOCKWISE_POSITIVE).with_neutral_mode(phoenix6.configs.config_groups.NeutralModeValue.BRAKE)
        cfgs.with_hardware_limit_switch(hardwareLimitSwitchConfig)
        cfgs.with_feedback(motorControllerFeedbackConfig)

        
        status = phoenix6.StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.climbMotor.configurator.apply(cfgs)
            if status.is_ok():
                break
        
        # SmartDashboard.putString("Climb/Deploy Limit Switch", self.climbMotor.get_reverse_limit().value)
        # SmartDashboard.putString("Climb/Climb Limit Switch", self.climbMotor.get_forward_limit().value)
        

    def periodic(self):
        # Right now this is not working right now
        # if not self.cageInGripSwitch.get():
        #     SmartDashboard.putBoolean("READY TO CLIMB", True)
        # else:
        #     SmartDashboard.putBoolean("READY TO CLIMB", False)

        # For testing to see if periodic even runs
        SmartDashboard.putBoolean("READY TO CLIMB", True)

        # SmartDashboard.putString("Climb/Deploy Limit Switch", self.climbMotor.get_reverse_limit().value)
        # SmartDashboard.putString("Climb/Climb Limit Switch", self.climbMotor.get_forward_limit().value)
        ...

    def forward(self):
        ''' Reel in the robot claw '''
        if not self.cageInGripSwitch.get():
             #The is triggered bc normally open
             self.climbMotor.set(self.climbConstants.kSpeedForwardIn_gripped)
        else:
            self.climbMotor.set(self.climbConstants.kSpeedForwardIn)

    def backward(self):
        ''' Reel out the robot claw '''
        self.climbMotor.set(self.climbConstants.kSpeedBackOut)

    def stop(self):
        '''Stop the reel'''
        self.climbMotor.set(0)
