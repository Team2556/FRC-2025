
import commands2
import phoenix6
from phoenix6 import controls, configs, signals
from phoenix6.controls import Follower
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from wpilib import SmartDashboard, DigitalInput

from constants import ElevatorConstants


# Create a new ElevatorSubsystem... using phonix6 PID
class ElevatorSubsystem(commands2.Subsystem):# .ProfiledPIDSubsystem):
    def __init__(self) -> None:
        '''IM AN ELEVATOR'''

        # Start at position 0
        self.setpoint = 0

        # Declare the motors
        self.elevmotor_right = TalonFX(ElevatorConstants.kRightMotorPort, "rio")
        self.elevmotor_left = TalonFX(ElevatorConstants.kLeftMotorPort, "rio")
        
        # Make the right motor follow the left (so moving the left one moves the right one in the opposite direction)
        # TODO: Actually make this work it's important we kinda need it
        self.elevmotor_right.set_control(Follower(
                self.elevmotor_left.device_id,                                 
                motor_alignment = signals.spn_enums.MotorAlignmentValue.OPPOSED
            ))

        self.elevmotor_right.setNeutralMode(NeutralModeValue.BRAKE)
        self.elevmotor_left.setNeutralMode(NeutralModeValue.BRAKE)
        
        # Declare Limit switches (2 on each direction)
        self.limit_bottomLeft = DigitalInput(ElevatorConstants.kBottomLeftLimitSwitchChannel)
        self.limit_bottomRight = DigitalInput(ElevatorConstants.kBottomRightLimitSwitchChannel)
        self.limit_top = DigitalInput(ElevatorConstants.kTopLimitSwitchChannel)

        # Setup all the PID stuff
        cfg = configs.TalonFXConfiguration()
        cfg.slot0.k_p = ElevatorConstants.kElevatorKp
        cfg.slot0.k_i = ElevatorConstants.kElevatorKi
        cfg.slot0.k_d = ElevatorConstants.kElevatorKd
        cfg.slot0.k_g = ElevatorConstants.kElevatorKg

        cfg.slot0.gravity_type = signals.GravityTypeValue.ELEVATOR_STATIC
        cfg.slot0.static_feedforward_sign = signals.StaticFeedforwardSignValue.USE_VELOCITY_SIGN

        cfg.torque_current.peak_forward_torque_current = ElevatorConstants.kpeak_forward_torque_current
        cfg.torque_current.peak_reverse_torque_current = ElevatorConstants.kpeak_reverse_torque_current

        elevmotorLimitswitch_cfg = (configs.HardwareLimitSwitchConfigs()
                                       .with_forward_limit_enable(True)
                                       .with_forward_limit_autoset_position_enable(False)
                                       # .with_forward_limit_autoset_position_value(ElevatorConstants.kMaxElevatorHeight)
                                       .with_forward_limit_remote_sensor_id(ElevatorConstants.kTopLimitSwitchChannel)
                                       .with_forward_limit_type(signals.ForwardLimitTypeValue.NORMALLY_OPEN)
                                       .with_reverse_limit_enable(True)
                                       .with_reverse_limit_autoset_position_enable(True)
                                       .with_reverse_limit_autoset_position_value(ElevatorConstants.kMinElevatorHeight)
                                       .with_reverse_limit_remote_sensor_id(ElevatorConstants.kBottomLeftLimitSwitchChannel)
                                       .with_reverse_limit_type(signals.ReverseLimitTypeValue.NORMALLY_OPEN)
                                       )
        cfg.with_hardware_limit_switch(elevmotorLimitswitch_cfg)


        elevmotorSoftLimits_cfg = (configs.SoftwareLimitSwitchConfigs()
                                   .with_forward_soft_limit_enable(True)
                                   .with_forward_soft_limit_threshold(ElevatorConstants.kMaxElevatorHeight)
                                   .with_reverse_soft_limit_enable(True)
                                   .with_reverse_soft_limit_threshold(ElevatorConstants.kMinElevatorHeight)
                                   )
        cfg.with_software_limit_switch(elevmotorSoftLimits_cfg)
        
        elevmotorFeedback_cfg = (configs.FeedbackConfigs().with_feedback_sensor_source(signals.FeedbackSensorSourceValue.ROTOR_SENSOR)
                                 #The functions distance to rotations had this already TODO: which way to go  
                                 .with_sensor_to_mechanism_ratio(ElevatorConstants.kElevatorSensorToMech)
                                    )
        cfg.with_feedback(elevmotorFeedback_cfg)
        
        cfg.motor_output.with_inverted(phoenix6.configs.config_groups.InvertedValue.CLOCKWISE_POSITIVE)
        cfg.motor_output.with_neutral_mode(phoenix6.configs.config_groups.NeutralModeValue.BRAKE)
        
        self.cfg_slot0 = cfg.slot0

        for _ in range(0, 5):
            status = self.elevmotor_left.configurator.apply(cfg)
            if status.is_ok():
                break
        for _ in range(0, 5):
            status = self.elevmotor_right.configurator.apply(cfg)
            if status.is_ok():
                break
            
        # TODO test this
        self.home_voltage = controls.DutyCycleOut(
            0, # Output will be changed when this is actually used
            # limit_forward_motion = False, # These are changed on the bottom in periodic 
            # limit_reverse_motion = False
        )
        
        self.position_voltage = controls.PositionVoltage(
            0, # Position will be changed when this is actually used
            ElevatorConstants.kElevatorSpeed,
            # limit_forward_motion = False, # These are changed on the bottom in periodic 
            # limit_reverse_motion = False
        ).with_slot(0)
        
        self.setupSmartDashboard()
        
        self.elevmotor_left.set_position(0)
        
        self.isHoming = False

    def moveElevator(self) -> None:
        """Setpoint is in meters of elevator elevation from the lowest physical limit"""
        self.elevmotor_left.set_control(self.position_voltage.with_position(self.setpoint)
                                        .with_velocity(ElevatorConstants.kElevatorSpeed))
        self.isHoming = False

    def setElevatorSpeed(self, increment):
        # Manual moving the elevator
        self.elevmotor_left.set_control(self.home_voltage.with_output(increment))
        self.isHoming = True
        
    def updateSlot0(self, k_p: float = None, k_i: float = None, k_d: float = None, k_g: float = None) -> None:
        """Don't update it too much because it causes problems so it only does it when it needs to"""
        updated = False
        if self.cfg_slot0.k_p != k_p: 
            self.cfg_slot0.k_p = k_p
            updated = True
        if self.cfg_slot0.k_i != k_i: 
            self.cfg_slot0.k_i = k_i
            updated = True
        if self.cfg_slot0.k_d != k_d: 
            self.cfg_slot0.k_d = k_d
            updated = True
        if self.cfg_slot0.k_g != k_g: 
            self.cfg_slot0.k_g = k_g
            updated = True
        # TODO: add others if needed
        if updated:
            for _ in range(0, 5):
                status = self.elevmotor_left.configurator.apply(self.cfg_slot0 )
                if status.is_ok():
                    break

    def getLimitBottom(self): 
        # Changed from "or" to "and" because one of these accidentally setting off will be a big problem
        return not self.limit_bottomLeft.get() and not self.limit_bottomRight.get()
    
    def getLimitTop(self):
        return not self.limit_top.get()
    
    def update_setpoint(self, setpoint: float, incremental = False, constrain: bool = True) -> None:
        '''Setpoint is in meters of elevator elevation from lowest physical limit'''
        if incremental:
            self.setpoint += setpoint
        else:
            self.setpoint = setpoint
        #within bounds check and reset to bounds:
        if constrain:
            if self.setpoint > ElevatorConstants.kMaxElevatorHeight:
                self.setpoint = ElevatorConstants.kMaxElevatorHeight
            elif self.setpoint < ElevatorConstants.kMinElevatorHeight:
                self.setpoint = ElevatorConstants.kMinElevatorHeight
    
    def get_position(self):
        return self.elevmotor_left.get_position().value
    
        
    def setupSmartDashboard(self):
        SmartDashboard.putNumber("Elevator/Kp", ElevatorConstants.kElevatorKp)
        SmartDashboard.putNumber("Elevator/Ki", ElevatorConstants.kElevatorKi)
        SmartDashboard.putNumber("Elevator/Kd", ElevatorConstants.kElevatorKd)
        SmartDashboard.putNumber("Elevator/Kg", ElevatorConstants.kElevatorKg)
        
        SmartDashboard.putNumber("Elevator/Velocity", ElevatorConstants.kElevatorSpeed)
        SmartDashboard.putNumber("Elevator/Homing Speed", ElevatorConstants.kHomingRate)
        
        SmartDashboard.putNumber("Elevator/Target Value Accuracy", ElevatorConstants.kTargetValueAccuracy)
        SmartDashboard.putNumber("Elevator/Target Value Adder", ElevatorConstants.kTargetValueAdder)
        
        SmartDashboard.putNumber("Elevator/Coral L3", ElevatorConstants.kCoralLv3)
        SmartDashboard.putNumber("Elevator/Algae L3", ElevatorConstants.kAlgaeLv3)
        SmartDashboard.putNumber("Elevator/Coral L4", ElevatorConstants.kCoralLv4)
        
        SmartDashboard.putNumber("Elevator/Height To Slow Down", ElevatorConstants.kLowEnoughToSlowDown)
        SmartDashboard.putNumber("Elevator/Slow Down Multiplier", ElevatorConstants.kLowEnoughSpeedMultiplier)
        
    def updateSmartDashboard(self):
        # Position Stuff
        SmartDashboard.putNumber("Elevator/Setpoint", self.setpoint)
        SmartDashboard.putNumber("Elevator/Position", self.elevmotor_left.get_position().value)
        SmartDashboard.putNumber("Elevator/SetpointError", self.setpoint - self.elevmotor_left.get_position().value)
        
        SmartDashboard.putBoolean("Elevator/Top Limit Switch", self.getLimitTop())
        SmartDashboard.putBoolean("Elevator/Bottom Limit Switch", self.getLimitBottom())
        
        # PID Stuff
        self.updateSlot0(
            k_p = SmartDashboard.getNumber("Elevator/Kp",0.0),
            k_i = SmartDashboard.getNumber("Elevator/Ki",0.0),
            k_d = SmartDashboard.getNumber("Elevator/Kd",0.0),
            k_g = SmartDashboard.getNumber("Elevator/Kg",0.0)
        )
        
        # Temporary update PID
        # SmartDashboard.putNumber("Elevator/Kp", self.cfg_slot0.k_p)
        # SmartDashboard.putNumber("Elevator/Ki", self.cfg_slot0.k_i)
        # SmartDashboard.putNumber("Elevator/Kd", self.cfg_slot0.k_d)
        # SmartDashboard.putNumber("Elevator/Kg", self.cfg_slot0.k_g)
        
        ElevatorConstants.kTargetValueAccuracy = SmartDashboard.getNumber("Elevator/Target Value Accuracy", ElevatorConstants.kTargetValueAccuracy)
        ElevatorConstants.kTargetValueAdder = SmartDashboard.getNumber("Elevator/Target Value Adder", ElevatorConstants.kTargetValueAdder)
        
        # Setpoint Stuff
        ElevatorConstants.kCoralLv3 = SmartDashboard.getNumber("Elevator/Coral L3", ElevatorConstants.kCoralLv3)
        ElevatorConstants.kAlgaeLv3 = SmartDashboard.getNumber("Elevator/Algae L3", ElevatorConstants.kAlgaeLv3)
        ElevatorConstants.kCoralLv4 = SmartDashboard.getNumber("Elevator/Coral L4", ElevatorConstants.kCoralLv4)   
        
        self.position_voltage.velocity = SmartDashboard.getNumber("Elevator/Velocity", ElevatorConstants.kElevatorSpeed)
        ElevatorConstants.kHomingRate = SmartDashboard.getNumber("Elevator/Homing Speed", ElevatorConstants.kHomingRate)
        
        ElevatorConstants.kLowEnoughToSlowDown = SmartDashboard.getNumber("Elevator/Height To Slow Down", ElevatorConstants.kLowEnoughToSlowDown)
        ElevatorConstants.kLowEnoughSpeedMultiplier = SmartDashboard.getNumber("Elevator/Slow Down Multiplier", ElevatorConstants.kLowEnoughSpeedMultiplier)
        
    def periodic(self):
        self.updateSmartDashboard()
        
        # Can do this all in motor config maybe TODO: update & remove yay
        if self.getLimitBottom() and self.isHoming:
            # self.position_voltage.limit_forward_motion = True
            self.elevmotor_left.set_position(0) # To zero it
        # else: 
        #     self.position_voltage.limit_forward_motion = False
            
        # if self.getLimitTop():
        #     self.position_voltage.limit_reverse_motion = True
        # else: 
        #     self.position_voltage.limit_reverse_motion = False