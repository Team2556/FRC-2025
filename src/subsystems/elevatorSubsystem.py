import commands2
import wpilib
from wpilib import SmartDashboard
import wpimath.controller
from wpimath.controller import PIDController, ProfiledPIDController
import wpimath.trajectory
from wpimath.units import meters, inches, seconds, metersToInches, inchesToMeters
import phoenix6
from phoenix6 import hardware, controls, configs, StatusCode, signals
from phoenix6.controls import Follower
from phoenix6.signals import NeutralModeValue
from constants import ElevatorConstants
from math import pi


# Create a new ElevatorSubsystem... using phonix6 PID
class ElevatorSubsystem(commands2.Subsystem):# .ProfiledPIDSubsystem):
    def __init__(self) -> None:
        '''IM AN ELEVATOR'''

        super().__init__( )
        self.i = 0

        # Start at position 0, use slot 0
        wpilib.SmartDashboard.putNumber("Elevator/Setpoint", 0.0)
        self.setpoint = wpilib.SmartDashboard.getNumber("Elevator/Setpoint", 0.0)

        # Declare the motors
        self.elevmotor_right = phoenix6.hardware.TalonFX(ElevatorConstants.kRightMotorPort, "rio")
        self.elevmotor_left = phoenix6.hardware.TalonFX(ElevatorConstants.kLeftMotorPort, "rio")
        
        # Make the right motor follow the left (so moving the left one moves the right one in the opposite direction)
        # TODO: Actually make this work it's important we kinda need it
        self.elevmotor_right.set_control(Follower(self.elevmotor_left.device_id, oppose_master_direction=True))
        
        # Make it so when motor speed is set to 0 then it stays at 0 and resists movement against it
        self.elevmotor_right.setNeutralMode(NeutralModeValue.BRAKE)
        self.elevmotor_left.setNeutralMode(NeutralModeValue.BRAKE)
        
        # Declare the encoders (they're not referenced later but still needed?)
        self.elevCANcoder_left = phoenix6.hardware.CANcoder(ElevatorConstants.kLeftMotorPort)
        self.elevCANcoder_right = phoenix6.hardware.CANcoder(ElevatorConstants.kRightMotorPort)
        
        # Declare Limit switches (2 on each direction)
        self.limit_bottomLeft = wpilib.DigitalInput(ElevatorConstants.kBottomLeftLimitSwitchChannel)
        self.limit_bottomRight = wpilib.DigitalInput(ElevatorConstants.kBottomRightLimitSwitchChannel)
        # self.limit_topLeft = wpilib.DigitalInput(ElevatorConstants.kTopLeftLimitSwitchChannel)
        self.limit_topRight = wpilib.DigitalInput(ElevatorConstants.kTopRightLimitSwitchChannel)

        # Setup all the PID stuff
        cfg = configs.TalonFXConfiguration()
        cfg.slot0.k_p = ElevatorConstants.kElevatorKp
        cfg.slot0.k_i = ElevatorConstants.kElevatorKi
        cfg.slot0.k_d = ElevatorConstants.kElevatorKd
        
        # # cfg.slot0.integralZone = 0
        # # cfg.slot0.forwardSoftLimitThreshold = 0

        cfg.slot0.gravity_type = signals.GravityTypeValue.ELEVATOR_STATIC
        cfg.slot0.static_feedforward_sign = signals.StaticFeedforwardSignValue.USE_VELOCITY_SIGN
        cfg.slot0.k_g = ElevatorConstants.kGVolts
        

        cfg.slot0.k_a = ElevatorConstants.kAVoltSecondSquaredPerMeter
        cfg.slot0.k_v = ElevatorConstants.kVVoltSecondPerMeter
        # cfg.slot0.maxIntegralAccumulator = 0
        

        # cfg.voltage.peak_output_forward = 8
        cfg.stator_current_limit_enable = True
        cfg.torque_current.peak_forward_torque_current = ElevatorConstants.kpeak_forward_torque_current
        cfg.torque_current.peak_reverse_torque_current = ElevatorConstants.kpeak_reverse_torque_current
        # Would only work with CAN based (prob CRTE only) sensors as limitswitches
        elevmotorLimitswitch_cfg = (configs.HardwareLimitSwitchConfigs()
                                       .with_forward_limit_enable(True)
                                       .with_forward_limit_autoset_position_enable(True)
                                       .with_forward_limit_autoset_position_value(self.distanceToRotations(ElevatorConstants.kMaxElevatorHeight))
                                       .with_forward_limit_remote_sensor_id(ElevatorConstants.kTopRightLimitSwitchChannel)
                                       .with_forward_limit_type(signals.ForwardLimitTypeValue.NORMALLY_OPEN)
                                       
                                       .with_reverse_limit_autoset_position_enable(True)
                                       .with_reverse_limit_autoset_position_value(self.distanceToRotations(ElevatorConstants.kElevatorOffsetMeters))
                                       .with_reverse_limit_remote_sensor_id(ElevatorConstants.kBottomLeftLimitSwitchChannel)
                                       .with_reverse_limit_type(signals.ReverseLimitTypeValue.NORMALLY_OPEN)
                                       )
        cfg.with_hardware_limit_switch(elevmotorLimitswitch_cfg)

        # cfg.with_current_limits(
        # configs.CurrentLimitsConfigs().with_supply_current_limit()

        elevmotorSoftLimits_cfg = (configs.SoftwareLimitSwitchConfigs()
                                   .with_forward_soft_limit_enable(True)
                                   .with_forward_soft_limit_threshold(self.distanceToRotations(ElevatorConstants.kMaxElevatorHeight))
                                   .with_reverse_soft_limit_enable(True)
                                   .with_reverse_soft_limit_threshold(self.distanceToRotations(ElevatorConstants.kElevatorOffsetMeters))
                                   )
        cfg.with_software_limit_switch(elevmotorSoftLimits_cfg)
        
        elevmotorFeedback_cfg = (configs.FeedbackConfigs().with_feedback_sensor_source(signals.FeedbackSensorSourceValue.ROTOR_SENSOR)
                                 #The functions distance to rotations had this already TODO: which way to go  .with_sensor_to_mechanism_ratio(ElevatorConstants.kElevatorGearing)#/(2*pi*ElevatorConstants.kElevatorDrumRadius))
                                    )
        cfg.with_feedback(elevmotorFeedback_cfg)

        self.cfg_slot0 = cfg.slot0

        # Retry config apply up to 5 times, report if failure
        status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.elevmotor_left.configurator.apply(cfg)
            status = self.elevmotor_right.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

        # Create handle for the control
        # Make sure we start at 0
        #TODO: add to teleop init a homing command, once we have the limit switches
        self.homing_control = (controls.VelocityVoltage(-1 * self.distanceToRotations(ElevatorConstants.kHomingRate))
                               .with_slot(0)
                               .with_limit_reverse_motion(False))#self.getLimitBottom))
        self.elevmotor_left.set_position(0)#self.distanceToRotations(ElevatorConstants.kElevatorOffsetMeters))
        
        self.position_voltage = controls.PositionVoltage(
            0, # Position will be changed when this is actually used
            self.distanceToRotations(ElevatorConstants.kElevatorSpeed),
            limit_forward_motion = False,#self.getLimitTop(),
            limit_reverse_motion = False#self.getLimitBottom()
        ).with_slot(0)
        # self.elevmotor_left.set_control(request=self.position_voltage.with_position(self.distanceToRotations(inchesToMeters(.25)))

    def moveElevator(self, movement=None) -> None:
        '''Setpoint is in meters of elevator elevation from lowest physical limit'''
        if not movement:
            movement = self.setpoint
        self.elevmotor_left.set_control(self.position_voltage.with_position(self.distanceToRotations(movement)))
        # self.elevmotor_right.set_control(self.position_voltage.with_position(self.distanceToRotations(-1 * movement)))
        print(movement)
        
    def updateSlot0(self,  k_p: float = None, k_i:float =None, k_d:float=None, k_g: float=None   ) -> None:
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
            #repete up to 5 times
            status: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
            for _ in range(0, 5):
                status = self.elevmotor_left.configurator.apply(self.cfg_slot0 )
                if status.is_ok():
                    break
            if not status.is_ok():
                print(f"Could not apply updated gravity compensation, error code: {status.name}")

    def distanceToRotations(self, distance: float) -> float:
        return distance * ElevatorConstants.kElevatorGearing/(2*pi*ElevatorConstants.kElevatorDrumRadius)
    def rotationsToDistance(self, rotations: float) -> float:
        return rotations * 2*pi*ElevatorConstants.kElevatorDrumRadius/ElevatorConstants.kElevatorGearing

    def getLimitBottom(self):
        return not self.limit_bottomLeft.get() and not self.limit_bottomRight.get()
    def getLimitTop(self):
        return not self.limit_topRight.get()
    
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
    
    def reset_zero_point(self, let_droop: bool = True) -> None:
        #put the motor in neutral - no breaking
        # if let_droop: self.elevmotor_left.set_control(self.elevmotor_left.setNeutralMode(NeutralModeValue.COAST))
        
        # self.elevmotor_left.set_position(0) #self.elevmotor_left.set_control(lambda: )
        return commands2.cmd.run(lambda: self.elevmotor_left.set_position(0))
        
    def let_elevator_drop(self) -> None:
        # Put the motor in neutral - no breaking
        return commands2.cmd.run(lambda: self.elevmotor_left.setNeutralMode(NeutralModeValue.COAST))
    
    def elevator_motors_break(self):
        return commands2.cmd.run(lambda: self.elevmotor_left.setNeutralMode(NeutralModeValue.BRAKE))

    def homeInToZero(self):
        '''Makes elevator move down'''
        self.elevmotor_left.set_control(self.homing_control)
    
    def get_position(self):
        return self.rotationsToDistance(self.elevmotor_left.get_position().value)
    
    def incrementElevator(self, increment):
        self.elevmotor_left.set(increment)
        self.elevmotor_right.set(-1 * increment)
        # WE WILL GET PID SETPOINTS TO WORK
        
    def periodic(self):
        wpilib.SmartDashboard.putNumber("Elevator/Position_calced", self.rotationsToDistance(self.elevmotor_left.get_position().value))
        wpilib.SmartDashboard.putNumber("Elevator/SetpointError_calced", self.setpoint - self.rotationsToDistance(self.elevmotor_left.get_position().value))
        self.i +=1
        SmartDashboard.putNumber("Counter on elevator periodic", self.i)
        self.updateSlot0(k_p= SmartDashboard.getNumber("Elevator/Kp",0.0),k_i=SmartDashboard.getNumber("Elevator/Ki",0.0),k_d=SmartDashboard.getNumber("Elevator/Kd",0.0),#k_f=SmartDashboard.getNumber("Elevator\Kf",0.0),k_izone=SmartDashboard.getNumber("Elevator\Izone",0.0),k_peak_output=SmartDashboard.getNumber("Elevator\Peak Output",0.0),k_allowable_error=SmartDashboard.getNumber("Elevator\Allowable Error",0.0),k_cruise_velocity=SmartDashboard.getNumber("Elevator\Cruise Velocity",0.0),k_acceleration=SmartDashboard.getNumber("Elevator\Acceleration",0.0),k_g=SmartDashboard.getNumber("Elevator\Gravity Compensation",0.0))
            k_g=SmartDashboard.getNumber("Elevator/Kg",0.0))
        
        if self.getLimitBottom():
            self.position_voltage.limit_forward_motion = True
            self.elevmotor_left.set_position(0) # To zero it

        # print(self.getLimitBottom(), self.getLimitTop())

        print(self.elevmotor_left.get_position().value)