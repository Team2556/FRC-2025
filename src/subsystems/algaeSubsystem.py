# Thing that controls the algae

from wpilib import SmartDashboard, DigitalInput
from constants import AlgaeConstants
from commands2.subsystem import Subsystem
import phoenix6

class AlgaeSubsystem(Subsystem):
    '''This thing does algae intake and discharge.'''
    
    def __init__(self):
        # Declare motor controllers
        self.pivotMotor = phoenix6.hardware.TalonFX(AlgaeConstants.kPivotMotorChannel, "rio")
        self.intakeMotor = phoenix6.hardware.TalonFX(AlgaeConstants.kIntakeWheelsChannel, "rio")
        # Declare limit switch
        self.bottomLimitSwitch = DigitalInput(AlgaeConstants.kBottomLimitSwitchChannel)
        self.topLimitSwitch = DigitalInput(AlgaeConstants.kTopLimitSwitchChannel)

        # self.otherLimitSwitch = DigitalInput(AlgaeConstants.kOtherLimitSwitchChannel) 
        # There might be another limit switch maybe
        
        # Make it so if motor is set at 0 then it resists any turning if force is applied to it
        # So algae doesn't fall out and stuff
        self.pivotMotor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        self.intakeMotor.setNeutralMode(phoenix6.signals.NeutralModeValue.BRAKE)
        
        self.setupSmartDashboard()
        
        # Creates a "CTRE Control Request Object" for making it spin
        self.velocityOut = phoenix6.controls.VoltageOut(output=0)
        
        # All the following config stuff so setting position PID still works
        cfg = phoenix6.configs.TalonFXConfiguration()
        # TODO: make these constants
        cfg.slot0.k_p = AlgaeConstants.kPivotp
        cfg.slot0.k_i = AlgaeConstants.kPivoti
        cfg.slot0.k_d = AlgaeConstants.kPivotd
        cfg.slot0.k_g = AlgaeConstants.kPivotg
        self.setpoint=0
        
        # Don't think these do anything at all
        # cfg.slot0.integralZone = 0
        # cfg.slot0.forwardSoftLimitThreshold = 0

        # TODO: adding this stuff might make it more accurate/faster/better/but it takes more time so maybe
        cfg.slot0.gravity_type = phoenix6.signals.GravityTypeValue.ARM_COSINE
        # cfg.slot0.static_feedforward_sign = phoenix6.signals.StaticFeedforwardSignValue.USE_VELOCITY_SIGN
        # cfg.slot0.k_g = ElevatorConstants.kGVolts
        
        # cfg.slot0.k_a = ElevatorConstants.kAVoltSecondSquaredPerMeter
        # cfg.slot0.k_v = ElevatorConstants.kVVoltSecondPerMeter
        # cfg.slot0.maxIntegralAccumulator = 0
        
        # # cfg.voltage.peak_output_forward = 8
        # cfg.stator_current_limit_enable = True
        
        # Limits something on the motor so it's useful
        cfg.torque_current.peak_forward_torque_current = AlgaeConstants.kPeakForwardTorqueCurrent
        cfg.torque_current.peak_reverse_torque_current = AlgaeConstants.kPeakReverseTorqueCurrent
        
        # Actually give the motor all the config stuff
        self.PIDconfig = cfg
        
        # Test five times because it might not work some of the time
        status: phoenix6.StatusCode = phoenix6.StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = self.pivotMotor.configurator.apply(self.PIDconfig)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")
        
        # Creates another "CTRE Control Request Object" for making it position
        self.positionVoltage = phoenix6.controls.PositionVoltage(
            0, # Position will be changed when this is actually used
            velocity=AlgaeConstants.kPivotRotationsPerSecond,
            limit_forward_motion=False,#self.getBottomLimitSwitchActive(), # Changed from True so motors actually turn
            limit_reverse_motion=False#self.getTopLimitSwitchActive(),
            # ignore_hardware_limits=True
        ).with_slot(0)#._.
        
    # I might need this for PID
    def updatePIDvalues(self, k_p: float = None, k_i: float = None, k_d : float = None, k_g: float = None) -> None:
        valueUpdated = False
        if self.PIDconfig.slot0.k_p != k_p: 
            self.PIDconfig.slot0.k_p = k_p
            valueUpdated = True
        if self.PIDconfig.slot0.k_i != k_i:  
            self.PIDconfig.slot0.k_i = k_i
            valueUpdated = True
        if self.PIDconfig.slot0.k_d != k_d: 
            self.PIDconfig.slot0.k_d = k_d
            valueUpdated = True
        if self.PIDconfig.slot0.k_g != k_g: 
            self.PIDconfig.slot0.k_g = k_g
            valueUpdated = True
        # TODO: Add other values here if needed (like k_f or something)
        if valueUpdated:
            # Repeat up to 5 times because it might not work some of the time
            status: phoenix6.StatusCode = phoenix6.StatusCode.STATUS_CODE_NOT_INITIALIZED
            for _ in range(0, 5):
                status = self.pivotMotor.configurator.apply(self.PIDconfig)
                if status.is_ok():
                    break
            if not status.is_ok(): 
                print(f"Could not apply updated gravity compensation, error code: {status.name}")
    
    def updatePivotSetpoint(self, setpoint: float, increment = False, constrain: bool = False) -> None:
        '''Setpoint is in meters of elevator elevation from lowest physical limit'''
        if increment:
            self.setpoint += setpoint
        else:
            self.setpoint = setpoint
        # Another restrain in case it's told to go past it's bounds
        if constrain:
            if self.setpoint > AlgaeConstants.kPivotMaxHeight:
                self.setpoint = AlgaeConstants.kPivotMaxHeight
            elif self.setpoint < AlgaeConstants.kPivotMinHeight:
                self.setpoint = AlgaeConstants.kPivotMinHeight
    
    # Motor spinning functions
    def changePivotPosition(self, setpoint: float = None) -> None:
        '''Sets the position of the pivot motor (obvoiusly)'''
        # TODO: Actually test this
        self.pivotMotor.set_control(self.positionVoltage.with_position(2)) 
        # self.pivotMotor.set_position()
    
    def spinIntakeMotor(self, speed) -> None:
        '''Spins the intake motor (speed=1 is intake, speed=-1 is discharge hopefully)'''
        # TODO: Figure out the correct way to do this
        voltage = 12
        self.intakeMotor.set_control(self.velocityOut.with_output(speed * voltage))
        # self.intakeMotor.setVoltage(speed * voltage)
        # self.intakeMotor.set(speed)
        
    def getCurrent(self) -> int:
        '''Gets the current of the intaking motor. 
            Useful for stopping the motor if it's trying to intake an algae further than it can'''  
        # TODO: Figure out if this is actually the correct function to use
        return self.intakeMotor.get_supply_current().value
    
    def getBottomLimitSwitchActive(self) -> bool:
        '''Returns true if bottom limit switch is active'''
        return not self.bottomLimitSwitch.get()

    def getTopLimitSwitchActive(self) -> bool:
        '''Returns true if top limit switch is active'''
        return not self.topLimitSwitch.get()
    
    def spinPivotMotor(self, speed) -> None:
        voltage = 12
        self.pivotMotor.set(speed)
        
    # TODO: Add a cool list/grid layout with the new values to ShuffleBoard
    def setupSmartDashboard(self):
        '''Sets up all the SmartDashboard (tuning) valuess'''
        SmartDashboard.putNumber("Algae/Pivot P", AlgaeConstants.kPivotp)
        SmartDashboard.putNumber("Algae/Pivot I", AlgaeConstants.kPivoti)
        SmartDashboard.putNumber("Algae/Pivot D", AlgaeConstants.kPivotd)
        SmartDashboard.putNumber("Algae/Pivot G", AlgaeConstants.kPivotg)
        SmartDashboard.putNumber("Algae/Amp Value to Detect if Stalled", AlgaeConstants.kAmpValueToDetectIfMotorStalled)
        SmartDashboard.putNumber("Algae/Target Value Accuracy", AlgaeConstants.kTargetValueAccuracy)
        SmartDashboard.putNumber("Algae/Pivot Max Height", AlgaeConstants.kPivotMaxHeight)
        SmartDashboard.putNumber("Algae/Pivot Min Height", AlgaeConstants.kPivotMinHeight)
        SmartDashboard.putNumber("Algae/Pivot Reef Intaking Value", AlgaeConstants.kPivotReefIntakingValue)
        SmartDashboard.putNumber("Algae/Pivot Ground Intaking Value", AlgaeConstants.kPivotGroundIntakingValue)
        SmartDashboard.putNumber("Algae/Pivot Processing Value", AlgaeConstants.kPivotProcessingValue)
        SmartDashboard.putNumber("Algae/Pivot Idle Value", AlgaeConstants.kPivotIdleValue)
        SmartDashboard.putNumber("Algae/Pivot Rotations Per Second", AlgaeConstants.kPivotRotationsPerSecond)
        SmartDashboard.putNumber("Algae/Intake Multiplier", AlgaeConstants.kIntakeMultiplier)
        SmartDashboard.putNumber("Algae/Intaking Delay", AlgaeConstants.kTimeItTakesToIntake)
        SmartDashboard.putNumber("Algae/Processing Delay", AlgaeConstants.kTimeItTakesToProcess)
        
    def updateSmartDashboard(self):
        '''Put all SmartDashboard stuff for this subsystem here'''
        self.updatePIDvalues(
            SmartDashboard.getNumber("Algae/Pivot P", self.PIDconfig.slot0.k_p), # TODO FIX THIS LATER PLS
            SmartDashboard.getNumber("Algae/Pivot I", self.PIDconfig.slot0.k_i),
            SmartDashboard.getNumber("Algae/Pivot D", self.PIDconfig.slot0.k_d),
            SmartDashboard.getNumber("Algae/Pivot G", self.PIDconfig.slot0.k_g),
        )
        # Output values
        SmartDashboard.putString("Algae/Pivot Position", self.pivotMotor.get_position().__str__())
        SmartDashboard.putString("Algae/Intake Speed", self.intakeMotor.get().__str__())
        SmartDashboard.putBoolean("Algae/Bottom Limit Switch", self.getBottomLimitSwitchActive())
        SmartDashboard.putBoolean("Algae/Top Limit Switch", self.getTopLimitSwitchActive())
        
        # Tuning values
        AlgaeConstants.kAmpValueToDetectIfMotorStalled = SmartDashboard.getNumber("Algae/Amp Value to Detect if Stalled", AlgaeConstants.kAmpValueToDetectIfMotorStalled)
        AlgaeConstants.kTargetValueAccuracy = SmartDashboard.getNumber("Algae/Target Value Accuracy", AlgaeConstants.kTargetValueAccuracy)
        AlgaeConstants.kPivotMaxHeight = SmartDashboard.getNumber("Algae/Pivot Max Height", AlgaeConstants.kPivotMaxHeight)
        AlgaeConstants.kPivotMinHeight = SmartDashboard.getNumber("Algae/Pivot Min Height", AlgaeConstants.kPivotMinHeight)
        AlgaeConstants.kPivotReefIntakingValue = SmartDashboard.getNumber("Algae/Pivot Reef Intaking Value", AlgaeConstants.kPivotReefIntakingValue)
        AlgaeConstants.kPivotGroundIntakingValue = SmartDashboard.getNumber("Algae/Pivot Ground Intaking Value", AlgaeConstants.kPivotGroundIntakingValue)
        AlgaeConstants.kPivotProcessingValue = SmartDashboard.getNumber("Algae/Pivot Processing Value", AlgaeConstants.kPivotProcessingValue)
        AlgaeConstants.kPivotIdleValue = SmartDashboard.getNumber("Algae/Pivot Idle Value", AlgaeConstants.kPivotIdleValue)
        AlgaeConstants.kPivotRotationsPerSecond = SmartDashboard.getNumber("Algae/Pivot Rotations Per Second", AlgaeConstants.kPivotRotationsPerSecond)
        AlgaeConstants.kIntakeMultiplier = SmartDashboard.getNumber("Algae/Intake Multiplier", AlgaeConstants.kIntakeMultiplier)
        AlgaeConstants.kTimeItTakesToIntake = SmartDashboard.getNumber("Algae/Intaking Delay", AlgaeConstants.kTimeItTakesToIntake)
        AlgaeConstants.kTimeItTakesToProcess = SmartDashboard.getNumber("Algae/Processing Delay", AlgaeConstants.kTimeItTakesToProcess)
        
    def periodic(self) -> None:
        # self.pivotMotor.set_control(self.positionVoltage.with_position(self.setpoint)) 
        # Sets setpoint to 0 if bottom limit switch active. No top limit switch though so umm...

        self.justHitLimitSwitch = False
        if self.getBottomLimitSwitchActive() and not self.justHitLimitSwitch:
            self.spinPivotMotor(0)
            self.pivotMotor.set_position(0)
            self.justHitLimitSwitch = True
        else:
            self.justHitLimitSwitch = False

        
        # Stop intake motor if motor current supply value says to stop
        # if self.getCurrent() > AlgaeConstants.kAmpValueToDetectIfMotorStalled:
        #     self.spinIntakeMotor(0)
        # Updates SmartDashboard
        self.updateSmartDashboard()
        
        # print(self.getBottomLimitSwitchActive(), self.pivotMotor.get_position().__str__())
