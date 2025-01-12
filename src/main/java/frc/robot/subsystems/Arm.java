package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private TalonFX pivotOne;
    private TalonFX pivotTwo;
    public ArmState armState;
    private TalonFX extend;
    private TalonFXConfiguration pivotOneConfig = new TalonFXConfiguration();
    private TalonFXConfiguration pivotTwoConfig = new TalonFXConfiguration();
    private TalonFXConfiguration extendConfig = new TalonFXConfiguration();
    private FeedbackConfigs pivotFeedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio((20) * (40/18));
    private Slot0Configs extendPIDConfigs = new Slot0Configs().withKP(ArmConstants.extend_kP);
    private FeedbackConfigs extendFeedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(4);
    private MotionMagicConfigs pivotOneMotionMagicConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(ArmConstants.kArmMaxAccel).withMotionMagicCruiseVelocity(ArmConstants.kArmMaxVel); 
    private MotionMagicConfigs pivotTwoMotionMagicConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(ArmConstants.kArmMaxAccel).withMotionMagicCruiseVelocity(ArmConstants.kArmMaxVel); 
    private MotionMagicConfigs extendMotionMagicConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(ArmConstants.kArmMaxAccel).withMotionMagicCruiseVelocity(ArmConstants.kArmMaxVel); 
    private CurrentLimitsConfigs pivotOneCurrentLimitsConfigs = new CurrentLimitsConfigs().withSupplyCurrentLimit(ArmConstants.kArmCurrentLimit);
    private CurrentLimitsConfigs pivotTwoCurrentLimitsConfigs = new CurrentLimitsConfigs().withSupplyCurrentLimit(ArmConstants.kArmCurrentLimit);
    private CurrentLimitsConfigs extendCurrentLimitsConfigs = new CurrentLimitsConfigs().withSupplyCurrentLimit(ArmConstants.kArmCurrentLimit);
    private MotionMagicVoltage pivotControl = new MotionMagicVoltage(0);
    //private MotionMagicVoltage extendControl = new MotionMagicVoltage(0);
    private PositionVoltage extendControl  = new PositionVoltage(0).withSlot(0);
    
    
    public Arm(){
        pivotOneConfig.withMotionMagic(pivotOneMotionMagicConfigs).withCurrentLimits(pivotOneCurrentLimitsConfigs).withFeedback(pivotFeedbackConfigs);
        pivotTwoConfig.withMotionMagic(pivotTwoMotionMagicConfigs).withCurrentLimits(pivotTwoCurrentLimitsConfigs).withFeedback(extendFeedbackConfigs);
        extendConfig.withMotionMagic(extendMotionMagicConfigs).withCurrentLimits(extendCurrentLimitsConfigs).withSlot0(extendPIDConfigs);
        pivotOne = new TalonFX(ArmConstants.kPivotOneCANID);
        pivotTwo = new TalonFX(ArmConstants.kPivotTwoCANID);
        extend = new TalonFX(ArmConstants.kExtendCANID);
        armState = ArmState.STOW;
        pivotOne.setControl(pivotControl);
        extend.setControl(extendControl);
    }
    public static enum ArmState{
        INTAKE_FRONT(ArmConstants.intakeFront[0],ArmConstants.intakeFront[1]),
        INTAKE_BACK(ArmConstants.intakeBack[0],ArmConstants.intakeBack[1]),
        STOW(ArmConstants.Stow[0],ArmConstants.Stow[1]),
        L1(ArmConstants.L1[0],ArmConstants.L1[1]),
        L2(ArmConstants.L2[0],ArmConstants.L2[1]),
        L3(ArmConstants.L3[0],ArmConstants.L3[1]),
        L4(ArmConstants.L4[0],ArmConstants.L4[1]),
        PROCESSOR(ArmConstants.Processor[0],ArmConstants.Processor[1]),
        ALGAE_REEF(ArmConstants.AlgaeReef[0],ArmConstants.AlgaeReef[1]);
        public double pivotSetpoint;
        public double extendSetpoint;
        private ArmState(double pivotSetpoint, double extendSetpoint){
            this.pivotSetpoint = pivotSetpoint;
            this.extendSetpoint = extendSetpoint;
        }
    }
    @Override
    public void periodic() {
        DogLog.log("Arm state", this.armState);
        pivotControl.withPosition(this.armState.pivotSetpoint);
        extendControl.withPosition(this.armState.extendSetpoint);
        

    }
    public Command setState(ArmState newState){
        return run(()->{
            this.armState = newState;
        });
    }

    
    
    

    
}
