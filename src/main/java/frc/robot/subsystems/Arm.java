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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private TalonFX pivotOne;
    private TalonFX pivotTwo;
    public PivotState pivotState;
    public ExtendState extendState;
    public static PivotState prevPivotState;
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
            pivotOne.getConfigurator().apply(pivotOneConfig);
            pivotTwo.getConfigurator().apply(pivotTwoConfig);
            extend.getConfigurator().apply(extendConfig);
            pivotState = PivotState.STOW;
            extendState = ExtendState.STOW;
            prevPivotState = PivotState.STOW;
            pivotOne.setControl(pivotControl);
            extend.setControl(extendControl);
        }
        public static enum PivotState{
            INTAKE_FRONT(ArmConstants.intakeFront[0]),
            INTAKE_BACK(ArmConstants.intakeBack[0]),
            STOW(ArmConstants.Stow[0]),
            L1(ArmConstants.L1[0]),
            L2(ArmConstants.L2[0]),
            L3(ArmConstants.L3[0]),
            L4(ArmConstants.L4[0]),
            PROCESSOR(ArmConstants.Processor[0]),
            ALGAE_REEF(ArmConstants.AlgaeReef[0]),
            DUNK(prevPivotState.pivotSetpoint - 10);
        public double pivotSetpoint;
        private PivotState(double pivotSetpoint){
            this.pivotSetpoint = pivotSetpoint;
        }
    }
        public static enum ExtendState{
            INTAKE_FRONT(ArmConstants.intakeFront[1]),
            INTAKE_BACK(ArmConstants.intakeBack[1]),
            STOW(ArmConstants.Stow[1]),
            L1(ArmConstants.L1[1]),
            L2(ArmConstants.L2[1]),
            L3(ArmConstants.L3[1]),
            L4(ArmConstants.L4[1]),
            PROCESSOR(ArmConstants.Processor[1]),
            ALGAE_REEF(ArmConstants.AlgaeReef[1]);
            public double extendSetpoint;
        private ExtendState(double extendSetpoint){
            this.extendSetpoint = extendSetpoint;
        }
    }
    @Override
    public void periodic() {
        DogLog.log("Arm state", this.pivotState);
        DogLog.log("Pivot current draw", pivotOne.getSupplyCurrent().getValueAsDouble() + pivotTwo.getSupplyCurrent().getValueAsDouble() );
        pivotControl.withPosition(Rotation2d.fromDegrees(this.pivotState.pivotSetpoint).getRotations());
        extendControl.withPosition(this.extendState.extendSetpoint);
        

    }
    public Command setPivot(PivotState newState){
        return runOnce(()->{
            this.pivotState = newState;
        });
    }
    public Command setExtend(ExtendState newState){
        return runOnce(()->{
            this.extendState = newState;
        });
    }
    public boolean atPivotSetpoint(){
        return MathUtil.isNear(this.pivotState.pivotSetpoint, Units.rotationsToDegrees(pivotControl.Position), 3); 
    }
    public boolean atExtendSetpoint(){
        return MathUtil.isNear(this.extendState.extendSetpoint, extendControl.Position, 3); 
    }
    public double getArmCurrentDraw(){
        return pivotOne.getSupplyCurrent().getValueAsDouble() + pivotTwo.getSupplyCurrent().getValueAsDouble() + extend.getSupplyCurrent().getValueAsDouble();
    }
    public PivotState getPivotState(){
        return this.pivotState;
    }
    public Command dunkCoral(){
        return runOnce(
            ()->{
                prevPivotState = this.pivotState;
                this.pivotState = PivotState.DUNK;
            }
        );
    }


    
    
    

    
}
