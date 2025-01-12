package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private TalonFX pivotOne;
    private TalonFX pivotTwo;
    private TalonFX extend;
    private TalonFXConfiguration pivotOneConfig = new TalonFXConfiguration();
    private TalonFXConfiguration pivotTwoConfig = new TalonFXConfiguration();
    private TalonFXConfiguration extendConfig = new TalonFXConfiguration();
    private MotionMagicConfigs pivotOneMotionMagicConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(ArmConstants.kArmMaxAccel).withMotionMagicCruiseVelocity(ArmConstants.kArmMaxVel); 
    private MotionMagicConfigs pivotTwoMotionMagicConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(ArmConstants.kArmMaxAccel).withMotionMagicCruiseVelocity(ArmConstants.kArmMaxVel); 
    private MotionMagicConfigs extendMotionMagicConfigs = new MotionMagicConfigs().withMotionMagicAcceleration(ArmConstants.kArmMaxAccel).withMotionMagicCruiseVelocity(ArmConstants.kArmMaxVel); 
    private CurrentLimitsConfigs pivotOneCurrentLimitsConfigs = new CurrentLimitsConfigs().withSupplyCurrentLimit(ArmConstants.kArmCurrentLimit);
    private CurrentLimitsConfigs pivotTwoCurrentLimitsConfigs = new CurrentLimitsConfigs().withSupplyCurrentLimit(ArmConstants.kArmCurrentLimit);
    private CurrentLimitsConfigs extendCurrentLimitsConfigs = new CurrentLimitsConfigs().withSupplyCurrentLimit(ArmConstants.kArmCurrentLimit);
    public Arm(){
        pivotOneConfig.withMotionMagic(pivotOneMotionMagicConfigs).withCurrentLimits(pivotOneCurrentLimitsConfigs);
        pivotTwoConfig.withMotionMagic(pivotOneMotionMagicConfigs).withCurrentLimits(pivotTwoCurrentLimitsConfigs);
        extendConfig.withMotionMagic(extendMotionMagicConfigs).withCurrentLimits(extendCurrentLimitsConfigs);
        pivotOne = new TalonFX(ArmConstants.kPivotOneCANID);
        pivotTwo = new TalonFX(ArmConstants.kPivotTwoCANID);
        extend = new TalonFX(ArmConstants.kExtendCANID);
    }
    public static enum ArmState{
        INTAKE_FRONT,
        INTAKE_BACK,
        STOW,
        L1,
        L2,
        L3,
        L4,
        PROCESSOR,
        ALGAE_REEF;

    }
    
    

    
}
