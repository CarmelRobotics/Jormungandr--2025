package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private IntakeState intakeState = IntakeState.IDLE;
    private TalonFX intakeMotor;
    private TalonFXConfiguration intakeConfiguration;
    private CurrentLimitsConfigs intakeCurrentConfig = new CurrentLimitsConfigs().withSupplyCurrentLimit(IntakeConstants.kIntakeCurrentLimit).withSupplyCurrentLimitEnable(true);
    private VoltageOut intakeRequest = new VoltageOut(0);
    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.kIntakeCANID);
        intakeConfiguration.withCurrentLimits(intakeCurrentConfig);
        intakeMotor.getConfigurator().apply(intakeConfiguration);
        intakeMotor.setControl(intakeRequest);

    }
    public static enum IntakeState{
        INTAKING(12),
        HOLDING(5),
        OUTTAKING(-12),
        IDLE(0);
        public double voltage;
        private IntakeState(double voltage){
            this.voltage = voltage;
        }
    }
    @Override
    public void periodic() {
        intakeRequest.withOutput(this.intakeState.voltage);
        if(MathUtil.isNear(30, intakeMotor.getSupplyCurrent().getValueAsDouble(),.75) && MathUtil.isNear(0, intakeMotor.getVelocity().getValueAsDouble(), 0.5)&&this.intakeState != IntakeState.OUTTAKING){
            this.intakeState = IntakeState.HOLDING;
        }
        DogLog.log("Intake state", this.intakeState);
    }
    public Command sendIntakeRequest(IntakeState newState){
        return runOnce(()->{
            this.intakeState = newState;
        });
    }
    public IntakeState getState(){
        return this.intakeState;
    }
    
}

