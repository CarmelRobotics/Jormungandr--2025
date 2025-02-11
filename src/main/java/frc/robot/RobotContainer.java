// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.ExtendState;
import frc.robot.subsystems.Arm.PivotState;
import frc.robot.subsystems.Intake.IntakeState;

import java.lang.management.OperatingSystemMXBean;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public CommandXboxController kController;
  public Arm arm;
  public Intake intake;
  private Command retract;
  private Command waitForExtension;
  private Command waitForPivot;

  private Command stowArm;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    retract  = arm.setExtend(ExtendState.STOW);
    waitForExtension =  Commands.waitUntil(()->arm.atExtendSetpoint());
    waitForPivot = Commands.waitUntil(()->arm.atPivotSetpoint());
    stowArm = Commands.sequence(arm.setExtend(ExtendState.STOW),Commands.waitUntil(()->arm.atExtendSetpoint()),arm.setPivot(PivotState.STOW)); 
    kController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    DogLog.setPdh(new PowerDistribution(1, ModuleType.kRev));
    DogLog.setEnabled(true);
    DogLog.setOptions(new DogLogOptions().withNtPublish(true).withCaptureDs(true).withLogExtras(true));
    // Configure the trigger bindings
    configureBindings();
    
  }
  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    kController.rightTrigger().onTrue(new ParallelCommandGroup(intake.sendIntakeRequest(IntakeState.INTAKING),new SequentialCommandGroup(arm.setPivot(PivotState.INTAKE_FRONT),new WaitUntilCommand(()->arm.atPivotSetpoint()),arm.setExtend(ExtendState.INTAKE_FRONT)))).onFalse(stowArm);
   // kController.povDown().onTrue(stowArm);
    kController.a().onTrue(Commands.sequence(retract,waitForExtension,arm.setPivot(PivotState.L4),waitForPivot,arm.setExtend(ExtendState.L4))).onFalse(stowArm);
    kController.b().onTrue(Commands.sequence(retract,waitForExtension,arm.setPivot(PivotState.L3),waitForPivot,arm.setExtend(ExtendState.L3))).onFalse(stowArm);
    kController.x().onTrue(Commands.sequence(retract,waitForExtension,arm.setPivot(PivotState.L2),waitForPivot,arm.setExtend(ExtendState.L2))).onFalse(stowArm);
    kController.y().onTrue(Commands.sequence(retract,waitForExtension,arm.setPivot(PivotState.L1),waitForPivot,arm.setExtend(ExtendState.L1))).onFalse(stowArm);
    kController.leftTrigger().onTrue(Commands.parallel(Commands.sequence(retract,waitForExtension,arm.setPivot(PivotState.INTAKE_BACK),waitForPivot,arm.setExtend(ExtendState.INTAKE_BACK)),intake.sendIntakeRequest(IntakeState.INTAKING))).onFalse(stowArm);
    kController.leftBumper().onTrue(intake.sendIntakeRequest(IntakeState.OUTTAKING));
    kController.rightBumper().onTrue(Commands.sequence(retract,waitForExtension,arm.setPivot(PivotState.ALGAE),waitForPivot,arm.setExtend(ExtendState.ALGAE))).onFalse(stowArm);
    kController.povUp().onTrue(Commands.sequence(retract,waitForExtension,arm.setPivot(PivotState.PROCESSOR),waitForPivot,arm.setExtend(ExtendState.PROCESSOR))).onFalse(stowArm);
    kController.povRight().onTrue(Commands.sequence(retract,waitForExtension,arm.setPivot(PivotState.ALGAE_REEF),waitForPivot,arm.setExtend(ExtendState.ALGAE_REEF))).onFalse(stowArm);
    kController.povLeft().onTrue(arm.dunkCoral());
    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in 
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
