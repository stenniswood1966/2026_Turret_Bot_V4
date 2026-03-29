// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  private Boolean killFlag = false;
  public static TalonFX motor1 = new TalonFX(constants.kg_IntakeSubsystem.k_Motor1ID,constants.kg_IntakeSubsystem.k_Motor1CANBus);
  private static TalonFX motor2 = new TalonFX(constants.kg_IntakeSubsystem.k_Motor2ID, constants.kg_IntakeSubsystem.k_Motor2CANBus);


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    var fx_cfg = new TalonFXConfiguration();

    //configuration for motor1
    fx_cfg.MotorOutput.Inverted = constants.kg_IntakeSubsystem.k_Motor1InvertedValue;
    fx_cfg.MotorOutput.NeutralMode = constants.kg_IntakeSubsystem.k_Motor1NeutralModeValue;

    //amp limits
    fx_cfg.CurrentLimits.StatorCurrentLimit = constants.kg_IntakeSubsystem.k_Motor1StatorCurrentLimit;
    fx_cfg.CurrentLimits.StatorCurrentLimitEnable = constants.kg_IntakeSubsystem.k_Motor1StatorCurrentLimitEnable;
    fx_cfg.CurrentLimits.SupplyCurrentLimit = constants.kg_IntakeSubsystem.k_Motor1SupplyCurrentLimit;
    fx_cfg.CurrentLimits.SupplyCurrentLimitEnable = constants.kg_IntakeSubsystem.k_Motor1StatorCurrentLimitEnable;
    fx_cfg.CurrentLimits.SupplyCurrentLowerLimit = constants.kg_IntakeSubsystem.k_Motor1SupplyCurrentLowerLimit;
    fx_cfg.CurrentLimits.SupplyCurrentLowerTime = constants.kg_IntakeSubsystem.k_Motor1SupplyCurrentLowerTime;

    motor1.getConfigurator().apply(fx_cfg, 0.050);
  
    //config motor 2
    var fx_cfg2 = new TalonFXConfiguration(); //creates a default TalonFX configuration

    fx_cfg2.CurrentLimits.StatorCurrentLimit = constants.kg_IntakeSubsystem.k_Motor1StatorCurrentLimit;
    fx_cfg2.CurrentLimits.StatorCurrentLimitEnable = constants.kg_IntakeSubsystem.k_Motor1StatorCurrentLimitEnable;
    fx_cfg2.CurrentLimits.SupplyCurrentLimit = constants.kg_IntakeSubsystem.k_Motor1SupplyCurrentLimit;
    fx_cfg2.CurrentLimits.SupplyCurrentLimitEnable = constants.kg_IntakeSubsystem.k_Motor1StatorCurrentLimitEnable;
    fx_cfg2.CurrentLimits.SupplyCurrentLowerLimit = constants.kg_IntakeSubsystem.k_Motor1SupplyCurrentLowerLimit;
    fx_cfg2.CurrentLimits.SupplyCurrentLowerTime = constants.kg_IntakeSubsystem.k_Motor1SupplyCurrentLowerTime;

    //configuration for motor 2
    fx_cfg2.MotorOutput.NeutralMode = constants.kg_IntakeSubsystem.k_Motor2NeutralModeValue;

    motor2.getConfigurator().apply(fx_cfg2, 0.050);

    motor2.setControl(new Follower(constants.kg_IntakeSubsystem.k_Motor1ID, constants.kg_IntakeSubsystem.k_Motor2AlignmentValue));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler runOnce
  }

  public Command stopCommand(){
    return runOnce(() -> stop());
  }

  private void stop()
  {
    motor1.set(0);
  }

  public Command intakeCommand(){
    return runOnce(() -> intakeMethod());
  }

  private void intakeMethod(){
    if (!killFlag) {
      motor1.setControl(new DutyCycleOut(constants.kg_IntakeSubsystem.k_IntakePower));
    }
  }

  public Command antiJamCommand(){
    return runOnce(() -> antiJamMethod());
  }

  private void antiJamMethod(){
    if (!killFlag) {
      motor1.setControl(new DutyCycleOut(constants.kg_IntakeSubsystem.k_AntiJamPower));
    }
  }

  public Command killCommand(){
    return runOnce(() -> killMethod());
  }

  private void killMethod(){
    stop();
    killFlag = !killFlag;
  }
}
