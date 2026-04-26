// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundIntakeRoller;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntakeRollerSubsystem extends SubsystemBase {
  private final CANBus canBus = CANBus.roboRIO();
  private final TalonFX motor1 = new TalonFX(GroundIntakeRollerConstants.MOTOR_1_ID, canBus);
  private final TalonFX motor2 = new TalonFX(GroundIntakeRollerConstants.MOTOR_2_ID, canBus);

  private final Follower controlRequest =
      new Follower(GroundIntakeRollerConstants.MOTOR_1_ID, MotorAlignmentValue.Opposed);

  /** Creates a new GoundIntakeRollerSubsystem. */
  public GroundIntakeRollerSubsystem() {
    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.CurrentLimits.StatorCurrentLimit = 30;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motor1.getConfigurator().apply(talonFXConfig);
    motor2.getConfigurator().apply(talonFXConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command stopIntake() {
    return this.runOnce(
        () -> {
          motor1.stopMotor();
          motor2.stopMotor();
        });
  }

  public Command startIntake() {
    return this.runOnce(
        () -> {
          motor1.setVoltage(GroundIntakeRollerConstants.INTAKE_VOLTAGE);
          motor2.setControl(controlRequest);
        });
  }
}
