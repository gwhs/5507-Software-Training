// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final CANBus canBus = CANBus.roboRIO();
  private final TalonFX motor1 = new TalonFX(ShooterConstants.MOTOR_1_ID, canBus); // Right Front
  private final TalonFX motor2 = new TalonFX(ShooterConstants.MOTOR_2_ID, canBus); // Right Back
  private final TalonFX motor3 = new TalonFX(ShooterConstants.MOTOR_3_ID, canBus); // Middle Front
  private final TalonFX motor4 = new TalonFX(ShooterConstants.MOTOR_4_ID, canBus); // Middle Back
  private final TalonFX motor5 = new TalonFX(ShooterConstants.MOTOR_5_ID, canBus); // Left Front
  private final TalonFX motor6 = new TalonFX(ShooterConstants.MOTOR_6_ID, canBus); // Left Back

  private final VelocityVoltage velocityRequest1 = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest2 = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest3 = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest4 = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest5 = new VelocityVoltage(0).withEnableFOC(true);
  private final VelocityVoltage velocityRequest6 = new VelocityVoltage(0).withEnableFOC(true);

  public ShooterSubsystem() {
    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    talonFXConfig.CurrentLimits.StatorCurrentLimit = 40;
    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    talonFXConfig.Slot0.kS = 0.2;
    talonFXConfig.Slot0.kG = 0;
    talonFXConfig.Slot0.kA = 0;
    talonFXConfig.Slot0.kV = 0.122;
    talonFXConfig.Slot0.kP = 0.1;
    talonFXConfig.Slot0.kI = 0;
    talonFXConfig.Slot0.kD = 0;

    motor1.getConfigurator().apply(talonFXConfig);
    motor2.getConfigurator().apply(talonFXConfig);
    motor3.getConfigurator().apply(talonFXConfig);
    motor4.getConfigurator().apply(talonFXConfig);
    motor5.getConfigurator().apply(talonFXConfig);
    motor6.getConfigurator().apply(talonFXConfig);
  }

  public Command runVelocity(double rotationsPerSecond) {
    return this.runOnce(
            () -> {
              motor1.setControl(velocityRequest1.withVelocity(rotationsPerSecond));
              motor2.setControl(velocityRequest2.withVelocity(rotationsPerSecond));
              motor3.setControl(velocityRequest3.withVelocity(rotationsPerSecond));
              motor4.setControl(velocityRequest4.withVelocity(rotationsPerSecond));
              motor5.setControl(velocityRequest5.withVelocity(rotationsPerSecond));
              motor6.setControl(velocityRequest6.withVelocity(rotationsPerSecond));
            })
        .withName("Run Velocity");
  }

  public Command stopShooters() {
    return this.runOnce(
        () -> {
          motor1.stopMotor();
          motor2.stopMotor();
          motor3.stopMotor();
          motor4.stopMotor();
          motor5.stopMotor();
          motor6.stopMotor();
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
