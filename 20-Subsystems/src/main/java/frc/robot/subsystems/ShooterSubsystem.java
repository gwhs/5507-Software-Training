// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.fasterxml.jackson.databind.ser.std.StaticListSerializerBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final int TalonFX = 21;
  //private final TalonFX motor1;
  private final TalonFX motor;

  public ShooterSubsystem(CANBus canBus) {
    motor = new TalonFX(21, canBus);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor.getConfigurator().apply(config); //run on wednesday pg 57
    //motor1 = new TalonFX(22, canBus);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //motor.setVoltage(1);
    //motor1.setVoltage(1);
  }
  public Command startShooter() {
    return this.runOnce(() -> {
      motor.setVoltage(1);
    });
  }

  public Command stopShooter() {
    return this.runOnce(() -> {
      motor.setVoltage((0));
    });
  }
  public Command runVoltage(double volts) {
    return this.runOnce(() -> {
      motor.setVoltage(volts);
    });
  }
}
