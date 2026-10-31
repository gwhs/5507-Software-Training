// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new PivotSubsystem. */
  private final TalonFX motor;

  private double goalPosition = 0;

  private final Trigger isAtPosition = new Trigger(() -> MathUtil.isNear(goalPosition, getPosition(), 10));

  private final Alert motorNotConnectedAlert = new Alert ("Pivot Motor Not Connected", AlertType.kError);

  private final StatusSignal <Voltage> motorVoltage;
  private final StatusSignal <Temperature> motorTemp;
  private final StatusSignal <Current> motorStatorCurrent;
  private final StatusSignal <AngularVelocity> motorVelocity;

  public PivotSubsystem(CANBus canBus) {
    motor = new TalonFX (32, canBus);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.CurrentLimits.StatorCurrentLimit = 20;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command runPosition() {
    return this.runOnce(() -> 
      motor.setControl());
    };

  public double getPosition() {
    return motorPosition;
  }
}
