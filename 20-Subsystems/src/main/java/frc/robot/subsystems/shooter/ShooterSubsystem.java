// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX motor;

  private final Alert motorNotConnectedAlert = new Alert("Shooter Motor Not Connected", AlertType.kError);

  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Temperature> motorTemp;
  private final StatusSignal<Current> motorStatorCurrent;

  private final VelocityVoltage request = new VelocityVoltage(0);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(CANBus canBus) {
    motor = new TalonFX(21, canBus);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 10;

    config.Slot0.kS = 0;
    config.Slot0.kV = 0.1125;
    config.Slot0.kP = 0;

    motor.getConfigurator().apply(config);

    motorVoltage = motor.getMotorVoltage();
    motorTemp = motor.getDeviceTemp();
    motorStatorCurrent = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50, motorVoltage, motorTemp, motorStatorCurrent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motorNotConnectedAlert.set(!motor.isConnected());

    BaseStatusSignal.refreshAll(motorVoltage, motorTemp, motorStatorCurrent);

    DogLog.log("Shooter/Motor Voltage", motorVoltage.getValueAsDouble());
    DogLog.log("Shooter/Motor Temp", motorTemp.getValueAsDouble());
    DogLog.log("Shooter/Stator Current", motorStatorCurrent.getValueAsDouble());
  }

  public Command startShooter() {
    return this.runOnce(() -> {
      motor.setVoltage(1);
    });
  }

  public Command stopShooter() {
    return this.runOnce(() -> {
      motor.setVoltage(0);
    });
  }

  public Command runVoltage(double volt) {
    return this.runOnce(() -> {
      motor.setVoltage(volt);
    });
  }

  public Command runVelocity (double rps) {
    return this.runOnce(() -> {
      motor.setControl(request.withVelocity(rps));
    });
  }
}
