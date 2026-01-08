// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DummySubsystem extends SubsystemBase {
  /** Creates a new DummySubsystem. */
  
  private final TalonFX motor = new TalonFX(1, "rio");

  public DummySubsystem() {

    TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();

    //talonFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfig.CurrentLimits.StatorCurrentLimit = 1;

    talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonFXConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5;
    talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonFXConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -5;

    SmartDashboard.putData("Run Motor at 1V", runVoltage(1));
    SmartDashboard.putData("Run Motor at 0V", runVoltage(0));
    SmartDashboard.putData("Run Motor at -1V", runVoltage(-1));

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++)
    {
      status = motor.getConfigurator().apply(talonFXConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK())
    {
      new Alert("Could not configure device. Error: " + status.toString(), AlertType.kError).set(true);
    }
  }

  @Override
  public void periodic() {}

    // This method will be called once per scheduler run
    public Command runVoltage(double voltage) {
      return this.runOnce(() -> {
        motor.setVoltage(voltage);
      });
    }
    
  }

