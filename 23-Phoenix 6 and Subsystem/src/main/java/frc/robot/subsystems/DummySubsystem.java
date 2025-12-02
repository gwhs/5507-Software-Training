// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DummySubsystem extends SubsystemBase {
  /** Creates a new DummySubsystem. */
  
  private final TalonFX motor = new TalonFX(1, "rio");

  public DummySubsystem() {
    SmartDashboard.putData("Run Motor at 1V", runVoltage(1));
    SmartDashboard.putData("Run Motor at 0V", runVoltage(0));
    SmartDashboard.putData("Run Motor at -1V", runVoltage(-1));
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

