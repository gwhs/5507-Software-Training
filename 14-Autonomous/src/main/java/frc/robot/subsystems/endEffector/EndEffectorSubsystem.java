// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endEffector;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {
  private FlywheelSim motor =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getFalcon500Foc(1), 0.001, EndEffectorConstants.GEAR_RATIO),
          DCMotor.getFalcon500Foc(1));

  private boolean sensor = false;

  public EndEffectorSubsystem() {
    SmartDashboard.putData(
        "Simulate End Effector Sensor",
        Commands.sequence(
            Commands.runOnce(() -> sensor = true),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> sensor = false)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command runVoltage(double voltage) {
    return Commands.runOnce(() -> motor.setInputVoltage(voltage));
  }

  public boolean hasGamePiece() {
    return sensor;
  }
}
