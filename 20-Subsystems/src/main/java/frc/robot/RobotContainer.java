// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RobotContainer {

  private final CommandXboxController controller = new CommandXboxController(0);

  private final CANBus rioCANBus = new CANBus("rio");
  private final CANBus canivoreCANBus = new CANBus("CAN_Network");

  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(canivoreCANBus);
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem(canivoreCANBus);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.a().onTrue(shooterSubsystem.runVoltage(12));
    controller.a().onFalse(shooterSubsystem.runVoltage(0));

    controller.y().onTrue(pivotSubsystem.runPosition(0));
    controller.x().onTrue(pivotSubsystem.runPosition(5));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void periodic() {
  }
}
