// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  private final CANBus rioBus = new CANBus("rio");
  private final CANBus canivoreBus = new CANBus("CAN_Network");
  private final CommandXboxController controller = new CommandXboxController(0);

  private final ShooterSubsystem shooter = new ShooterSubsystem(rioBus);

  public RobotContainer() {
    
    configureBindings();
  }

  private void configureBindings() {
    controller.a().onTrue(shooter.startShooter());
    controller.a().onFalse(shooter.stopShooter()); //do slide 41 on wednesday
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
