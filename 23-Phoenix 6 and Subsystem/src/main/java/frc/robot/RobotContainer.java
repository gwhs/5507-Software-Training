// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystem.arm.ArmSubsystem;
import frc.robot.subsystem.elevator.ElevatorSubsystem;

public class RobotContainer {
  /*
   * Subsystems
   */
  private final ArmSubsystem arm = new ArmSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();

  private final RobotVisualizer robotVisualizer = new RobotVisualizer(arm, elevator);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
