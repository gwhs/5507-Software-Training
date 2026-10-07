// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants_Anemone;

public class RobotContainer {
  private final SwerveSubsystem drivetrain = TunerConstants_Anemone.createDrivetrain();

  private final CommandXboxController controller = new CommandXboxController(0);

  private final DriveCommand defaultDriveCommand = new DriveCommand(drivetrain, controller);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(defaultDriveCommand);
    controller.leftTrigger().onTrue(drivetrain.setSlowMode(true));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
