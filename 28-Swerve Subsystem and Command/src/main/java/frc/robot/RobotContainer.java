// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants_Comp;

public class RobotContainer {
  private final SwerveSubsystem drivetrain = TunerConstants_Comp.createDrivetrain();

  private final CommandXboxController controller = new CommandXboxController(0);

  private final DriveCommand defualtDriveCommand = new DriveCommand(drivetrain, controller);

  public RobotContainer() {
    configureBindings();

    drivetrain.setDefaultCommand(defualtDriveCommand);
  }

  private void configureBindings() {
    controller.x().onTrue(drivetrain.setSlowMode(true)).onFalse(drivetrain.setSlowMode(false));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
