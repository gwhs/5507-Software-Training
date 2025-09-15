// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem.FaceTarget;
import frc.robot.subsystems.swerve.TunerConstants_Comp;

public class RobotContainer {
  private final SwerveSubsystem swerve = TunerConstants_Comp.createDrivetrain();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();

  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(swerve.defaultDrive(controller));

    controller.povUp().whileTrue(elevator.runVoltage(3));
    controller.povDown().whileTrue(elevator.runVoltage(-3));

    controller.y().whileTrue(elevator.runPosition(0.85));
    controller.x().whileTrue(elevator.runPosition(0.5));
    controller.b().whileTrue(elevator.runPosition(0.3));
    controller.a().whileTrue(elevator.runPosition(0));

    controller
        .leftBumper()
        .whileTrue(swerve.faceTarget(FaceTarget.FORTY_FIVE))
        .onFalse(swerve.faceTarget(FaceTarget.NONE));
    controller
        .rightBumper()
        .whileTrue(swerve.faceTarget(FaceTarget.REEF))
        .onFalse(swerve.faceTarget(FaceTarget.NONE));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
