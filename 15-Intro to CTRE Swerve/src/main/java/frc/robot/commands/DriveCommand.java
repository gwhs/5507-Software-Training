// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  /** Creates a new DriveCommand. */
  private final SwerveSubsystem drivetrain;

  private final CommandXboxController controller;

  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();

  private final double maxSpeed = 4.5; // ms/s
  private final double maxAngularSpeed = 2.5 * Math.PI; // radians

  private final double deadband = 0.06;

  public DriveCommand(SwerveSubsystem drivetrain, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xInput = -controller.getLeftY();
    double yInput = -controller.getLeftX();
    double rotationInput = -controller.getRightX();

    //deadband
    xInput = MathUtil.applyDeadband(xInput, deadband);
    yInput = MathUtil.applyDeadband(yInput, deadband);
    rotationInput = MathUtil.applyDeadband(rotationInput, deadband);

    if (drivetrain.isSlowMode()) {
      xInput = xInput * drivetrain.getTranslationSlowFactor();
      yInput = yInput * drivetrain.getTranslationSlowFactor();
      rotationInput = rotationInput * drivetrain.getRotationSlowFactor();
    }

    double xVelocity = xInput * maxSpeed;
    double yVelocity = yInput * maxSpeed;
    double rotationVelocity = rotationInput * maxAngularSpeed;

    drivetrain.setControl(
        fieldCentric
            .withVelocityX(xVelocity)
            .withVelocityY(yVelocity)
            .withRotationalRate(rotationVelocity));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
