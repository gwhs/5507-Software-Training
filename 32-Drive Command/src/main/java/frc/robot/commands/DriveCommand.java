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
  private SwerveSubsystem swerve;
  private CommandXboxController controller;
  private SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

  private final double maxSpeed = 4.5; // m/s
  private final double maxAngularSpeed = 2.5 * Math.PI; // radianss/s//s/s/s/s

  private final double deadband = 0.06;
  
  public DriveCommand(SwerveSubsystem swerve, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.controller = controller;

    addRequirements(swerve);

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

    xInput = MathUtil.applyDeadband(xInput, deadband);
    yInput = MathUtil.applyDeadband(yInput, deadband);
    rotationInput = MathUtil.applyDeadband(rotationInput, deadband);

    if(swerve.isSlowMode()) {
      xInput = xInput * swerve.getTranslationSlowFactor();
      yInput = yInput * swerve.getTranslationSlowFactor();
      rotationInput = rotationInput * swerve.getRotationalSlowFactor();
    }

    double xVelocity = maxSpeed * xInput;
    double yVelocity = maxSpeed * yInput;
    double rotationVelocity = maxAngularSpeed * rotationInput;

    swerve.setControl(request.withVelocityX(xVelocity).withVelocityY(yVelocity).withRotationalRate(rotationVelocity));
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
