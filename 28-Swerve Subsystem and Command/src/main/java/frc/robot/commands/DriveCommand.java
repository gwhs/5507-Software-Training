// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem.RotationTarget;

public class DriveCommand extends Command {
  private final SwerveSubsystem drivetrain;
  private final CommandXboxController controller;

  private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();

  private final double maxSpeed = 4.5; // m/s
  private final double maxAngularSpeed = 2.5 * Math.PI; // radians/s

  private final double deadband = 0.1;

  private final PIDController robotHeadingController = new PIDController(0.2, 0, 0);

  public DriveCommand(SwerveSubsystem drivetrain, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;

    robotHeadingController.enableContinuousInput(-180, 180);

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

    // Apply deadband
    xInput = MathUtil.applyDeadband(xInput, deadband);
    yInput = MathUtil.applyDeadband(yInput, deadband);
    rotationInput = MathUtil.applyDeadband(rotationInput, deadband);

    // Auto Rotate
    boolean hasRotationInput = Math.abs(controller.getRightX()) > 0.1;
    
    if(drivetrain.getRotationTarget() != RotationTarget.NORMAL && !hasRotationInput) {
      double currentRobotHeading = drivetrain.getState().Pose.getRotation().getDegrees();

      robotHeadingController.setSetpoint(drivetrain.getGoalHeading());

      double pidOutput = robotHeadingController.calculate(currentRobotHeading);
      rotationInput = pidOutput;

      DogLog.log("Drive Command/Auto Rotate PID output", pidOutput);
      DogLog.log("Drive Command/Auto Rotate goal (degree)", drivetrain.getGoalHeading());
      DogLog.log("Drive Command/Current Robot Heading (degree)", currentRobotHeading);
    }

    // Slow Mode
    if(drivetrain.isSlowMode()) {
      xInput = xInput * drivetrain.getTranslationSlowFactor();
      yInput = yInput * drivetrain.getTranslationSlowFactor();
      rotationInput = rotationInput * drivetrain.getRotationalSlowFactor();
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
