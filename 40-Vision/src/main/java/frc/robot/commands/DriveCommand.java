// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants_mk5n;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCommand extends Command {
  private final CommandXboxController controller;
  private final SwerveSubsystem swerve;

  private double maxSpeed = TunerConstants_mk5n.kSpeedAt12Volts.in(MetersPerSecond);
  private double maxAngularRate = 2.5 * Math.PI;

  private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();

  /** Creates a new DriveCommand. */
  public DriveCommand(CommandXboxController controller, SwerveSubsystem swerve) {
    this.controller = controller;
    this.swerve = swerve;
    
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xInput = MathUtil.applyDeadband(-controller.getLeftY(), 0.1);
          double yInput = MathUtil.applyDeadband(-controller.getLeftX(), 0.1);
          double rotInput = MathUtil.applyDeadband(-controller.getRightX(), 0.1);

          double xVelocity = xInput * maxSpeed;
          double yVelocity = yInput * maxSpeed;
          double angularVelocity = rotInput * maxAngularRate;

          swerve.setControl(
              fieldCentricRequest
                  .withVelocityX(xVelocity)
                  .withVelocityY(yVelocity)
                  .withRotationalRate(angularVelocity));
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
