// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.autonomous.BasicAuton;
import frc.robot.subsystems.groundIntakePivot.GroundIntakePivotSubsystem;
import frc.robot.subsystems.groundIntakeRoller.GroundIntakeRollerSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants_mk5n;

public class RobotContainer {
  private final SwerveSubsystem swerve = TunerConstants_mk5n.createDrivetrain();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IndexerSubsystem indexer = new IndexerSubsystem();
  private final GroundIntakePivotSubsystem groundIntakePivot = new GroundIntakePivotSubsystem();
  private final GroundIntakeRollerSubsystem groundIntakeRoller = new GroundIntakeRollerSubsystem();

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private final CommandXboxController controller = new CommandXboxController(0);

  private final DriveCommand swerveDriveCommand = new DriveCommand(controller, swerve);

  public RobotContainer() {
    configureBindings();

    autoChooser.setDefaultOption("Basic Auton", new BasicAuton(shooter, indexer, groundIntakePivot, groundIntakeRoller));
  }

  public void periodic() {
  }

  private void configureBindings() {
    swerve.setDefaultCommand(swerveDriveCommand);

    controller.leftTrigger().whileTrue(deployGroundIntake());
    controller.leftTrigger().onFalse(stopGroundIntake());
  }

  public Command deployGroundIntake() {
    return Commands.parallel(
        groundIntakeRoller.startIntake(), 
        groundIntakePivot.deploy()
      );
  }

  public Command stopGroundIntake() {
    return groundIntakeRoller.stopIntake();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
