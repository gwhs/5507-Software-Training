// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(swerve.defaultDrive(controller));

    /********************
     * Button Bindings
     ********************/
    // TODO: Press right trigger to shoot balls (end with semicolon ;)
    // vvvvv YOUR CODE GOES HERE vvvvv

    // ^^^^^                     ^^^^^

    // TODO: Release right trigger to stop shooting balls (end with semicolon ;)
    // vvvvv YOUR CODE GOES HERE vvvvv

    // ^^^^^                     ^^^^^

    // Example: 
    // Press left trigger to deploy ground intake
    // Release left trigger to stop spinning the ground intake
    controller.leftTrigger().whileTrue(deployGroundIntake());
    controller.leftTrigger().onFalse(stopGroundIntake());
  }

  /********************
   * Command Compositions: Helper methods to make code more readable
   ********************/

  public Command shootBall() {
    return Commands.parallel(
        // TODO: Run shooter at 40 rotations per second (end with comma ,)
        // vvvvv YOUR CODE GOES HERE vvvvv

        // ^^^^^                     ^^^^^

        // TODO: Run indexer at 10 volts
        // vvvvv YOUR CODE GOES HERE vvvvv

        // ^^^^^                     ^^^^^
        );
  }

  public Command stopShooting() {
    return Commands.parallel(
        // TODO: Stop shooter (add comma , at the end) (end with comma ,)
        // vvvvv YOUR CODE GOES HERE vvvvv

        // ^^^^^                     ^^^^^

        // TODO: Stop indexer
        // vvvvv YOUR CODE GOES HERE vvvvv

        // ^^^^^                     ^^^^^
        );
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
}
