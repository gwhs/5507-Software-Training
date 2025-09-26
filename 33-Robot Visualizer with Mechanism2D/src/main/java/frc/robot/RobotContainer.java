// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
import frc.robot.subsystems.groundIntake.GroundIntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.TunerConstants_Comp;

public class RobotContainer {
  private final SwerveSubsystem swerve = TunerConstants_Comp.createDrivetrain();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final GroundIntakeSubsystem groundIntake = new GroundIntakeSubsystem();
  private final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();

  private final CommandXboxController controller = new CommandXboxController(0);

  // TODO: Create the autonomous routine chooser

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(swerve.defaultDrive(controller));

    /********************
     * Button Bindings
     ********************/
    controller.a().onTrue(elevator.runHeight(0.3));
    controller.b().onTrue(elevator.runHeight(.75));

    controller.x().onTrue(arm.runAngle(90));
    controller.y().onTrue(arm.runAngle(-90));

    controller.leftBumper().onTrue(groundIntake.setAngleAndAmp(-100, 0, 0));
    controller.rightBumper().onTrue(groundIntake.setAngleAndAmp(0, 0, 0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void periodic() {}
}
