// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.Auto_Coral;
import frc.robot.subsystems.RobotVisualizer;
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

  private final RobotVisualizer robotVisualizer = new RobotVisualizer(elevator, arm, groundIntake);

  private final CommandXboxController controller = new CommandXboxController(0);
  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // TODO: Create the autonomous routine chooser

  public RobotContainer() {
    configureBindings();
    configureAutoChooser();

    SmartDashboard.putData("autonomous", autoChooser);
  }

  private void configureBindings() {
    swerve.setDefaultCommand(swerve.defaultDrive(controller));

    /********************
     *
     * Button Bindings
     ********************/
    controller.a().onTrue(elevator.runHeight(0.3));
    controller.b().onTrue(elevator.runHeight(.75));
    controller.x().onTrue(elevator.runHeight(0));
  }

  private void configureAutoChooser() {
    // TODO: Add your autonomous routine to the auto chooser
    autoChooser.addOption("Auto Coral", new Auto_Coral());
    // TODO: Published the autonomous routine chooser to SmartDashboard

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // TODO: return the command chosen instead
  }
}
