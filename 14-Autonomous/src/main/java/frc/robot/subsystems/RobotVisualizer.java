// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.groundIntake.GroundIntakeSubsystem;

public class RobotVisualizer extends SubsystemBase {
  private final ElevatorSubsystem elevator;
  private final ArmSubsystem arm;
  private final GroundIntakeSubsystem groundIntake;

  public RobotVisualizer(
      RobotContainer robotContainer,
      ElevatorSubsystem elevator,
      ArmSubsystem arm,
      GroundIntakeSubsystem groundIntake) {
    this.elevator = elevator;
    this.arm = arm;
    this.groundIntake = groundIntake;
  }

  @Override
  public void periodic() {
    double elevatorHeight = elevator.getHeightMeters();
    double armAngle = arm.getAngle();
    double groundIntakeAngle = groundIntake.getPivotAngle();

    Pose3d elevatorPosition = new Pose3d(0, 0, elevatorHeight, new Rotation3d(0, 0, 0));
    Pose3d armPosition =
        new Pose3d(
            0.03,
            0,
            .97 + elevatorHeight,
            new Rotation3d(0, Units.degreesToRadians(armAngle + 90 + 24), 0));
    Pose3d groundIntakePosition =
        new Pose3d(
            -0.27, 0, 0.242, new Rotation3d(0, Units.degreesToRadians(groundIntakeAngle), 0));

    DogLog.log(
        "Robot Visualizer/Component Positions",
        new Pose3d[] {elevatorPosition, armPosition, groundIntakePosition});
  }
}
