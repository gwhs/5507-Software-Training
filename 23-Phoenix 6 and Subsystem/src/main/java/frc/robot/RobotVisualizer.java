// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystem.arm.ArmSubsystem;
import frc.robot.subsystem.elevator.ElevatorSubsystem;

public class RobotVisualizer {
  private final ArmSubsystem arm;
  private final ElevatorSubsystem elevator;

  private final Mechanism2d panel = new Mechanism2d(1, 3);
  // Code for the stick figure of each subsystems
  MechanismRoot2d root = panel.getRoot("elevator", 0.5, 0.078);
  MechanismLigament2d m_elevator =
      root.append(
          new MechanismLigament2d("elevatorL", 1.5, 90, 10, new Color8Bit(Color.kFirstRed)));
  // arm
  MechanismLigament2d m_arm =
      m_elevator.append(
          new MechanismLigament2d(
              "arm", Units.inchesToMeters(20), 90, 10, new Color8Bit(Color.kWhite)));

  public RobotVisualizer(ArmSubsystem arm, ElevatorSubsystem elevator) {
    this.arm = arm;
    this.elevator = elevator;

    SmartDashboard.putData("RobotVisualizer", panel);
  }

  public void periodic() {
    double elevatorHeight = elevator.getHeightMeters();
    double armAngle = arm.getAngle();

    m_arm.setAngle(-armAngle + 90);

    m_elevator.setLength(elevatorHeight + Units.inchesToMeters(37.2));
  }
}
