// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  private SingleJointedArmSim climbSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500Foc(1),
          ClimbConstants.CLIMB_GEAR_RATIO,
          0.01,
          0.1,
          Units.degreesToRadians(0),
          Units.degreesToRadians(1000),
          false,
          0);

  private TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          ClimbConstants.MAX_VELOCITY * 360 / 60, ClimbConstants.MAX_ACCELERATION * 360 / 60);
  private ProfiledPIDController pidController = new ProfiledPIDController(.1, 0, 0, constraints);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {}

  public double getPosition() {
    return Units.radiansToRotations(climbSim.getAngleRads());
  }

  @Override
  public void periodic() {
    double pidOutput = pidController.calculate(getPosition());
    climbSim.setInputVoltage(pidOutput);

    climbSim.update(0.20);
  }

  public Command runPosition(double position) {
    return this.runOnce(
            () -> {
              pidController.setGoal(position);
            })
        .andThen(Commands.waitUntil(() -> MathUtil.isNear(position, getPosition(), 0.1)));
  }
}
