// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500Foc(1),
          ArmConstants.ARM_GEAR_RATIO,
          0.1,
          1,
          Units.degreesToRadians(ArmConstants.ARM_LOWER_BOUND),
          Units.degreesToRadians(ArmConstants.ARM_UPPER_BOUND),
          false,
          Units.degreesToRadians(-90));

  private TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          ArmConstants.MAX_VELOCITY * 360, ArmConstants.MAX_ACCELERATION * 360);

  private ProfiledPIDController pidController = new ProfiledPIDController(.1, 0, 0, constraints);

  public ArmSubsystem() {
    pidController.setGoal(-90);
  }

  @Override
  public void periodic() {
    double pidOutput = pidController.calculate(getAngle());
    armSim.setInputVoltage(pidOutput);

    armSim.update(0.20);
  }

  public double getAngle() {
    return Units.radiansToDegrees(armSim.getAngleRads());
  }

  public Command runAngle(double angle) {
    double clampedAngle =
        MathUtil.clamp(angle, ArmConstants.ARM_LOWER_BOUND, ArmConstants.ARM_UPPER_BOUND);
    return this.runOnce(
            () -> {
              pidController.setGoal(clampedAngle);
            })
        .andThen(Commands.waitUntil(() -> MathUtil.isNear(clampedAngle, getAngle(), 5)));
  }
}
