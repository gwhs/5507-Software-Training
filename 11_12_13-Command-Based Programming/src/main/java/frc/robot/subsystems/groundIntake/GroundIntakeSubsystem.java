// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.groundIntake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntakeSubsystem extends SubsystemBase {
  private SingleJointedArmSim pivotMotorSim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500Foc(1),
          GroundIntakeConstants.PIVOT_GEAR_RATIO,
          0.1,
          1,
          Units.degreesToRadians(GroundIntakeConstants.GROUND_INTAKE_LOWER_BOUND),
          Units.degreesToRadians(GroundIntakeConstants.GROUND_INTAKE_UPPER_BOUND),
          false,
          Units.degreesToRadians(90));

  private TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          GroundIntakeConstants.MAX_VELOCITY * 360, GroundIntakeConstants.MAX_ACCELERATION * 360);
  private ProfiledPIDController pidController = new ProfiledPIDController(.1, 0, 0, constraints);

  private FlywheelSim spinMotorSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getFalcon500Foc(1), 0.0001, GroundIntakeConstants.SPIN_GEAR_RATIO),
          DCMotor.getFalcon500Foc(1));

  public GroundIntakeSubsystem() {}

  @Override
  public void periodic() {
    double pidOutput = pidController.calculate(getPivotAngle());
    pivotMotorSim.setInputVoltage(pidOutput);

    spinMotorSim.update(0.20);
    pivotMotorSim.update(0.20);
  }

  public double getPivotAngle() {
    return Units.radiansToDegrees(pivotMotorSim.getAngleRads());
  }

  public Command setAngleAndAmp(double pivotAngle, double amp, double dutyCycle) {
    return this.runOnce(
        () -> {
          pidController.setGoal(pivotAngle);
          double resistance = DCMotor.getFalcon500(1).rOhms;
          double voltage = amp * resistance;
          spinMotorSim.setInputVoltage(voltage);
        });
  }
}
