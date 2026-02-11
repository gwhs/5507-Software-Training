// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private ElevatorSim elevatorSim =
      new ElevatorSim(0.12, 0.01, DCMotor.getFalcon500Foc(2), 0, 1.7, true, 0);

  private ProfiledPIDController pidController =
      new ProfiledPIDController(
          .008,
          0,
          0,
          new Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION));

  public ElevatorSubsystem() {}

  @Override
  public void periodic() {
    double pidOutput = pidController.calculate(getRotation());
    elevatorSim.setInputVoltage(pidOutput);

    elevatorSim.update(.020);
  }

  public double getRotation() {
    return ElevatorSubsystem.metersToRotations(elevatorSim.getPositionMeters());
  }

  public double getHeightMeters() {
    return elevatorSim.getPositionMeters();
  }

  public static double rotationsToMeters(double rotations) {
    return rotations
        / ElevatorConstants.GEAR_RATIO
        * (ElevatorConstants.SPROCKET_DIAMETER * Math.PI)
        * 1;
  }

  public static double metersToRotations(double meters) {
    return meters
        / (ElevatorConstants.SPROCKET_DIAMETER * Math.PI)
        * ElevatorConstants.GEAR_RATIO
        / 1;
  }

  public Command runHeight(double meters) {
    double clampedMeters = MathUtil.clamp(meters, 0, ElevatorConstants.TOP_METER);
    return this.runOnce(
            () -> {
              pidController.setGoal(metersToRotations(clampedMeters));
            })
        .andThen(
            Commands.waitUntil(
                () -> MathUtil.isNear(clampedMeters, rotationsToMeters(getRotation()), 0.5)));
  }
}
