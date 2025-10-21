// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private ElevatorSim elevatorSim =
      new ElevatorSim(DCMotor.getFalcon500Foc(2), 3, 20, 0.0125, 0, 2, true, 0);

  private double targetHeight = 0;

  public ElevatorSubsystem() {}

  @Override
  public void periodic() {
    double currentHeight = getHeight();
    /* Bang-bang control */
    // if (currentHeight > targetHeight) {
    //  elevatorSim.setInputVoltage(-3);
    // } else if (currentHeight < targetHeight) {
    //   elevatorSim.setInputVoltage(3);
    // } else {
    //   elevatorSim.setInputVoltage(0);
    // }

    /* Proportional control */
    double error = targetHeight - currentHeight;
    double kP = 50;
    double voltage = kP * error;
    elevatorSim.setInputVoltage(voltage);

    /* Trapezoidal Profile */

    /* Velocity Feedforward */

    /*
     * Log
     */
    DogLog.log("Elevator/Height (Meters)", getHeight());
    DogLog.log("Elevator/Target Height (Meters)", targetHeight);

    elevatorSim.update(.020);
    updateVisualizer();
  }

  public double getHeight() {
    return elevatorSim.getPositionMeters();
  }

  public Command runVoltage(double volt) {
    return this.run(() -> elevatorSim.setInputVoltage(volt))
        .finallyDo(() -> elevatorSim.setInputVoltage(0));
  }

  public Command runPosition(double targetHeight) {
    return this.runOnce(
        () -> {
          this.targetHeight = targetHeight;
        });
  }

  private void updateVisualizer() {
    Pose3d elevatorPosition = new Pose3d(0, 0, getHeight(), new Rotation3d(0, 0, 0));
    Pose3d armPosition = new Pose3d(0, 0, .994 + getHeight(), new Rotation3d(0, 0, 0));
    Pose3d groundIntakePosition = new Pose3d(-0.27, 0, 0.242, new Rotation3d(0, 0, 0));

    DogLog.log(
        "Robot Visualizer/Component Positions",
        new Pose3d[] {elevatorPosition, armPosition, groundIntakePosition});
  }
}
