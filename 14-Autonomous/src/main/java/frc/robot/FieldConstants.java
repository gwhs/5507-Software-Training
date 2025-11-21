// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class FieldConstants {
  public static final List<Pose2d> CORAL_STATION_POSE =
      new ArrayList<>(
          Arrays.asList(
              new Pose2d(16.05, 7.42, Rotation2d.fromDegrees(-126)), // Red Processor Side
              new Pose2d(16.05, 0.63, Rotation2d.fromDegrees(126)), // Red Non-Processor Side
              new Pose2d(1.5, 0.63, Rotation2d.fromDegrees(54)), // Blue Processor Side
              new Pose2d(1.5, 7.42, Rotation2d.fromDegrees(-54)) // Blue Non-Processor Side
              ));
}
