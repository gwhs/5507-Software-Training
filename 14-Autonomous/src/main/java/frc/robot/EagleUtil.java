// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class EagleUtil {
  public static Pose2d getClosestCoralStation(Pose2d robotPose) {
    return robotPose.nearest(FieldConstants.CORAL_STATION_POSE);
  }
}
