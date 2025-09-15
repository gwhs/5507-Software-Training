// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
  public static double BLUE_REEF_X = Units.inchesToMeters(144 + (93.5 - 14 * 2) / 2);
  public static double BLUE_REEF_Y = Units.inchesToMeters(158.50);
  public static Translation2d BLUE_REEF = new Translation2d(BLUE_REEF_X, BLUE_REEF_Y);

  public static double RED_REEF_X = Units.inchesToMeters(546.875 - (93.5 - 14 * 2) / 2);
  public static double RED_REEF_Y = Units.inchesToMeters(158.50);
  public static Translation2d RED_REEF = new Translation2d(RED_REEF_X, RED_REEF_Y);
}
