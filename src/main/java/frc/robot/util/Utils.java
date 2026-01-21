// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.text.DecimalFormat;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;

public final class Utils {
  /**
   * Log robot info message to console or other notification systems.
   */
  public static void logInfo(String msg) {
    System.out.println(msg);
  }

  /**
   * Log robot error message to console or other notification systems.
   */
  public static void logError(String msg) {
    System.err.println(msg);
    DriverStation.reportError(msg, false);
    Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Error", msg));
  }

  /**
   * Determine if the robot is currently on the red alliance.
   * @return True if the robot is on the red alliance, false otherwise.
   */
  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  /**
   * Format a double value to 4 decimal places.
   * @param value The double value to format.
   * @return The double value formatted to a standard number of decimal places.
   */
  public static double fmtDouble(double value) {
    DecimalFormat df = new DecimalFormat("0.0000");
    return Double.valueOf(df.format(value));
  }
}
