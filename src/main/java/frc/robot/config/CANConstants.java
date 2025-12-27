// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class CANConstants {
  public static final int kFrontLeftDriveID       = 11;
  public static final int kFrontRightDriveID      = 12;
  public static final int kBackLeftDriveID        = 14;
  public static final int kBackRightDriveID       = 13;

  public static final int kFrontLeftSteerID       = 2;
  public static final int kFrontRightSteerID      = 8;
  public static final int kBackLeftSteerID        = 4;
  public static final int kBackRightSteerID       = 6;

  public static final int kFrontLeftAbsEncoderID  = 0;
  public static final int kFrontRightAbsEncoderID = 1;
  public static final int kBackLeftAbsEncoderID   = 3;
  public static final int kBackRightAbsEncoderID  = 2;
}
