// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class DriveConstants {
  // Physical constants - adjust these for your robot
  public static final double kTrackWidth = Units.inchesToMeters(21.73); // Distance between left and right wheels
  public static final double kWheelBase = Units.inchesToMeters(21.73);  // Distance between front and rear wheels
  public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
  public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;
  public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
  public static final double kWheelCOF = 1.19; // could try 1.0 to 1.3, coefficient of friction of wheel on carpet
  public static final double kRobotMassKg = Units.lbsToKilograms(134);
  public static final double kRobotMOI = 6.884; // kg m^2, moment of inertia about center of robot
  public static final double kDriveMotorCurrentLimit = 60.0; // The max current draw in amps of a swerve module drive motor
  public static final double kDriveMotorCurrentLowerLimit = 40.0; // The lower limit current draw in amps of a swerve module drive motor
  public static final double kDriveMaxForwardVoltage = 12.0; // Max voltage to apply to drive motors when driving forward
  public static final double kDriveMaxReverseVoltage = -12.0; // Max voltage to apply to drive motors when driving backward
  public static final int    kSteerMotorMaxCurrent = 30; // The max current draw in amps of a swerve module steer motor
  public static final double kDriveGearRatio = 6.75; // Drive gear ratio (adjust for your setup)
  public static final double kSteerGearRatio = 21.4285714286; // Steering gear ratio (adjust for your setup)
  public static final double kDriveRPStoMPS = kWheelCircumference / kDriveGearRatio; // Convert drive motor rotations per second to meters per second
  public static final double kDriveMPStoRPS = kDriveGearRatio / kWheelCircumference; // Convert drive motor meters per second to rotations per second 
  public static final double kPeriodicTimeSeconds = 0.02; // 0.13; // 20ms (default) + 110ms => 0.02 + 0.11 spark max velocity lag 

  // Maximum speeds - adjust based on your robot capabilities
  public static final double kJoystickDeadband = 0.1; // maybe use 0.5?, joystick deadband
  public static final double kSlewRateLimit = 1.25; // joystick slew rate, 1/kSlewRateLimit = time to get to full sped
  public static final double kSlowModeScaler = 0.3; // scales speeds down in slow mode, 0.0 to 1.0
  public static final boolean kAntiJitter = false; // set to true to stop jittering from module noise
  public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(13); // max translation speed in meters per second, 12-16 ft/s is reasonable
  public static final double kMaxAccelMetersPerSecondSq = 4.5; // 4 to 6 is a reasonable range
  public static final double kMaxAngularSpeedRadiansPerSecond = Units.degreesToRadians(540);
  public static final double kMaxAngularAccelRadiansPerSecondSq = Units.degreesToRadians(720); // 0.5 is a conservative default
  public static final double kWheelLockTimeSeconds = 10.0; // seconds to lock wheels when robot is disabled

  // PID Constants for drive (tune these each year for the robot)
  // Tuning Process:
  // 1. Characterize with SysId tool
  // 2. Start with feedforward only (kP = 0)
  // 3. Add minimal P gain if needed (usually 0.05 - 0.2)
  // 4. Avoid I and D unless absolutely necessary
  public static final double kDriveKP = 0.0020645;
  public static final double kDriveKI = 0.0;
  public static final double kDriveKD = 0.0;
  public static final double kDriveKS = 0.1;  // Static friction
  public static final double kDriveKV = 2.7;  // Velocity feedforward
  public static final double kDriveKA = 0.3;  // Acceleration feedforward

  // PID Constants for steering (tune these each year for the robot)
  // Tuning Process:
  // 1. Start with P only (I=0, D=0)
  // 2. Increase P until fast response without overshoot
  // 3. Add small D if oscillating (typically 0.01-0.1)
  // 4. Avoid I unless steady-state error (very rare in steering)
  public static final double kSteerKP = 0.5; // 0.01
  public static final double kSteerKI = 0.0;
  public static final double kSteerKD = 0.0;  // 0.32
  public static final double kSteerFF = 0.0;  // 0.0 to 0.2, feedforward to help with static friction. Rev recommends 0

  // ------------------------------------------------------------
  // Define the swerve module constants
  // ------------------------------------------------------------
  // Front Left Module - Module 0
  public static final int kFrontLeftDriveMotorID = 11;
  public static final boolean kFrontLeftDriveMotorInverted = false;
  public static final int kFrontLeftSteerMotorID = 2;
  public static final boolean kFrontLeftSteerMotorInverted = false;
  public static final int kFrontLeftAbsoluteEncoderID = 0;
  public static final double kFrontLeftAbsoluteEncoderOffsetRadians = Units.degreesToRadians(7.8);
  public static final boolean kFrontLeftAbsoluteEncoderInverted = false;
  public static final Translation2d kFrontLeftLocation = new Translation2d(Units.inchesToMeters(11.5), Units.inchesToMeters(11.5));

  // Front Right Module - Module 1
  public static final int kFrontRightDriveMotorID = 12;
  public static final boolean kFrontRightDriveMotorInverted = false;
  public static final int kFrontRightSteerMotorID = 8;
  public static final boolean kFrontRightSteerMotorInverted = false;
  public static final int kFrontRightAbsoluteEncoderID = 1;
  public static final double kFrontRightAbsoluteEncoderOffsetRadians = Units.degreesToRadians(119.7);
  public static final boolean kFrontRightAbsoluteEncoderInverted = false;
  public static final Translation2d kFrontRightLocation = new Translation2d(Units.inchesToMeters(11.5), Units.inchesToMeters(-11.5));

  // Rear Left Module - Module 2
  public static final int kBackLeftDriveMotorID = 14;
  public static final boolean kBackLeftDriveMotorInverted = false;
  public static final int kBackLeftSteerMotorID = 4;
  public static final boolean kBackLeftSteerMotorInverted = false;
  public static final int kBackLeftAbsoluteEncoderID = 3;
  public static final double kBackLeftAbsoluteEncoderOffsetRadians = Units.degreesToRadians(68.7);
  public static final boolean kBackLeftAbsoluteEncoderInverted = false;
  public static final Translation2d kBackLeftLocation = new Translation2d(Units.inchesToMeters(-11.5), Units.inchesToMeters(11.5));

  // Rear Right Module - Module 3
  public static final int kBackRightDriveMotorID = 13;
  public static final boolean kBackRightDriveMotorInverted = false;
  public static final int kBackRightSteerMotorID = 6;
  public static final boolean kBackRightSteerMotorInverted = false;
  public static final int kBackRightAbsoluteEncoderID = 2;
  public static final double kBackRightAbsoluteEncoderOffsetRadians = Units.degreesToRadians(297.5);
  public static final boolean kBackRightAbsoluteEncoderInverted = false;
  public static final Translation2d kBackRightLocation = new Translation2d(Units.inchesToMeters(-11.5), Units.inchesToMeters(-11.5));
}
