// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
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
  public static final double kDriveGearRatio = 1 / 6.75; // Drive gear ratio (adjust for your setup)
  public static final double kSteerGearRatio = 1 / 21.4285714286; // Steering gear ratio (adjust for your setup)
  public static final double kDriveRot2Meter = kDriveGearRatio * kWheelCircumference ; // Convert drive motor rotations to meters 
  public static final double kDriveMeter2Rot = 1 / kDriveRot2Meter; // Convert drive motor meters to rotations 
  public static final double kSteerRot2Rad = kSteerGearRatio * 2 * Math.PI ; // Convert steering motor rotations to radians
  public static final double kSteerRpm2RadPerSec = kSteerRot2Rad / 60.0; // Convert steering motor RPM to radians per second
  public static final double kWheelCOF = 1.19; // could try 1.0 to 1.3, coefficient of friction of wheel on carpet
  public static final double kMaxSpeedAt12VoltsMPS = Units.feetToMeters(15); // MK4i 16.5 ft/s L3 Kraken FOC With 14t pinion
  public static final double kRobotMassKg = Units.lbsToKilograms(134);
  public static final double kRobotMOI = 3.08607399254; // kg m^2, moment of inertia about center of robot
  public static final double kDriveMotorCurrentLimit = 60.0; // The max current draw in amps of a swerve module drive motor
  public static final double kDriveMotorCurrentLowerLimit = 40.0; // The lower limit current draw in amps of a swerve module drive motor
  public static final double kDriveMaxForwardVoltage = 12.0; // Max voltage to apply to drive motors when driving forward
  public static final double kDriveMaxReverseVoltage = -12.0; // Max voltage to apply to drive motors when driving backward
  public static final int    kSteerMotorMaxCurrent = 30; // The max current draw in amps of a swerve module steer motor
  public static final double kPeriodicTimeSeconds = 0.02; // 0.13; // 20ms (default) + 110ms => 0.02 + 0.11 spark max velocity lag 

  public static final double kGyroXAngleOffsetDegrees = 0.0; // default 0.0 - offset if gyro X axis does not point forward

  // Maximum speeds - adjust based on your robot capabilities
  public static final double kJoystickDeadband = 0.1; // maybe use 0.5?, joystick deadband
  public static final double kSlewRateLimit = 3.0; // joystick slew rate, start at 3.0, decrease if too twitchy, increase if too sluggish 
  public static final boolean kAntiJitterEnabled = true; // set to true to stop jittering from module noise
  public static final double kAntiJitterSpeedDeadband = 0.001; // 1mm in m/s
  public static final double kAntiJitterAngleDeadband = Units.degreesToRadians(2.0); // radians
  public static final double kAntiJitterMinTurningSpeed = 0.1; // m/s
  public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(13); // max translation speed in meters per second, 12-16 ft/s is reasonable
  public static final double kMaxAccelMetersPerSecondSq = 5.5; // 4 to 6 is a reasonable range
  public static final double kMaxAngularSpeedRadsPerSecond = Units.degreesToRadians(540);
  public static final double kMaxAngularAccelRadsPerSecondSq = Units.degreesToRadians(720); // 0.5 is a conservative default
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
  public static final SwerveModule kFrontLeftSwerveModule = new SwerveModule(
    "FL",
    11,
    2,
    false,
    false,
    0,
    Units.degreesToRadians(7.8),
    false
  );

  // Front Right Module - Module 1
  public static final SwerveModule kFrontRightSwerveModule = new SwerveModule(
    "FR",
    12,
    8,
    false,
    false,
    1,
    Units.degreesToRadians(119.7),
    false
  );

  // Rear Left Module - Module 2
  public static final SwerveModule kBackLeftSwerveModule = new SwerveModule(
    "BL",
    14,
    4,
    false,
    false,
    3,
    Units.degreesToRadians(68.7),
    false
  );

  // Rear Right Module - Module 3
  public static final SwerveModule kBackRightSwerveModule = new SwerveModule(
    "BR",
    13,
    6,
    false,
    false,
    2,
    Units.degreesToRadians(297.5),
    false
  );

  // Define the module translations from the robot's center
  public static final Translation2d[] kSwerveModuleTranslations = new Translation2d[] {
    new Translation2d(Units.inchesToMeters(11.5), Units.inchesToMeters(11.5)), // FL
    new Translation2d(Units.inchesToMeters(11.5), Units.inchesToMeters(-11.5)),       // FR
    new Translation2d(Units.inchesToMeters(-11.5), Units.inchesToMeters(11.5)),       // BL
    new Translation2d(Units.inchesToMeters(-11.5), Units.inchesToMeters(-11.5))              // BR
  };

  // ------------------------------------------------------------
  // Autobuilder RobotConfig for PathPlanner
  // ------------------------------------------------------------
  public static final RobotConfig kRobotConfig = new RobotConfig(
    kRobotMassKg,
    kRobotMOI,
    new ModuleConfig(
      kWheelRadiusMeters,
      kMaxSpeedAt12VoltsMPS,
      kWheelCOF, 
      DCMotor.getKrakenX60(1).withReduction(1 / kDriveGearRatio), 
      kDriveMotorCurrentLimit,
      1
    ),
    kSwerveModuleTranslations
  );
}
