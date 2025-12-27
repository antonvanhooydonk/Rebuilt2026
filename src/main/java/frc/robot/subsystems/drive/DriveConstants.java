// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

import frc.robot.Constants.AnalogConstants;
import frc.robot.Constants.CANConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class DriveConstants {
  // ------------------------------------------------------------
  // Maximum drive & turning speeds - adjust as necessary
  // ------------------------------------------------------------
  public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(14); // ensure <= kMaxDriveVelocityAt12VoltsMPS, max translation speed in meters per second, set 7 in loft, 10 - 14 at competition
  public static final double kMaxAccelMetersPerSecondSq = Units.feetToMeters(12); // max acceleration in meters per second squared (must be <= max speed)
  public static final double kMaxAngularSpeedRadsPerSecond = Units.rotationsToRadians(1.75); // 1.25 rotations per second (target 8-10 rad/s (2.5π - 3π))
  public static final double kMaxAngularAccelRadsPerSecondSq = Units.rotationsToRadians(1.3125); // set to 75% of angular speed
 
  // ------------------------------------------------------------
  // Driver joystick settings
  // ------------------------------------------------------------
  public static final double kSlewRateLimit = 3.0; // slew rate, start at 3.0, decrease if too twitchy, increase if too sluggish 
  public static final double kJoystickDeadband = 0.1; // maybe use 0.5?, joystick deadband

  // ------------------------------------------------------------
  // Physical constants - adjust these to robot each year
  // ------------------------------------------------------------
  public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
  public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;
  public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
  public static final double kDriveGearRatio = 6.75; // Drive gear ratio for MKi L2 (adjust for your setup)
  public static final double kSteerGearRatio = 21.4285714286; // Steering gear ratio (adjust for your setup)
  public static final double kWheelCOF = 1.19; // could try 1.0 to 1.3, coefficient of friction of wheel on carpet
  public static final double kRobotMassKg = Units.lbsToKilograms(134);
  public static final double kRobotMOI = 3.08607399254; // kg m^2, moment of inertia about center of robot
  public static final double kMaxDriveVelocityAt12VoltsMPS = Units.feetToMeters(15.5); // MK4i L2 Kraken non-FOC With 14t pinion (https://www.swervedrivespecialties.com/products/mk4i-swerve-module?variant=47316033798445)

  public static final double kDriveMotorCurrentLimit = 90.0; // The max current draw in amps of a swerve module drive motor
  public static final double kDriveMotorCurrentLowerLimit = 70.0; // The regular lower limit current draw in amps of a swerve module drive motor
  public static final double kDriveMotorCurrentLowerTime = 0.5; // The time in seconds after which the lower current limit applies
  public static final double kDriveMotorStatorCurrentLimit = 120.00; // The max current draw in amps of a swerve module drive motor stator
  public static final double kDriveMaxForwardVoltage = 12.0; // Max voltage to apply to drive motors when driving forward
  public static final double kDriveMaxReverseVoltage = -12.0; // Max voltage to apply to drive motors when driving backward
  
  public static final int    kSteerMotorMaxCurrent = 25; // The max steady state current draw in amps of a swerve module steer motor
  public static final double kSteerMotorMaxPeakCurrent = 35; // The max burst/peak current draw in amps of a swerve module steer motor
  
  public static final double kGyroXAngleOffsetDegrees = 0.0; // default 0.0 - Rotate the gyro X axis if the gyro was not installed facing forward
  public static final double kPeriodicTimeSeconds = 0.02; // 0.13; // 20ms (default) + 110ms => 0.02 + 0.11 spark max velocity lag 

  // Define the module translations from the robot's center
  // See: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#coordinate-system
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
      kMaxDriveVelocityAt12VoltsMPS,
      kWheelCOF, 
      DCMotor.getKrakenX60(1).withReduction(kDriveGearRatio),
      kDriveMotorCurrentLimit,
      1
    ),
    kSwerveModuleTranslations
  );
 
  // ------------------------------------------------------------
  // Drive PID constants (tune these each year for the robot)
  // ------------------------------------------------------------
  // Tuning Process:
  // 1. Characterize with SysId tool
  // 2. Start with feedforward only (kP = 0)
  // 3. Add minimal P gain if needed (usually 0.05 - 0.2)
  // 4. Avoid I and D unless absolutely necessary
  public static final double kDriveKP = 0.075;  // 0.0020645;
  public static final double kDriveKI = 0.0;
  public static final double kDriveKD = 0.0;
  public static final double kDriveKS = 0.1;  // Static friction
  public static final double kDriveKV = 2.7;  // Velocity feedforward
  public static final double kDriveKA = 0.3;  // Acceleration feedforward

  // ------------------------------------------------------------
  // Steering PID Constants (tune these each year for the robot)
  // ------------------------------------------------------------
  // Tuning Process:
  // 1. Start with P only (I=0, D=0)
  // 2. Increase P until fast response without overshoot (typically 1.0 - 3.0)
  // 3. Add small D if oscillating (typically 0.01-0.1)
  // 4. Avoid I unless steady-state error (very rare in steering)
  public static final double kSteerKP = 1.5; // 0.01
  public static final double kSteerKI = 0.0;
  public static final double kSteerKD = 0.0;  // 0.32
  public static final double kSteerFF = 0.0;  // 0.0 to 0.2, feedforward to help with static friction. Rev recommends 0

  // ------------------------------------------------------------
  // Define the swerve module constants
  // ------------------------------------------------------------
  // Front Left Module - Module 0
  public static final SwerveModule kFrontLeftSwerveModule = new SwerveModule(
    "FL",
    CANConstants.kFrontLeftDriveID,
    CANConstants.kFrontLeftSteerID,
    false,
    false,
    AnalogConstants.kFrontLeftAbsEncoderID,
    Units.degreesToRadians(7.8),
    false
  );

  // Front Right Module - Module 1
  public static final SwerveModule kFrontRightSwerveModule = new SwerveModule(
    "FR",
    CANConstants.kFrontRightDriveID,
    CANConstants.kFrontRightSteerID,
    false,
    false,
    AnalogConstants.kFrontRightAbsEncoderID,    
    Units.degreesToRadians(119.7),
    false
  );

  // Back Left Module - Module 2
  public static final SwerveModule kBackLeftSwerveModule = new SwerveModule(
    "BL",
    CANConstants.kBackLeftDriveID,
    CANConstants.kBackLeftSteerID,
    false,
    false,
    AnalogConstants.kBackLeftAbsEncoderID,
    Units.degreesToRadians(68.7),
    false
  );

  // Back Right Module - Module 3
  public static final SwerveModule kBackRightSwerveModule = new SwerveModule(
    "BR",
    CANConstants.kBackRightDriveID,
    CANConstants.kBackRightSteerID,
    false,
    false,
    AnalogConstants.kBackRightAbsEncoderID,
    Units.degreesToRadians(297.5),
    false
  );
}
