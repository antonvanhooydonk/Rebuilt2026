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
  public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(14); // max translational speed, ensure <= kMaxDriveVelocityAt12VoltsMPS (15.5), set 7 in loft, 10 - 14 at competition
  public static final double kMaxAccelMetersPerSecondSq = Units.feetToMeters(12); // max translational acceleration
  public static final double kMaxAngularSpeedRadsPerSecond = Units.rotationsToRadians(1.75); // max rotational speed, conservative: 1.0 - 1.5, competitive: 1.5 - 2.0, aggressive: 2.0 - 2.5
  public static final double kMaxAngularAccelRadsPerSecondSq = kMaxAngularSpeedRadsPerSecond * 1.0; // max rotational acceleration, 0.75 (conservative) to 1.5 (aggressive) times kMaxAngularSpeedRadsPerSecond

  // ------------------------------------------------------------
  // Driver joystick settings
  // ------------------------------------------------------------
  public static final double kTranslationalSlewRateLimit = 4.0; // m/s per second, start at 3.0, decrease if too twitchy, increase if too sluggish 
  public static final double kRotationalSlewRateLimit = kMaxAngularAccelRadsPerSecondSq * 1.0; // rad/s per second, 0.6 - 1.0 times kMaxAngularAccelRadsPerSecondSq
  public static final double kJoystickDeadband = 0.1; // typically 0.05 to 0.15

  // ============================================================
  // BELOW THIS LINE SHOULDN'T BE CHANGED AT COMPETITION
  // ============================================================

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
   
  public static final double kGyroXAngleOffsetDegrees = 0.0; // default 0.0 - Rotate the gyro X axis if the gyro was not installed facing forward
  public static final double kPeriodicTimeSeconds = 0.02; // 20ms (default)

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
      60.0,
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
  public static final double kDriveKV = 0.12; // Velocity feedforward, usually 0.11 to 0.13
  public static final double kDriveKA = 0.0;  // Acceleration feedforward, usually 0.0 to 0.15, adjust in 0.05 increments

  // ------------------------------------------------------------
  // Steering PID Constants (tune these each year for the robot)
  // ------------------------------------------------------------
  // Tuning Process:
  // 1. Start with P only (I=0, D=0)
  // 2. Increase P until fast response without overshoot (typically 1.0 - 3.0)
  // 3. Add small D if oscillating (typically 0.01 - 0.1)
  // 4. Avoid I unless steady-state error (very rare in steering)
  public static final double kSteerKP = 1.5;  // 0.01
  public static final double kSteerKI = 0.0;
  public static final double kSteerKD = 0.0;  // 0.32
  public static final double kSteerKS = 0.0;  // 0.0 to 0.2, FF for static friction. Rev recommends 0

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
