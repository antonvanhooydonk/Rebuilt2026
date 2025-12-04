// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

/**
 * Constants for the elevator subsystem
 */
public final class ElevatorConstants {
  /**
   * Named positions for the elevator (in inches)
   */
  public final class Positions {
    public static final double GROUND = 0.0;
    public static final double CORAL1 = 1.9685;
    public static final double CORAL2 = 9.25197;
    public static final double CORAL3 = 21.6535;
    public static final double CORAL4 = 44.09449;
    public static final double ALGAE1 = 12.9921;
    public static final double ALGAE2 = 25.9843;
    public static final double ALGAE3 = 42.51969;
    public static final double MAX = 45.5; // Absolute maximum
  }

  // ============================================================
  // Mechanical Constants
  // ============================================================
  
  /**
   * Gear ratio from motor to output
   * Example: 20:1 means motor rotates 20 times for 1 output rotation
   */
  public static final double kGearRatio = 20.0;
  
  /**
   * Diameter of the spool/drum that the cable wraps around (inches)
   * Measure this carefully!
   */
  public static final double kSpoolDiameterInches = 2.0;
  
  /**
   * Spool circumference in inches
   */
  public static final double kSpoolCircumferenceInches = Math.PI * kSpoolDiameterInches;
  
  /**
   * How many inches the elevator moves per motor rotation
   * Formula: (spool circumference / gear ratio)
   * 
   * Example: 2" diameter spool, 20:1 gearing
   * = (2π / 20) = 0.314 inches per motor rotation
   */
  public static final double kInchesPerRotation = kSpoolCircumferenceInches / kGearRatio;
  
  // ============================================================
  // Motion Magic Parameters
  // ============================================================
  
  /**
   * Maximum velocity in rotations per second
   * Start conservative (e.g. 10 RPS) and increase if needed
   * 
   * To calculate from real units:
   * Desired speed = 20 inches/sec
   * = 20 / kInchesPerRotation = 20 / 0.314 = 63.7 RPS
   */
  public static final double kCruiseVelocityRPS = 10.0;
  
  /**
   * Maximum acceleration in rotations per second²
   * Typically 2-4x the cruise velocity for smooth motion
   */
  public static final double kAccelerationRPS2 = 30.0;
  
  /**
   * Maximum jerk in rotations per second³
   * Typically 10x acceleration for smooth motion
   * Higher values = snappier but less smooth
   */
  public static final double kJerkRPS3 = 300.0;
  
  // ============================================================
  // PID + Feedforward Gains
  // ============================================================
  
  /**
   * Proportional gain
   * Start at 1.0 and tune:
   * - Too low: slow response, never reaches target
   * - Too high: oscillation, overshoot
   */
  public static final double kP = 3.0;
  
  /**
   * Integral gain
   * Usually not needed for elevators with good kG tuning
   * Only add if there's steady-state error
   */
  public static final double kI = 0.0;
  
  /**
   * Derivative gain
   * Helps dampen oscillations
   * Start at 0, add only if needed
   */
  public static final double kD = 0.0;
  
  /**
   * Gravity feedforward (CRITICAL for elevators!)
   * This is the voltage needed to hold the elevator at any position
   * 
   * Tuning process:
   * 1. Set kP=0, kG=0, manually move elevator to mid-height
   * 2. Release and measure how fast it falls
   * 3. Increase kG until it holds position (start at 0.1)
   * 4. Fine-tune: if it drifts down, increase kG; if drifts up, decrease kG
   * 
   * Typical range: 0.1 to 0.5 depending on mass and gearing
   */
  public static final double kG = 0.2;
  
  /**
   * Velocity feedforward
   * Voltage per (rotation/sec) of velocity
   * Can be calculated with SysId or manually tuned
   */
  public static final double kV = 0.12;
  
  /**
   * Static friction feedforward
   * Voltage to overcome static friction
   * Usually 0.01 to 0.1
   */
  public static final double kS = 0.05;
  
  // ============================================================
  // Tolerances and Thresholds
  // ============================================================
  
  /**
   * How close to target before considering "at position" (inches)
   */
  public static final double kPositionToleranceInches = 0.25;
  
  /**
   * Velocity threshold for stall detection (rotations/sec)
   * If velocity is below this, check if stalling
   */
  public static final double kStallVelocityThreshold = 0.5;
  
  /**
   * Current threshold for stall detection (amps)
   * If current is above this while velocity is low, consider it a stall
   */
  public static final double kStallCurrentThreshold = 50.0;
  
  // ============================================================
  // Tuning Guide
  // ============================================================
  
  /*
   * STEP-BY-STEP TUNING PROCESS:
   * 
   * 1. VERIFY MECHANICAL MEASUREMENTS
   *    - Measure spool diameter accurately
   *    - Confirm gear ratio from motor to spool
   *    - Calculate kInchesPerRotation
   * 
   * 2. TUNE GRAVITY FEEDFORWARD (kG) - MOST IMPORTANT
   *    - Set kP=0, kI=0, kD=0, kG=0
   *    - Move elevator to mid-height manually
   *    - Release and observe if it falls
   *    - Increase kG by 0.05 increments until it holds position
   *    - Fine-tune until it doesn't drift up or down
   * 
   * 3. TUNE MOTION MAGIC PROFILE
   *    - Start with conservative values (low velocity/accel)
   *    - Test motion and gradually increase until satisfactory
   *    - Ensure motion is smooth without jerking
   * 
   * 4. TUNE PROPORTIONAL GAIN (kP)
   *    - Start at 1.0
   *    - If too slow to reach target, increase kP
   *    - If oscillating, decrease kP
   *    - Goal: reaches target quickly without overshoot
   * 
   * 5. ADD DERIVATIVE (kD) IF NEEDED
   *    - Only if there's oscillation that kP adjustment can't fix
   *    - Start small (0.1) and increase gradually
   * 
   * 6. VERIFY SOFT LIMITS
   *    - Command positions beyond limits
   *    - Ensure soft limits prevent motion
   *    - Adjust limits if needed for safety margin
   * 
   * 7. TEST STALL DETECTION
   *    - Manually obstruct elevator
   *    - Verify stall is detected and logged
   *    - Adjust thresholds if needed
   */
}
