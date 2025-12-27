// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.PWMConstants;
import frc.robot.util.Utils;

/**
 * Elevator subsystem using dual TalonFX motors in leader-follower configuration.
 * 
 * Features:
 * - Motion Magic for smooth profiled motion
 * - Soft limits for safety
 * - Gravity compensation (kG)
 * - Stall detection with current monitoring
 * - Named setpoint positions for easy use
 */
public class ElevatorSubsystem extends SubsystemBase {
  // Hardware
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;
  
  // Control requests (reused for efficiency)
  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0);
  private final NeutralOut neutralRequest = new NeutralOut();
  
  // State tracking
  private double targetPositionInches = 0;
  private double lastStallCheckTime = 0;
  
  /**
   * Creates a new ElevatorSubsystem
   */
  public ElevatorSubsystem() {
    // Initialize motors
    leaderMotor = new TalonFX(PWMConstants.LeftElevatorID);
    followerMotor = new TalonFX(PWMConstants.RightElevatorID);
    
    // Configure motors
    configureLeaderMotor();
    configureFollowerMotor();
    
    // Zero the elevator (assumes starting at bottom)
    resetPosition();
    
    // Initialize dashboard
    SmartDashboard.putData("Elevator", this);
    
    // Output initialization progress
    Utils.logInfo("Elevator subsystem initialized");
  }
  
  /**
   * Configures the leader (left) motor
   */
  private void configureLeaderMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // Motor output configuration
    config.MotorOutput
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withNeutralMode(NeutralModeValue.Brake);
    
    // Current limits - protect motors and mechanisms
    config.CurrentLimits
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(60.0) // Peak current threshold
      .withSupplyCurrentLowerLimit(40) // Continuous limit
      .withSupplyCurrentLowerTime(0.5) // Time at peak before limiting
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(80.0); // Thermal protection
    
    // Voltage limits
    config.Voltage
      .withPeakForwardVoltage(12.0)
      .withPeakReverseVoltage(-12.0);
    
    // Motion Magic configuration
    config.MotionMagic
      .withMotionMagicCruiseVelocity(ElevatorConstants.kCruiseVelocityRPS)
      .withMotionMagicAcceleration(ElevatorConstants.kAccelerationRPS2)
      .withMotionMagicJerk(ElevatorConstants.kJerkRPS3);
    
    // PID + Feedforward (Slot 0)
    config.Slot0
      .withKP(ElevatorConstants.kP)
      .withKI(ElevatorConstants.kI)
      .withKD(ElevatorConstants.kD)
      .withKG(ElevatorConstants.kG)  // Gravity compensation - CRITICAL for elevators
      .withKV(ElevatorConstants.kV)  // Velocity feedforward
      .withKS(ElevatorConstants.kS); // Static friction
    
    // Software limit switches - CRITICAL SAFETY FEATURE
    config.SoftwareLimitSwitch
      .withForwardSoftLimitEnable(true)
      .withForwardSoftLimitThreshold(inchesToRotations(ElevatorConstants.Positions.MAX))
      .withReverseSoftLimitEnable(true)
      .withReverseSoftLimitThreshold(0);
    
    // Apply configuration
    leaderMotor.getConfigurator().apply(config);

    // Set status frame periods (in Hz) to optimize CAN bus usage
    leaderMotor.getPosition().setUpdateFrequency(50);      // Position - 50 Hz (20ms)
    leaderMotor.getVelocity().setUpdateFrequency(50);      // Velocity - 50 Hz
    leaderMotor.getMotorVoltage().setUpdateFrequency(10);  // Voltage - 10 Hz (100ms)
    leaderMotor.getSupplyCurrent().setUpdateFrequency(10); // Current - 10 Hz
    leaderMotor.getDeviceTemp().setUpdateFrequency(4);     // Temperature - 4 Hz (250ms)
    
    // Optimize CAN bus utilization
    leaderMotor.optimizeBusUtilization();
  }
  
  /**
   * Configures the follower (right) motor
   */
  private void configureFollowerMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // Motor output - opposite inversion of leader for mechanical opposition
    config.MotorOutput
      .withInverted(InvertedValue.Clockwise_Positive)
      .withNeutralMode(NeutralModeValue.Brake);
    
    // Current limits (same as leader)
    config.CurrentLimits
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(60.0) // Peak current threshold
      .withSupplyCurrentLowerLimit(40) // Continuous limit
      .withSupplyCurrentLowerTime(0.5) // Time at peak before limiting
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(80.0);
    
    // Apply configuration
    followerMotor.getConfigurator().apply(config);
    
    // Set follower mode (opposeLeaderInvert = true for mechanical opposition)
    followerMotor.setControl(new Follower(PWMConstants.LeftElevatorID, true));
    
    // Set slower frame rates for follower
    followerMotor.getPosition().setUpdateFrequency(10);      // 10 Hz
    followerMotor.getVelocity().setUpdateFrequency(10);      // 10 Hz
    followerMotor.getMotorVoltage().setUpdateFrequency(4);   // 4 Hz
    followerMotor.getSupplyCurrent().setUpdateFrequency(4);  // 4 Hz
    followerMotor.getDeviceTemp().setUpdateFrequency(1);     // 1 Hz

    // Optimize CAN bus
    followerMotor.optimizeBusUtilization();
  }
  
  @Override
  public void periodic() {
    // Check for stall conditions periodically (every 0.5 seconds)
    double currentTime = Timer.getFPGATimestamp();
    if (currentTime - lastStallCheckTime > 0.5) {
      if (isStalling()) {
        Utils.logInfo("Elevator stalling! Current: " + leaderMotor.getSupplyCurrent().getValueAsDouble() + "A");
      }
      lastStallCheckTime = currentTime;
    }
  }
  
  // ============================================================
  // Position Control Methods
  // ============================================================
  
  /**
   * Sets the elevator to a target position in inches
   * @param inches Target height in inches from the ground
   */
  public void setPositionInches(double inches) {
    double clampedInches = MathUtil.clamp(inches, ElevatorConstants.Positions.GROUND, ElevatorConstants.Positions.MAX);
    targetPositionInches = clampedInches;
    double rotations = inchesToRotations(clampedInches);
    leaderMotor.setControl(mmRequest.withPosition(rotations));
  }
  
  /**
   * Sets the elevator to a named position
   * @param position One of the Positions constants
   */
  public void setPosition(double position) {
    setPositionInches(position);
  }
  
  /**
   * Stops the elevator (applies neutral output)
   */
  public void stop() {
    leaderMotor.setControl(neutralRequest);
  }
  
  /**
   * Resets the elevator position to zero
   * Should be called when elevator is at the bottom
   */
  public void resetPosition() {
    leaderMotor.setPosition(0);
    followerMotor.setPosition(0);
    targetPositionInches = 0;
    Utils.logInfo("Elevator position reset to zero");
  }
  
  // ============================================================
  // State Query Methods
  // ============================================================
  
  /**
   * Gets the current elevator position in inches
   * @return Current position in inches
   */
  public double getPositionInches() {
    return rotationsToInches(leaderMotor.getPosition().getValueAsDouble());
  }
  
  /**
   * Gets the current elevator position in rotations
   * @return Current position in motor rotations
   */
  public double getPositionRotations() {
    return leaderMotor.getPosition().getValueAsDouble();
  }
  
  /**
   * Gets the target position in inches
   * @return Target position in inches
   */
  public double getTargetPositionInches() {
    return targetPositionInches;
  }
  
  /**
   * Gets the current velocity in inches per second
   * @return Velocity in inches/second
   */
  public double getVelocityInchesPerSecond() {
    double rps = leaderMotor.getVelocity().getValueAsDouble();
    return rps * ElevatorConstants.kInchesPerRotation;
  }
  
  /**
   * Checks if the elevator is at the target position
   * @return True if within tolerance
   */
  public boolean atTarget() {
    double error = Math.abs(getPositionInches() - targetPositionInches);
    return error < ElevatorConstants.kPositionToleranceInches;
  }
  
  /**
   * Checks if the elevator is stalling
   * A stall is detected when:
   * 1. Not at target position
   * 2. Velocity is near zero
   * 3. Current draw is high
   * @return True if stalling
   */
  public boolean isStalling() {
    // Skip if at target (expected to have zero velocity)
    if (atTarget()) return false;
    
    double velocity = Math.abs(leaderMotor.getVelocity().getValueAsDouble());
    double current = leaderMotor.getSupplyCurrent().getValueAsDouble();
    
    return velocity < ElevatorConstants.kStallVelocityThreshold && 
           current > ElevatorConstants.kStallCurrentThreshold;
  }
  
  /**
   * Gets the current supply current draw
   * @return Current in amps
   */
  public double getCurrentAmps() {
    return leaderMotor.getSupplyCurrent().getValueAsDouble();
  }
  
  /**
   * Gets the closed loop error in inches
   * @return Error in inches
   */
  public double getErrorInches() {
    return rotationsToInches(leaderMotor.getClosedLoopError().getValueAsDouble());
  }
  
  // ============================================================
  // Unit Conversion Methods
  // ============================================================
  
  /**
   * Converts inches to motor rotations
   * @param inches Height in inches
   * @return Motor rotations
   */
  private double inchesToRotations(double inches) {
    return inches / ElevatorConstants.kInchesPerRotation;
  }
  
  /**
   * Converts motor rotations to inches
   * @param rotations Motor rotations
   * @return Height in inches
   */
  private double rotationsToInches(double rotations) {
    return rotations * ElevatorConstants.kInchesPerRotation;
  }
  
  // ============================================================
  // Command Factory Methods
  // ============================================================
  
  /**
   * Creates a command to move to a specific position
   * @param inches Target position in inches
   * @return Command that moves to position and ends when reached
   */
  public Command moveToPositionCommand(double inches) {
    return runOnce(() -> setPositionInches(inches))
      .andThen(run(() -> {}).until(this::atTarget))
      .withName("MoveElevatorTo_" + inches);
  }
  
  /**
   * Creates a command to move to ground position
   */
  public Command moveToGroundCommand() {
    return moveToPositionCommand(ElevatorConstants.Positions.GROUND).withName("ElevatorGround");
  }
  
  /**
   * Creates a command to move to coral level 1 position
   */
  public Command moveToCoralOneCommand() {
    return moveToPositionCommand(ElevatorConstants.Positions.CORAL1).withName("ElevatorCoralOne");
  }
  
  /**
   * Creates a command to move to coral level 2 position
   */
  public Command moveToCoralTwoCommand() {
    return moveToPositionCommand(ElevatorConstants.Positions.CORAL2).withName("ElevatorCoralTwo");
  }
  
  /**
   * Creates a command to move to coral level 3 position
   */
  public Command moveToCoralThreeCommand() {
    return moveToPositionCommand(ElevatorConstants.Positions.CORAL3).withName("ElevatorCoralThree");
  }
  
  /**
   * Creates a command to move to coral level 4 position
   */
  public Command moveToCoralFourCommand() {
    return moveToPositionCommand(ElevatorConstants.Positions.CORAL4).withName("ElevatorCoralFour");
  }
  
  /**
   * Creates a command to move to algae level 1 position
   */
  public Command moveToAlgaeOneCommand() {
    return moveToPositionCommand(ElevatorConstants.Positions.ALGAE1).withName("ElevatorAlgaeOne");
  }
  
  /**
   * Creates a command to move to algae level 2 position
   */
  public Command moveToAlgaeTwoCommand() {
    return moveToPositionCommand(ElevatorConstants.Positions.ALGAE2).withName("ElevatorAlgaeTwo");
  }
  
  /**
   * Creates a command to move to algae level 3 position
   */
  public Command moveToAlgaeThreeCommand() {
    return moveToPositionCommand(ElevatorConstants.Positions.ALGAE3).withName("ElevatorAlgaeThree");
  }
  
  /**
   * Creates a command to reset the elevator position
   * Use when elevator is manually positioned at bottom
   */
  public Command resetPositionCommand() {
    return runOnce(this::resetPosition).withName("ResetElevatorPosition");
  }
  
  // ============================================================
  // Motor Control Methods (for testing/tuning)
  // ============================================================
  
  /**
   * Sets motor brake mode
   * @param brake True for brake, false for coast
   */
  public void setBrakeMode(boolean brake) {
    NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;    
    leaderMotor.setNeutralMode(mode);
    followerMotor.setNeutralMode(mode);
  }
  
  // ============================================================
  // Dashboard/Telemetry
  // ============================================================
  
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Position (in)", this::getPositionInches, null);
    builder.addDoubleProperty("Target (in)", this::getTargetPositionInches, null);
    builder.addDoubleProperty("Error (in)", this::getErrorInches, null);
    builder.addDoubleProperty("Velocity (in/s)", this::getVelocityInchesPerSecond, null);
    builder.addDoubleProperty("Current (A)", this::getCurrentAmps, null);
    builder.addBooleanProperty("At Target", this::atTarget, null);
    builder.addBooleanProperty("Stalling", this::isStalling, null);
  }
}
