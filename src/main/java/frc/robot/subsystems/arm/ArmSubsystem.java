// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.util.Utils;

/**
 * Arm subsystem using dual TalonFX motors in leader-follower configuration.
 * 
 * Features:
 * - Motion Magic for smooth profiled motion
 * - Soft limits for safety
 * - Gravity compensation (kG)
 * - Stall detection with current monitoring
 * - Named setpoint positions for easy use
 */
public class ArmSubsystem extends SubsystemBase {
  // Hardware
  private final TalonFX angleMotor;
  
  // Control requests (reused for efficiency)
  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0).withSlot(0);
  private final NeutralOut neutralRequest = new NeutralOut();
  
  // State tracking
  private double targetPositionDegrees = 0;
  private double lastStallCheckTime = 0;
  
  /**
   * Creates a new ArmSubsystem
   */
  public ArmSubsystem() {
    // Initialize motors
    angleMotor = new TalonFX(CANConstants.ArmID);
    
    // Configure motors
    configureAngleMotor();
    
    // Zero the arm (assumes starting at home position)
    resetPosition();
    
    // Initialize dashboard
    SmartDashboard.putData("Arm", this);
    
    // Output initialization progress
    Utils.logInfo("Arm subsystem initialized");
  }
  
  /**
   * Configures the angle adjuster motor
   */
  private void configureAngleMotor() {
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
      .withMotionMagicCruiseVelocity(ArmConstants.kCruiseVelocityRPS)
      .withMotionMagicAcceleration(ArmConstants.kAccelerationRPS2)
      .withMotionMagicJerk(ArmConstants.kJerkRPS3);
    
    // PID + Feedforward (Slot 0)
    config.Slot0
      .withKP(ArmConstants.kP)
      .withKI(ArmConstants.kI)
      .withKD(ArmConstants.kD)
      .withKG(ArmConstants.kG)  // Gravity compensation - CRITICAL for arms
      .withKV(ArmConstants.kV)  // Velocity feedforward
      .withKS(ArmConstants.kS); // Static friction
    
    // Software limit switches - CRITICAL SAFETY FEATURE
    config.SoftwareLimitSwitch
      .withForwardSoftLimitEnable(true)
      .withForwardSoftLimitThreshold(degreesToRotations(ArmConstants.Positions.MAX))
      .withReverseSoftLimitEnable(true)
      .withReverseSoftLimitThreshold(0);
    
    // Apply configuration
    angleMotor.getConfigurator().apply(config);
    
    // Set status frame periods (in Hz) to optimize CAN bus usage
    angleMotor.getPosition().setUpdateFrequency(50);      // Position - 50 Hz (20ms)
    angleMotor.getVelocity().setUpdateFrequency(50);      // Velocity - 50 Hz
    angleMotor.getMotorVoltage().setUpdateFrequency(10);  // Voltage - 10 Hz (100ms)
    angleMotor.getSupplyCurrent().setUpdateFrequency(10); // Current - 10 Hz
    angleMotor.getDeviceTemp().setUpdateFrequency(4);     // Temperature - 4 Hz (250ms)

    // Optimize CAN bus utilization
    angleMotor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    // Check for stall conditions periodically (every 0.5 seconds)
    double currentTime = Timer.getFPGATimestamp();
    if (currentTime - lastStallCheckTime > 0.5) {
      if (isStalling()) {
        Utils.logInfo("Arm stalling! Current: " + angleMotor.getSupplyCurrent().getValueAsDouble() + "A");
      }
      lastStallCheckTime = currentTime;
    }
  }
  
  // ============================================================
  // Position Control Methods
  // ============================================================
  
  /**
   * Sets the arm to a target position in degrees
   * @param degrees Target angle in degrees from the home position
   */
  public void setPositionDegrees(double degrees) {
    // Clamp to safe range
    degrees = Math.max(ArmConstants.Positions.HOME, Math.min(degrees, ArmConstants.Positions.MAX));
    
    targetPositionDegrees = degrees;
    double rotations = degreesToRotations(degrees);
    
    angleMotor.setControl(mmRequest.withPosition(rotations));
  }
  
  /**
   * Sets the arm to a named position
   * @param position One of the Positions constants
   */
  public void setPosition(double position) {
    setPositionDegrees(position);
  }
  
  /**
   * Stops the arm (applies neutral output)
   */
  public void stop() {
    angleMotor.setControl(neutralRequest);
  }
  
  /**
   * Resets the arm position to zero
   * Should be called when arm is at the bottom
   */
  public void resetPosition() {
    angleMotor.setPosition(0);
    targetPositionDegrees = 0;
    Utils.logInfo("Arm position reset to zero");
  }
  
  // ============================================================
  // State Query Methods
  // ============================================================
  
  /**
   * Gets the current arm position in degrees
   * @return Current position in degrees
   */
  public double getPositionDegrees() {
    return rotationsToDegrees(angleMotor.getPosition().getValueAsDouble());
  }
  
  /**
   * Gets the current arm position in rotations
   * @return Current position in motor rotations
   */
  public double getPositionRotations() {
    return angleMotor.getPosition().getValueAsDouble();
  }
  
  /**
   * Gets the target position in degrees
   * @return Target position in degrees
   */
  public double getTargetPositionDegrees() {
    return targetPositionDegrees;
  }
  
  /**
   * Gets the current velocity in degrees per second
   * @return Velocity in degrees/second
   */
  public double getVelocityDegreesPerSecond() {
    double rps = angleMotor.getVelocity().getValueAsDouble();
    return rps * ArmConstants.kDegreesPerRotation;
  }
  
  /**
   * Checks if the arm is at the target position
   * @return True if within tolerance
   */
  public boolean atTarget() {
    double error = Math.abs(getPositionDegrees() - targetPositionDegrees);
    return error < ArmConstants.kPositionToleranceDegrees;
  }
  
  /**
   * Checks if the arm is stalling
   * A stall is detected when:
   * 1. Not at target position
   * 2. Velocity is near zero
   * 3. Current draw is high
   * @return True if stalling
   */
  public boolean isStalling() {
    // Skip if at target (expected to have zero velocity)
    if (atTarget()) return false;
    
    double velocity = Math.abs(angleMotor.getVelocity().getValueAsDouble());
    double current = angleMotor.getSupplyCurrent().getValueAsDouble();
    
    return velocity < ArmConstants.kStallVelocityThreshold && 
           current > ArmConstants.kStallCurrentThreshold;
  }
  
  /**
   * Gets the current supply current draw
   * @return Current in amps
   */
  public double getCurrentAmps() {
    return angleMotor.getSupplyCurrent().getValueAsDouble();
  }
  
  /**
   * Gets the closed loop error in degrees
   * @return Error in degrees
   */
  public double getErrorDegrees() {
    return rotationsToDegrees(angleMotor.getClosedLoopError().getValueAsDouble());
  }
  
  // ============================================================
  // Unit Conversion Methods
  // ============================================================
  
  /**
   * Converts degrees to motor rotations
   * @param degrees Rotation in degrees
   * @return Motor rotations
   */
  private double degreesToRotations(double degrees) {
    return degrees / ArmConstants.kDegreesPerRotation;
  }
  
  /**
   * Converts motor rotations to degrees
   * @param rotations Motor rotations
   * @return Rotation in degrees
   */
  private double rotationsToDegrees(double rotations) {
    return rotations * ArmConstants.kDegreesPerRotation;
  }
  
  // ============================================================
  // Command Factory Methods
  // ============================================================
  
  /**
   * Creates a command to move to a specific position
   * @param degrees Target position in degrees
   * @return Command that moves to position and ends when reached
   */
  public Command moveToPositionCommand(double degrees) {
    return runOnce(() -> setPositionDegrees(degrees))
      .andThen(run(() -> {}).until(this::atTarget))
      .withName("MoveArmTo_" + degrees);
  }
  
  /**
   * Creates a command to move to ground position
   */
  public Command moveToHomeCommand() {
    return moveToPositionCommand(ArmConstants.Positions.HOME).withName("ArmGround");
  }
  
  /**
   * Creates a command to move to coral moving position
   */
  public Command moveToCoralMoveCommand() {
    return moveToPositionCommand(ArmConstants.Positions.CORALMOVE).withName("ArmCoralMove");
  }
  
  /**
   * Creates a command to move to coral level 1 position
   */
  public Command moveToCoralOneCommand() {
    return moveToPositionCommand(ArmConstants.Positions.CORAL1).withName("ArmCoralOne");
  }
  
  /**
   * Creates a command to move to coral level 2 position
   */
  public Command moveToCoralTwoCommand() {
    return moveToPositionCommand(ArmConstants.Positions.CORAL2).withName("ArmCoralTwo");
  }
  
  /**
   * Creates a command to move to coral level 3 position
   */
  public Command moveToCoralThreeCommand() {
    return moveToPositionCommand(ArmConstants.Positions.CORAL3).withName("ArmCoralThree");
  }
  
  /**
   * Creates a command to move to coral level 4 position
   */
  public Command moveToCoralFourCommand() {
    return moveToPositionCommand(ArmConstants.Positions.CORAL4).withName("ArmCoralFour");
  }
  
  /**
   * Creates a command to move to algae moving position
   */
  public Command moveToAlgaeMoveCommand() {
    return moveToPositionCommand(ArmConstants.Positions.ALGAEMOVE).withName("ArmAlgaeMove");
  }
  
  /**
   * Creates a command to move to algae level 1 position
   */
  public Command moveToAlgaeOneCommand() {
    return moveToPositionCommand(ArmConstants.Positions.ALGAE1).withName("ArmAlgaeOne");
  }
  
  /**
   * Creates a command to move to algae level 2 position
   */
  public Command moveToAlgaeTwoCommand() {
    return moveToPositionCommand(ArmConstants.Positions.ALGAE2).withName("ArmAlgaeTwo");
  }
  
  /**
   * Creates a command to move to algae processor position
   */
  public Command moveToAlgaeProcessorCommand() {
    return moveToPositionCommand(ArmConstants.Positions.ALGAEPROCESSOR).withName("ArmAlgaeProcessor");
  }
  
  /**
   * Creates a command to move to algae level 3 position
   */
  public Command moveToAlgaeNetCommand() {
    return moveToPositionCommand(ArmConstants.Positions.ALGAENET).withName("ArmAlgaeNet");
  }
  
  /**
   * Creates a command to reset the arm position
   * Use when arm is manually positioned at bottom
   */
  public Command resetPositionCommand() {
    return runOnce(this::resetPosition).withName("ResetArmPosition");
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
    angleMotor.setNeutralMode(mode);
  }
  
  // ============================================================
  // Dashboard/Telemetry
  // ============================================================
  
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Position (deg)", this::getPositionDegrees, null);
    builder.addDoubleProperty("Target (deg)", this::getTargetPositionDegrees, null);
    builder.addDoubleProperty("Error (deg)", this::getErrorDegrees, null);
    builder.addDoubleProperty("Velocity (deg/s)", this::getVelocityDegreesPerSecond, null);
    builder.addDoubleProperty("Current (A)", this::getCurrentAmps, null);
    builder.addBooleanProperty("At Target", this::atTarget, null);
    builder.addBooleanProperty("Stalling", this::isStalling, null);
  }
}
