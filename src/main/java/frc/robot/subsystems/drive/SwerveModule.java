// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.util.ThriftyBotEncoder;
import frc.robot.util.Utils;

/**
 * Represents a single swerve drive module with a Kraken x60 drive motor,
 * and a Neo steering motor with a ThriftyBot absolute encoder attached.
 */
public class SwerveModule implements Sendable {
  // Hardware
  private final TalonFX driveMotor;
  private final SparkMax steerMotor;
  private final ThriftyBotEncoder absoluteEncoder;

  // Controllers
  private final SparkClosedLoopController steerPIDController;
  private final RelativeEncoder steerEncoder;

  // Control requests
  private final VelocityDutyCycle driveVelocityRequest = new VelocityDutyCycle(0);

  // Module identification
  private final int driveMotorID;
  private final int steerMotorID;
  private final boolean driveMotorInverted;
  private final boolean steerMotorInverted;
  private final int absoluteEncoderID;
  private final double absoluteEncoderOffsetRadians;
  private final boolean absoluteEncoderInverted;
  private final String moduleName;

  // Previous target/desired state
  private SwerveModuleState lastState;

  private double lastMoveAtTime = 0;

  /**
   * Construct a SwerveModule with the given parameters
   * @param driveMotorID CAN ID of the drive motor
   * @param steerMotorID CAN ID of the steering motor
   * @param driveMotorInverted Indicates that the drive motor is inverted
   * @param steerMotorInverted Indicates that the steer motor is inverted
   * @param absoluteEncoderPort Analog port of the absolute encoder
   * @param absoluteEncoderOffsetRadians Offset angle in radians for the absolute encoder
   * @param absoluteEncoderInverted Indicates that the absolute encoder is inverted
   * @param moduleName Name of the module for debugging
   */
  public SwerveModule(
    int driveMotorID,
    int steerMotorID,
    boolean driveMotorInverted,
    boolean steerMotorInverted,
    int absoluteEncoderID,
    double absoluteEncoderOffsetRadians,
    boolean absoluteEncoderInverted,
    String moduleName
  ) {
    // Cache module info
    this.driveMotorID = driveMotorID;
    this.steerMotorID = steerMotorID;
    this.driveMotorInverted = driveMotorInverted;
    this.steerMotorInverted = steerMotorInverted;
    this.absoluteEncoderID = absoluteEncoderID;
    this.absoluteEncoderOffsetRadians = absoluteEncoderOffsetRadians;
    this.absoluteEncoderInverted = absoluteEncoderInverted;
    this.moduleName = moduleName;

    // Initialize the drive motor (we use a Kraken x60)
    driveMotor = new TalonFX(this.driveMotorID);

    // Initialize the steering motor (we use a Neo) 
    steerMotor = new SparkMax(this.steerMotorID, MotorType.kBrushless);

    // Initialize the absolute encoder (we use a ThriftyBot)
    absoluteEncoder = new ThriftyBotEncoder(
      this.absoluteEncoderID, 
      this.absoluteEncoderOffsetRadians, 
      this.absoluteEncoderInverted,
      this.moduleName
    );

    // Configure drive motor (we use a KrakenX60)
    configureDriveMotor();

    // Configure steering motor (we use a Neo550)
    configureSteerMotor();

    // Get steering PID controller and relative encoder
    steerPIDController = steerMotor.getClosedLoopController();
    steerEncoder = steerMotor.getEncoder();    

    // Reset encoders
    resetEncoders();

    // Initialize target state
    lastState = getState();

    // Initialize dashboard values
    SmartDashboard.putData("Drive/Modules/" + this.moduleName, this);

    // Output initialization progress
    Utils.logInfo(this.moduleName + " swerve module intialized");
  }

  /**
   * Called periodically by the drive subsystem
   */
  public void periodic() {
    syncEncoders();
  }

  /**
   * Configures the Kraken60 drive motor
   */
  private void configureDriveMotor() {
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    // Motor output configuration
    driveConfig.MotorOutput
      .withDutyCycleNeutralDeadband(0.001)  // 0.1% deadband (tight control)
      .withNeutralMode(NeutralModeValue.Brake)
      .withInverted(
        driveMotorInverted 
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive
      );
    
    // Current limits
    driveConfig.CurrentLimits
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(DriveConstants.kDriveMotorCurrentLimit)
      .withSupplyCurrentLowerLimit(DriveConstants.kDriveMotorCurrentLowerLimit)
      .withSupplyCurrentLowerTime(1.0)
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(120);

    // Voltage compensation
    driveConfig.Voltage
      .withPeakForwardVoltage(DriveConstants.kDriveMaxForwardVoltage)
      .withPeakReverseVoltage(DriveConstants.kDriveMaxReverseVoltage)
      .withSupplyVoltageTimeConstant(0.02);
    
    // Velocity PID (runs on onboard motor controller)
    driveConfig.Slot0
      .withKP(DriveConstants.kDriveKP)
      .withKI(DriveConstants.kDriveKI)
      .withKD(DriveConstants.kDriveKD)
      .withKS(DriveConstants.kDriveKS)
      .withKV(DriveConstants.kDriveKV)
      .withKA(DriveConstants.kDriveKA);
    
    // Apply the configuration to the motor
    driveMotor.getConfigurator().apply(driveConfig);

    // -------------------------------------------------------
    // OPTIMIZE CAN STATUS FRAMES for reduced lag

    // HIGH PRIORITY - Critical for control (100Hz = 10ms)
    driveMotor.getVelocity().setUpdateFrequency(100.0);     // Velocity feedback
    driveMotor.getPosition().setUpdateFrequency(100.0);     // Position feedback
    
    // MEDIUM PRIORITY - Useful for monitoring (50Hz = 20ms)
    driveMotor.getMotorVoltage().setUpdateFrequency(50.0);  // Motor voltage
    driveMotor.getSupplyCurrent().setUpdateFrequency(50.0); // Supply current
    driveMotor.getTorqueCurrent().setUpdateFrequency(50.0); // Stator/torque current
    
    // LOW PRIORITY - Reduce CAN traffic (4Hz = 250ms)
    driveMotor.getDeviceTemp().setUpdateFrequency(4.0);     // Temperature
  }

  /**
   * Configures the Neo550 steering motor
   */
  private void configureSteerMotor() {
    SparkMaxConfig steerConfig = new SparkMaxConfig();

    // Basic motor configuration
    steerConfig
      .idleMode(IdleMode.kBrake)
      .inverted(steerMotorInverted ? true : false)
      .smartCurrentLimit(DriveConstants.kSteerMotorMaxCurrent)
      .voltageCompensation(12.0);

    // Set position conversion factor (rotations to radians)
    // Set velocity conversion factor (rotations per minute to radians per second)
    steerConfig.encoder
      .positionConversionFactor(2 * Math.PI / DriveConstants.kSteerGearRatio)
      .velocityConversionFactor(2 * Math.PI / DriveConstants.kSteerGearRatio / 60.0);

    // Closed-loop PID configuration
    steerConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(-1.0, 1.0)
      .positionWrappingEnabled(true)
      .positionWrappingMinInput(-Math.PI)
      .positionWrappingMaxInput(Math.PI)
      .p(DriveConstants.kSteerKP)
      .i(DriveConstants.kSteerKI)
      .d(DriveConstants.kSteerKD)
      .velocityFF(DriveConstants.kSteerFF);

    // OPTIMIZE CAN STATUS FRAMES for reduced lag
    steerConfig.signals
      .primaryEncoderPositionPeriodMs(10)   // Position: 100Hz (was Status2)
      .primaryEncoderVelocityPeriodMs(10)   // Velocity: 100Hz (was Status2)
      .appliedOutputPeriodMs(100)           // Applied output: 10Hz (was Status0)
      .faultsPeriodMs(200)                  // Faults: 5Hz (was Status1)
      .analogVoltagePeriodMs(500)           // Analog: unused (was Status3)
      .externalOrAltEncoderPosition(500)    // Alt encoder: unused (was Status4)
      .externalOrAltEncoderVelocity(500);   // Alt encoder: unused (was Status4)    

    // Configure MAXMotion
    // Not currently used, but could be useful in the future.
    // Be sure to change the setSteerAngle() method to use SparkMax.ControlType.kMAXMotionPositionControl
    // steerConfig.closedLoop.maxMotion
    //   .allowedClosedLoopError(0)
    //   .maxVelocity(DriveConstants.kMaxAngularSpeedRadiansPerSecond)
    //   .maxAcceleration(DriveConstants.kMaxAngularAccelRadiansPerSecondSq);

    // Apply the configuration to the motor
    steerMotor.configure(
      steerConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
  }

  /**
   * Resets the drive and steering encoders
   */
  public void resetEncoders() {
    // Reset drive motor encoder position
    driveMotor.setPosition(0);
    
    // Set the relative encoder to match the absolute encoder
    steerEncoder.setPosition(absoluteEncoder.getAngleRadians());
  }

  /**
   * Check relative encoder is synced to absolute encoder
   */
  public void syncEncoders() {
    double absoluteAngle = absoluteEncoder.getAngleRadians();
    double relativeAngle = steerEncoder.getPosition();
    double error = Math.abs(absoluteAngle - relativeAngle);
    double timeSinceLastMove = Timer.getFPGATimestamp() - lastMoveAtTime;
    
    // Re-sync if the robot has been still for 0.5 seconds 
    // and error is > 5 degrees (indicates encoder drift)
    if (timeSinceLastMove > 0.5 && error > Units.degreesToRadians(5.0)) { 
      resetEncoders();
    }
  }

  /**
   * Gets the target state of the swerve module
   * @return
   */
  public SwerveModuleState getLastState() {
    return lastState;
  }

  /**
   * Gets the current state of the swerve module
   * @return Current SwerveModuleState
   */
  public SwerveModuleState getState() {
    double velocity = driveMotor.getVelocity().getValueAsDouble() * DriveConstants.kDriveRPStoMPS;
    Rotation2d angle = new Rotation2d(steerEncoder.getPosition());
    return new SwerveModuleState(velocity, angle);
  }

  /**
   * Gets the current position of the swerve module
   * @return Current SwerveModulePosition
   */
  public SwerveModulePosition getPosition() {
    double distance = driveMotor.getPosition().getValueAsDouble() * DriveConstants.kDriveRPStoMPS;
    Rotation2d angle = new Rotation2d(steerEncoder.getPosition());
    return new SwerveModulePosition(distance, angle);
  }

  /**
   * Sets the desired state of the swerve module
   * @param desiredState The desired SwerveModuleState
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the desired state to avoid spinning more than 90 degrees
    desiredState.optimize(getState().angle);

    // Apply anti-jitter if enabled
    if (DriveConstants.kAntiJitter) {
      desiredState = applyAntiJitter(desiredState);
    }

    // Set drive motor speed
    setDriveVelocity(desiredState.speedMetersPerSecond);
    
    // Set steering angle
    setSteerAngle(desiredState.angle.getRadians());

    // Record last time the robot was commanded to move
    if (Math.abs(desiredState.speedMetersPerSecond) > 0.001) {
      lastMoveAtTime = Timer.getFPGATimestamp();
    }

    // Cache the target state
    lastState = desiredState;
  }

  /**
   * Sets the drive motor velocity adjusted for gear ratio and wheel circumference
   * @param velocityMetersPerSecond Desired velocity in m/s
   */
  private void setDriveVelocity(double velocityMetersPerSecond) {
    double velocityRPS = velocityMetersPerSecond * DriveConstants.kDriveMPStoRPS;
    driveMotor.setControl(driveVelocityRequest.withVelocity(velocityRPS));
  }

  /**
   * Sets the steering angle
   * @param angleRadians Desired angle in radians
   */
  private void setSteerAngle(double angleRadians) {
    steerPIDController.setReference(angleRadians, SparkMax.ControlType.kPosition);
  }

  /**
   * Applies anti-jitter filtering to the new state
   * @param newState The desired SwerveModuleState to filter
   * @return The filtered SwerveModuleState
   */
  private SwerveModuleState applyAntiJitter(SwerveModuleState newState) {
    // Configurable anti-jitter parameters (move to constants)
    double speedDeadband = 0.001; // 1mm in m/s
    double angleDeadband = Units.degreesToRadians(2.0); // radians
    double minSpeedForSteering = 0.1; // m/s

    // Calculate the differences between new state and the last target state
    double speedDiff = Math.abs(newState.speedMetersPerSecond - lastState.speedMetersPerSecond);
    double angleDiff = Math.abs(newState.angle.minus(lastState.angle).getRadians());
    
    // Apply speed filtering
    double finalSpeed = (speedDiff < speedDeadband) 
      ? lastState.speedMetersPerSecond 
      : newState.speedMetersPerSecond;
    
    // Apply angle filtering with speed consideration
    Rotation2d finalAngle;
    if (Math.abs(finalSpeed) > minSpeedForSteering || angleDiff > angleDeadband) {
      finalAngle = newState.angle;
    } else {
      finalAngle = lastState.angle;
    }
    
    // Return the filtered state
    return new SwerveModuleState(finalSpeed, finalAngle);
  }

  /**
   * Set both drive and steering motor motor to brake/coast mode
   * @param brake True for brake mode, false for coast mode
   */
  public void setMotorBrake(boolean brake) {
    // Set drive motor neutral/idle mode
    driveMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    
    // Set steering motor neutral/idle mode
    steerMotor.configure(
      new SparkMaxConfig().idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kNoPersistParameters
    );
  }

  /**
   * Stops both drive and steering motors
   */
  public void stop() {
    driveMotor.stopMotor();
    steerMotor.stopMotor();
  }
  
  /**
   * Checks if the steering motor is at the target angle
   * @param toleranceRadians Tolerance in radians
   * @return True if at target
   */
  public boolean isSteerAtTarget(double toleranceRadians) {
    return Math.abs(steerEncoder.getPosition() - absoluteEncoder.getAngleRadians()) < toleranceRadians;
  }
  
  /**
   * Initialize the data sent to SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Last Target Speed", () -> lastState.speedMetersPerSecond, null);
    builder.addDoubleProperty("Last Target Angle", () -> lastState.angle.getDegrees(), null);
    builder.addDoubleProperty("Current Speed", () -> getState().speedMetersPerSecond, null);
    builder.addDoubleProperty("Current Angle", () -> getState().angle.getDegrees(), null);
    builder.addDoubleProperty("Absolute Encoder (deg)", () -> absoluteEncoder.getAngleDegrees(), null);
    builder.addDoubleProperty("Drive Current", () -> driveMotor.getSupplyCurrent().getValueAsDouble(), null);
    builder.addDoubleProperty("Drive Temp", () -> driveMotor.getDeviceTemp().getValueAsDouble(), null);
    builder.addDoubleProperty("Steer Current", () -> steerMotor.getOutputCurrent(), null);
    builder.addDoubleProperty("Steer Temp", () -> steerMotor.getMotorTemperature(), null);
  }
}
