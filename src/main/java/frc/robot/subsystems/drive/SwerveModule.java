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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.util.ThriftyBotEncoder;

/**
 * Represents a single swerve drive module with a Kraken60 drive motor
 * and a Neo550 steering motor with absolute encoder feedback
 */
public class SwerveModule {
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

    // Initialize the drive motor (we use a Kraken60)
    driveMotor = new TalonFX(this.driveMotorID);

    // Initialize the steering motor (we use a Neo550) 
    steerMotor = new SparkMax(this.steerMotorID, MotorType.kBrushless);

    // Initialize the absolute encoder (we use a ThriftyBot)
    absoluteEncoder = new ThriftyBotEncoder(
      this.absoluteEncoderID, 
      this.absoluteEncoderOffsetRadians, 
      this.absoluteEncoderInverted,
      this.moduleName + " Encoder"
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
  }

  /**
   * Configures the Kraken60 drive motor
   */
  private void configureDriveMotor() {
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    // Motor output configuration
    driveConfig.MotorOutput
      .withNeutralMode(NeutralModeValue.Brake)
      .withInverted(
        driveMotorInverted 
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive
      );
    
    // Current limits
    driveConfig.CurrentLimits
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(DriveConstants.kDriveMotorMaxCurrent)
      .withSupplyCurrentLowerLimit(DriveConstants.kDriveMotorMaxCurrent * 0.75)
      .withSupplyCurrentLowerTime(1.0);
    
    // Voltage compensation
    driveConfig.Voltage
      .withPeakForwardVoltage(DriveConstants.kDriveMaxForwardVoltage)
      .withPeakReverseVoltage(DriveConstants.kDriveMaxReverseVoltage);
    
    // PID configuration for velocity control
    driveConfig.Slot0
      .withKP(DriveConstants.kDriveKP)
      .withKI(DriveConstants.kDriveKI)
      .withKD(DriveConstants.kDriveKD);
    
    // Apply the configuration to the motor
    driveMotor.getConfigurator().apply(driveConfig);
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
      .smartCurrentLimit(DriveConstants.kSteerMotorMaxCurrent);

    // Set position conversion factor (rotations to radians)
    // Set velocity conversion factor (rotations per minute to radians per second)
    steerConfig.encoder
      .positionConversionFactor(2 * Math.PI / DriveConstants.kSteerGearRatio)
      .velocityConversionFactor(2 * Math.PI / DriveConstants.kSteerGearRatio / 60.0);

    // Closed-loop PID configuration
    steerConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .positionWrappingEnabled(true)
      .positionWrappingMinInput(-Math.PI)
      .positionWrappingMaxInput(Math.PI)
      .outputRange(-1.0, 1.0)
      .p(DriveConstants.kSteerKP)
      .i(DriveConstants.kSteerKI)
      .d(DriveConstants.kSteerKD);

    // Apply the configuration to the motor
    steerMotor.configure(
      steerConfig, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );
  }

  /**
   * Periodic method to be called by the drive subsystem's periodic.
   * Can put diagnostics, telemetry or state updates here.
   */
  public void periodic() {
    // Call the absolute encoder periodic method
    absoluteEncoder.periodic();
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
   * Gets the current state of the swerve module
   * @return Current SwerveModuleState
   */
  public SwerveModuleState getState() {
    double velocity = driveMotor.getVelocity().getValueAsDouble() * DriveConstants.kDriveRpsToMps;
    Rotation2d angle = new Rotation2d(steerEncoder.getPosition());
    return new SwerveModuleState(velocity, angle);
  }

  /**
   * Gets the current position of the swerve module
   * @return Current SwerveModulePosition
   */
  public SwerveModulePosition getPosition() {
    double distance = driveMotor.getPosition().getValueAsDouble() * DriveConstants.kDriveRpsToMps;
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

    // Smooth out the steering angle (may not need this?)
    desiredState.cosineScale(getState().angle);

    // Prevent jittering from noise that can appear when the swerve module is idle.
    // NOTE: may not be necessary
    // NOTE: Commented out to start since this may interfere with PID tuning
    // if (
    //   Math.abs(desiredState.speedMetersPerSecond) < 0.001 && // less than 1 mm per sec
		// 	Math.abs(desiredState.angle.getRadians() - steerEncoder.getPosition()) < Rotation2d.fromDegrees(1).getRadians() // less than 1 degree
    // ) {
		// 	stop();
    //   return;
		// }

    // Set drive motor speed
    setDriveVelocity(desiredState.speedMetersPerSecond);
    
    // Set steering angle
    setSteerAngle(desiredState.angle.getRadians());
    
    // Update dashboard
    updateDashboard(desiredState);      
  }

  /**
   * Sets the drive motor velocity adjusted for gear ratio and wheel circumference
   * @param velocityMetersPerSecond Desired velocity in m/s
   */
  private void setDriveVelocity(double velocityMetersPerSecond) {
    double velocityRPS = velocityMetersPerSecond * DriveConstants.kDriveMpsToRps;
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
   * Updates SmartDashboard with module information
   * @param state Current desired state
   */
  private void updateDashboard(SwerveModuleState state) {
    SmartDashboard.putNumber(moduleName + " Desired Speed", state.speedMetersPerSecond);
    SmartDashboard.putNumber(moduleName + " Desired Angle", state.angle.getDegrees());
    SmartDashboard.putNumber(moduleName + " Current Speed", getState().speedMetersPerSecond);
    SmartDashboard.putNumber(moduleName + " Current Angle", getState().angle.getDegrees());
    SmartDashboard.putNumber(moduleName + " Absolute Encoder", absoluteEncoder.getAngleDegrees());
    SmartDashboard.putNumber(moduleName + " Drive Current", driveMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(moduleName + " Drive Temp", driveMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(moduleName + " Steer Current", steerMotor.getOutputCurrent());
    SmartDashboard.putNumber(moduleName + " Steer Temp", steerMotor.getMotorTemperature());
  }
}
