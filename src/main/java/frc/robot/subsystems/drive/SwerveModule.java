// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  private final VelocityVoltage driveVelocityRequest;

  // Cached configurations
  private final TalonFXConfiguration driveConfig;
  private final SparkMaxConfig steerConfig;

  // Module identification
  private final int driveMotorID;
  private final int steerMotorID;
  private final boolean driveMotorInverted;
  private final boolean steerMotorInverted;
  private final int absoluteEncoderID;
  private final double absoluteEncoderOffsetRadians;
  private final boolean absoluteEncoderInverted;
  private final String moduleName;

  // Cached state & position for optimization
  private SwerveModuleState cachedState;
  private SwerveModulePosition cachedPosition;

  // Target state (used for telemetry only)
  private SwerveModuleState targetState;

  /**
   * Construct a SwerveModule with the given parameters
   * @param moduleName Name of the module for debugging
   * @param driveMotorID CAN ID of the drive motor
   * @param steerMotorID CAN ID of the steering motor
   * @param driveMotorInverted Indicates that the drive motor is inverted
   * @param steerMotorInverted Indicates that the steer motor is inverted
   * @param absoluteEncoderID Analog ID of the absolute encoder
   * @param absoluteEncoderOffsetRadians Offset angle in radians for the absolute encoder
   * @param absoluteEncoderInverted Indicates that the absolute encoder is inverted
   */
  public SwerveModule(
    String moduleName,
    int driveMotorID,
    int steerMotorID,
    boolean driveMotorInverted,
    boolean steerMotorInverted,
    int absoluteEncoderID,
    double absoluteEncoderOffsetRadians,
    boolean absoluteEncoderInverted
  ) {
    // Cache module info
    this.moduleName = moduleName;
    this.driveMotorID = driveMotorID;
    this.steerMotorID = steerMotorID;
    this.driveMotorInverted = driveMotorInverted;
    this.steerMotorInverted = steerMotorInverted;
    this.absoluteEncoderID = absoluteEncoderID;
    this.absoluteEncoderOffsetRadians = absoluteEncoderOffsetRadians;
    this.absoluteEncoderInverted = absoluteEncoderInverted;

    // Initialize the drive motor
    driveMotor = new TalonFX(this.driveMotorID);
    driveConfig = new TalonFXConfiguration();

    // Initialize the steering motor
    steerMotor = new SparkMax(this.steerMotorID, MotorType.kBrushless);
    steerConfig = new SparkMaxConfig();

    // Initialize the absolute encoder
    absoluteEncoder = new ThriftyBotEncoder(
      this.absoluteEncoderID, 
      this.absoluteEncoderOffsetRadians, 
      this.absoluteEncoderInverted,
      this.moduleName
    );

    // Initialize the drive velocity request
    driveVelocityRequest = new VelocityVoltage(0).withSlot(0);

    // Configure drive motor
    configureDriveMotor();

    // Configure steering motor
    configureSteerMotor();

    // Get the steering motor's onboard PID controller
    steerPIDController = steerMotor.getClosedLoopController();

    // Get the steering motor's relative encoder
    steerEncoder = steerMotor.getEncoder();

    // Reset encoders (do this after getting the encoder)
    resetEncoders();

    // Initialize cached state & position
    cachedState = new SwerveModuleState(0.0, new Rotation2d());
    cachedPosition = new SwerveModulePosition(0.0, new Rotation2d());
    targetState = new SwerveModuleState(0.0, new Rotation2d());

    // Initialize dashboard values
    SmartDashboard.putData("Drive/Modules/" + this.moduleName, this);

    // Output initialization progress
    Utils.logInfo(this.moduleName + " swerve module initialized");
  }

  /**
   * Called periodically by the drive subsystem
   */
  public void periodic() {
    // Update cached state (avoids newing SwerveModuleState each time)
    this.cachedState.speedMetersPerSecond = (driveMotor.getVelocity().getValueAsDouble() / DriveConstants.kDriveGearRatio) * DriveConstants.kWheelCircumference;
    this.cachedState.angle = new Rotation2d(normalizeAngle(steerEncoder.getPosition()));

    // Update cached position (avoids newing SwerveModulePosition each time)
    this.cachedPosition.distanceMeters = (driveMotor.getPosition().getValueAsDouble() / DriveConstants.kDriveGearRatio) * DriveConstants.kWheelCircumference;
    this.cachedPosition.angle = new Rotation2d(normalizeAngle(steerEncoder.getPosition()));

    // Update the absolute encoder (for diagnostics)
    this.absoluteEncoder.periodic();
  }

  /**
   * Configures the drive motor.
   * We use a Kraken x60.
   */
  private void configureDriveMotor() {
    // Motor outputs
    driveConfig.MotorOutput
      .withDutyCycleNeutralDeadband(0.001)  // 0.1% deadband (tight control)
      .withNeutralMode(NeutralModeValue.Brake)
      .withInverted(driveMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);
    
    // Current limits (hardcoded here for safety)
    driveConfig.CurrentLimits
      .withSupplyCurrentLimitEnable(true)   // Enable supply limits
      .withSupplyCurrentLimit(60)                 // Peak current spike limit in Amps
      .withSupplyCurrentLowerLimit(40)       // Continuous current limit in Amps
      .withSupplyCurrentLowerTime(0.5)        // Time until lower current in seconds
      .withStatorCurrentLimitEnable(true)   // Enable stator limits
      .withStatorCurrentLimit(80);                // Max stator current in Amps (prevents overheating)

    // Voltage compensation
    driveConfig.Voltage
      .withPeakForwardVoltage(12)                   // Max voltage when running motor forward
      .withPeakReverseVoltage(-12)                                        // Max voltage when running motor in reverse
      .withSupplyVoltageTimeConstant(0.02);  // Voltage filter time constant in seconds

    // Velocity PID (runs on onboard motor controller, tunable in constants)
    driveConfig.Slot0
      .withKP(DriveConstants.kDriveKP)      // Poportional gain
      .withKI(DriveConstants.kDriveKI)      // Integral gain
      .withKD(DriveConstants.kDriveKD)      // Derivative gain
      .withKS(DriveConstants.kDriveKS)      // Static feedforward
      .withKV(DriveConstants.kDriveKV)      // Velocity feedforward
      .withKA(DriveConstants.kDriveKA);     // Acceleration feedforward
    
    // Apply the configuration to the motor
    driveMotor.getConfigurator().apply(driveConfig);

    // -------------------------------------------------------
    // OPTIMIZE CAN STATUS FRAMES for reduced lag
    // -------------------------------------------------------

    // HIGH PRIORITY - Critical for control (100Hz = 10ms)
    driveMotor.getVelocity().setUpdateFrequency(100.0);     // Velocity feedback
    driveMotor.getPosition().setUpdateFrequency(100.0);     // Position feedback
    
    // MEDIUM PRIORITY - Useful for monitoring (50Hz = 20ms)
    driveMotor.getMotorVoltage().setUpdateFrequency(50.0);  // Motor voltage
    driveMotor.getSupplyCurrent().setUpdateFrequency(50.0); // Supply current
    driveMotor.getTorqueCurrent().setUpdateFrequency(50.0); // Stator/torque current
    
    // LOW PRIORITY - Reduce CAN traffic (4Hz = 250ms)
    driveMotor.getDeviceTemp().setUpdateFrequency(4.0);     // Temperature

    // Optimize CAN bus utilization
    driveMotor.optimizeBusUtilization();
  }

  /**
   * Configures the steering motor.
   * We use a Neo with Spark Max controller.
   */
  private void configureSteerMotor() {
    // Basic motor configuration (hardcoded here for safety)
    steerConfig
      .idleMode(IdleMode.kBrake)                       // Brake mode for better control
      .inverted(steerMotorInverted)                    // Set motor inversion
      .smartCurrentLimit(25)                // Continuous current limit in Amps
      .secondaryCurrentLimit(35, 20)  // Peak current limit: 35A for 20ms
      .voltageCompensation(12.0);       // Enable voltage compensation at 12V

    // Closed-loop PID configuration (gains are tunable in constants)
    steerConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(-1.0, 1.0)
      .positionWrappingEnabled(true)      
      .positionWrappingInputRange(0.0, 2 * Math.PI) 
      .p(DriveConstants.kSteerKP)                 // Proportional gain
      .i(DriveConstants.kSteerKI)                 // Integral gain
      .d(DriveConstants.kSteerKD)                 // Derivative gain
      .feedForward.kS(DriveConstants.kSteerKS);   // Static feedforward gain

    // Set position conversion factor (rotations to radians)
    // Set velocity conversion factor (rotations per minute to radians per second)
    steerConfig.encoder
      .positionConversionFactor(2 * Math.PI / DriveConstants.kSteerGearRatio)
      .velocityConversionFactor(2 * Math.PI / DriveConstants.kSteerGearRatio / 60.0);

    // Optimize CAN status frames for reduced lag
    steerConfig.signals
      .primaryEncoderPositionPeriodMs(10)   // Position: 100Hz (was Status2)
      .primaryEncoderVelocityPeriodMs(10)   // Velocity: 100Hz (was Status2)
      .appliedOutputPeriodMs(100)           // Applied output: 10Hz (was Status0)
      .faultsPeriodMs(200)                  // Faults: 5Hz (was Status1)
      .analogVoltagePeriodMs(500)           // Analog: unused (was Status3)
      .externalOrAltEncoderPosition(500)    // Alt encoder: unused (was Status4)
      .externalOrAltEncoderVelocity(500);   // Alt encoder: unused (was Status4)    

    // Apply the configuration to the motor
    steerMotor.configure(
      this.steerConfig, 
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
   * Gets the target state of the swerve module
   * @return
   */
  public SwerveModuleState getTargetState() {
    return new SwerveModuleState(targetState.speedMetersPerSecond, targetState.angle);
  }

  /**
   * Gets the most recent cached state of the swerve module
   * Updated in periodic() to help with optimization
   * @return Defensive copy of cached SwerveModuleState
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(cachedState.speedMetersPerSecond, cachedState.angle);
  }

  /**
   * Gets the most recent cached position of the swerve module
   * Updated in periodic() to help with optimization
   * @return Defensive copy of cached SwerveModulePosition
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(cachedPosition.distanceMeters, cachedPosition.angle);
  }

  /**
   * Sets the desired state of the swerve module
   * @param desiredState The desired SwerveModuleState
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Ensure desired state is not null
    if (desiredState == null) {
      return;
    }
  
    // The desired state and has already been optimized by setpoint generator.
    // Uncomment the line below only if NOT using the setpoint generator.
    // This line is left here so future maintainers are aware.
    // desiredState.optimize(getState().angle);
    
    // Command the robot to move
    setDriveVelocity(desiredState.speedMetersPerSecond);
    setSteerAngle(desiredState.angle.getRadians());

    // Update target state with current desired state for telemetry
    targetState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    targetState.angle = desiredState.angle;
  }

  /**
   * Sets the drive motor velocity adjusted for gear ratio and wheel circumference
   * @param velocityMPS Desired velocity in m/s
   */
  private void setDriveVelocity(double velocityMPS) {
    double velocityRPS = (velocityMPS / DriveConstants.kWheelCircumference) * DriveConstants.kDriveGearRatio;
    driveVelocityRequest.Velocity = velocityRPS;
    driveMotor.setControl(driveVelocityRequest);
  }

  /**
   * Sets the steering angle
   * @param angleRadians Desired angle in radians
   */
  private void setSteerAngle(double angleRadians) {
    double angle = normalizeAngle(angleRadians);
    steerPIDController.setSetpoint(angle, SparkMax.ControlType.kPosition);
  }

  /**
   * Normalizes an angle to the range [0, 2PI)
   * @param angleRadians Angle in radians
   * @return Normalized angle in radians
   */
  private double normalizeAngle(double angleRadians) {
    return MathUtil.inputModulus(angleRadians, 0, 2 * Math.PI);
  }

  /**
   * Set both drive and steering motor motor to brake/coast mode
   * @param brake True for brake mode, false for coast mode
   */
  public void setMotorBrake(boolean brake) {
    // Set drive motor neutral/idle mode
    driveMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    
    // Set steering motor neutral/idle mode
    steerConfig.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    steerMotor.configure(
      steerConfig, 
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
   * Initialize the data sent to SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Target Speed (mps)", () -> targetState.speedMetersPerSecond, null);
    builder.addDoubleProperty("Target Angle (deg)", () -> targetState.angle.getDegrees(), null);
    builder.addDoubleProperty("Current Speed (mps)", () -> getState().speedMetersPerSecond, null);
    builder.addDoubleProperty("Current Angle (deg)", () -> getState().angle.getDegrees(), null);
    builder.addDoubleProperty("Absolute Encoder (deg)", () -> absoluteEncoder.getAngleDegrees(), null);
    builder.addDoubleProperty("Drive Voltage (V)", () -> driveMotor.getSupplyVoltage().getValueAsDouble(), null);
    builder.addDoubleProperty("Drive Current (A)", () -> driveMotor.getSupplyCurrent().getValueAsDouble(), null);
    builder.addDoubleProperty("Drive Temp (deg C)", () -> driveMotor.getDeviceTemp().getValueAsDouble(), null);
    builder.addDoubleProperty("Steer Voltage (V)", () -> steerMotor.getAppliedOutput(), null);
    builder.addDoubleProperty("Steer Current (A)", () -> steerMotor.getOutputCurrent(), null);
    builder.addDoubleProperty("Steer Temp (deg C)", () -> steerMotor.getMotorTemperature(), null);
  }
}
