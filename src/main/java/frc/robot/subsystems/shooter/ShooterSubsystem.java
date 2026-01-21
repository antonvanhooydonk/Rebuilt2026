// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PWMConstants;
import frc.robot.util.Utils;

public class ShooterSubsystem extends SubsystemBase {
  // Hardware
  private final TalonFX deflectorMotor;
  private final TalonFX shooterMotor;
  
  // Control requests (reusable objects)
  private final VoltageOut voltageRequest = new VoltageOut(0);
  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // Initialize hardware
    deflectorMotor = new TalonFX(PWMConstants.ShooterID);
    shooterMotor = new TalonFX(PWMConstants.ShooterID);
    
    // Configure motor
    configureMotor();    
        
    // Initialize dashboard
    SmartDashboard.putData("Shooter", this);
    
    // Output initialization progress
    Utils.logInfo("Shooter subsystem initialized");
  }
  
  /**
   * Configure the shooter motor with all settings
   */
  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // Motor output configuration
    config.MotorOutput
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withNeutralMode(NeutralModeValue.Brake);
    
    // Current limits
    config.CurrentLimits
      .withSupplyCurrentLimitEnable(true)
      .withSupplyCurrentLimit(70)
      .withStatorCurrentLimitEnable(true)
      .withStatorCurrentLimit(80);

    // Voltage limits
    config.Voltage
      .withPeakForwardVoltage(12.0)
      .withPeakReverseVoltage(-12.0);
    
    // Apply configuration
    shooterMotor.getConfigurator().apply(config);

    // Set update frequencies based on what we need
    shooterMotor.getVelocity().setUpdateFrequency(50);      // 50 Hz for monitoring
    shooterMotor.getSupplyCurrent().setUpdateFrequency(10); // 10 Hz for current monitoring
    shooterMotor.getMotorVoltage().setUpdateFrequency(10);  // 10 Hz
    shooterMotor.getDeviceTemp().setUpdateFrequency(4);     // 4 Hz - temperature changes slowly
    
    // Optimize CAN bus utilization
    shooterMotor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {}
    
  // ==================== Control Methods ====================
  
  /**
   * Set shooter deflector to a specific angle (in degrees)
   * @param angleDegrees Angle to set deflector to (in degrees)
   */
  private void setDeflectorAngle(double angleDegrees) {
    double clampedAngle = MathUtil.clamp(angleDegrees, 0, 45);
    deflectorMotor.setControl(voltageRequest.withOutput(clampedAngle));
  }

  /**
   * Set shooter motor to a specific voltage
   * @param voltage Voltage to apply (-12.0 to 12.0)
   */
  private void setVoltage(double voltage) {
    double clampedVoltage = MathUtil.clamp(voltage, -12, 12);
    shooterMotor.setControl(voltageRequest.withOutput(clampedVoltage));
  }
  
  /**
   * Stop the shooter motor
   */
  private void stop() {
    setVoltage(0);
  }

  /**
   * Convert a percentage (0.0 to 1.0) to a voltage (0V to 12V)
   * @param percent Percentage to convert, expressed as a decimal - 0.0 to 1.0
   * @return Voltage equivalent (0.0V to 12.0V)
   */
  private double percentToVoltage(double percent) {
    return MathUtil.clamp(percent, 0, 1) * 12.0;
  } 
  
  // ==================== Command Factories ====================

  /**
   * Command to output/shoot fuel into the hub.
   * @return Command that runs shooter
   */
  public Command outputFuelCommand(DoubleSupplier distanceToHubMeters) {
    return run(() -> {
      double distance = distanceToHubMeters.getAsDouble();
      double requiredAngleDegrees = 0;
      double requiredVoltage = 0;

      // Determine required voltage & deflector angle based on distance
      if (distance > 0 && distance <= 1) {
        requiredVoltage = percentToVoltage(0.25);
        requiredAngleDegrees = 0.0;
      } else if (distance > 1 && distance <= 2) {
        requiredVoltage = percentToVoltage(0.30);
        requiredAngleDegrees = 0.0;
      } else if (distance > 2 && distance <= 3) {
        requiredVoltage = percentToVoltage(0.35);
        requiredAngleDegrees = 0.0;
      }

      // Configure shooter state based on distance
      setDeflectorAngle(requiredAngleDegrees);
      setVoltage(requiredVoltage);
    }).withName("OutputFuel");
  }
  
  /**
   * Command to stop the shooter
   * @return Command that stops the shooter motor
   */
  public Command stopCommand() {
    return runOnce(this::stop)
      .withName("StopShooter");
  }
  
  // ==================== Telemetry Methods ====================
  
  /**
   * Get current motor velocity in rotations per second
   * @return velocity in RPS
   */
  private double getVelocityRPS() {
    return shooterMotor.getVelocity().getValueAsDouble();
  }
  
  /**
   * Get current motor supply current
   * @return current in amps
   */
  private double getCurrent() {
    return shooterMotor.getSupplyCurrent().getValueAsDouble();
  }
  
  /**
   * Get current motor voltage
   * @return voltage in volts
   */
  private double getVoltage() {
    return shooterMotor.getMotorVoltage().getValueAsDouble();
  }
  
  /**
   * Get motor temperature
   * @return temperature in Celsius
   */
  private double getTemperature() {
    return shooterMotor.getDeviceTemp().getValueAsDouble();
  }

  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ShooterSubsystem");
    builder.addDoubleProperty("Velocity (RPS)", () -> Utils.showDouble(getVelocityRPS()), null);
    builder.addDoubleProperty("Voltage (V)", () -> Utils.showDouble(getVoltage()), null);
    builder.addDoubleProperty("Current (A)", () -> Utils.showDouble(getCurrent()), null);
    builder.addDoubleProperty("Temperature (C)", () -> Utils.showDouble(getTemperature()), null);
  }
}
