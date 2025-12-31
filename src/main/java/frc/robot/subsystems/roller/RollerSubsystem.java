// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DIOConstants;
import frc.robot.Constants.PWMConstants;
import frc.robot.util.Utils;

public class RollerSubsystem extends SubsystemBase {
  // Hardware
  private final TalonFX rollerMotor;
  private final DigitalInput beamBreakSensor;
  
  // Control requests (reusable objects)
  private final VoltageOut voltageRequest = new VoltageOut(0);
  
  /** Creates a new RollerSubsystem. */
  public RollerSubsystem() {
    // Initialize hardware
    beamBreakSensor = new DigitalInput(DIOConstants.kBeamBreakSensorID);
    rollerMotor = new TalonFX(PWMConstants.RollerID);
    
    // Configure motor
    configureMotor();    
        
    // Initialize dashboard
    SmartDashboard.putData("Roller", this);
    
    // Output initialization progress
    Utils.logInfo("Roller subsystem initialized");
  }
  
  /**
   * Configure the roller motor with all settings
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
    
    // Slot 0 PID configuration (if needed for closed-loop control)
    config.Slot0
      .withKP(1)
      .withKI(0)
      .withKD(0)
      .withKS(0)
      .withKV(0)
      .withKA(0);
    
    // Apply configuration
    rollerMotor.getConfigurator().apply(config);

    // Set update frequencies based on what we need
    rollerMotor.getVelocity().setUpdateFrequency(50);      // 50 Hz for monitoring
    rollerMotor.getSupplyCurrent().setUpdateFrequency(10); // 10 Hz for current monitoring
    rollerMotor.getMotorVoltage().setUpdateFrequency(10);  // 10 Hz
    rollerMotor.getDeviceTemp().setUpdateFrequency(4);     // 4 Hz - temperature changes slowly
    
    // Optimize CAN bus utilization
    rollerMotor.optimizeBusUtilization();
  }

  @Override
  public void periodic() {}
  
  // ==================== Sensor Methods ====================
  
  /**
   * Check if the beam break sensor detects a game piece
   * @return true if beam is broken (game piece present)
   */
  public boolean hasGamePiece() {
    return !beamBreakSensor.get();
  }
  
  // ==================== Control Methods ====================
  
  /**
   * Set roller motor to a specific voltage
   * @param voltage Voltage to apply (-12.0 to 12.0)
   */
  private void setVoltage(double voltage) {
    double clampedVoltage = MathUtil.clamp(voltage, -12, 12);
    rollerMotor.setControl(voltageRequest.withOutput(clampedVoltage));
  }
  
  /**
   * Stop the roller motor
   */
  private void stop() {
    setVoltage(0);
  }
  
  // ==================== Command Factories ====================
  
  /**
   * Command to intake coral
   * @return Command that runs roller at intake speed for coral
   */
  public Command intakeCoralCommand() {
    return run(() -> setVoltage(RollerConstants.IntakeCoralVoltage))
      .withName("IntakeCoral");
  }

  /**
   * Command to output coral
   * @return Command that runs roller at output speed for coral
   */
  public Command outputCoralCommand() {
    return run(() -> setVoltage(RollerConstants.OutputCoralVoltage))
      .withName("OutputCoral");
  }

  /**
   * Command to intake algae
   * @return Command that runs roller at intake speed for algae
   */
  public Command intakeAlgaeCommand() {
    return run(() -> setVoltage(RollerConstants.IntakeAlgaeVoltage))
      .withName("IntakeAlgae");
  }

  /**
   * Command to output algae
   * @return Command that runs roller at output speed for algae
   */
  public Command outputAlgaeCommand() {
    return run(() -> setVoltage(RollerConstants.OutputAlgaeVoltage))
      .withName("OutputAlgae");
  }

  /**
   * Command to hold algae with gentle power
   * @return Command that holds algae in place
   */
  public Command holdAlgaeCommand() {
    return run(() -> setVoltage(RollerConstants.HoldAlgaeVoltage))
      .withName("HoldAlgae");
  }
  
  /**
   * Command to stop the roller
   * @return Command that stops the roller motor
   */
  public Command stopCommand() {
    return runOnce(this::stop)
      .withName("StopRoller");
  }
  
  /**
   * Command to intake until beam break is triggered
   * @param voltage Intake voltage to use
   * @return Command that intakes until game piece is detected
   */
  public Command intakeUntilGamePieceCommand(double voltage) {
    return run(() -> setVoltage(voltage))
      .until(this::hasGamePiece)
      .finallyDo(this::stop)
      .withName("IntakeUntilGamePiece");
  }
  
  /**
   * Command to intake coral until beam break is triggered
   * @return Command that intakes coral until detected
   */
  public Command intakeCoralUntilDetectedCommand() {
    return intakeUntilGamePieceCommand(RollerConstants.IntakeCoralVoltage)
      .withName("IntakeCoralUntilDetected");
  }
  
  /**
   * Command to intake algae until beam break is triggered
   * @return Command that intakes algae until detected
   */
  public Command intakeAlgaeUntilDetectedCommand() {
    return intakeUntilGamePieceCommand(RollerConstants.IntakeAlgaeVoltage)
      .withName("IntakeAlgaeUntilDetected");
  }
  
  // ==================== Telemetry Methods ====================
  
  /**
   * Get current motor velocity in rotations per second
   * @return velocity in RPS
   */
  public double getVelocityRPS() {
    return rollerMotor.getVelocity().getValueAsDouble();
  }
  
  /**
   * Get current motor supply current
   * @return current in amps
   */
  public double getCurrent() {
    return rollerMotor.getSupplyCurrent().getValueAsDouble();
  }
  
  /**
   * Get current motor voltage
   * @return voltage in volts
   */
  public double getVoltage() {
    return rollerMotor.getMotorVoltage().getValueAsDouble();
  }
  
  /**
   * Get motor temperature
   * @return temperature in Celsius
   */
  public double getTemperature() {
    return rollerMotor.getDeviceTemp().getValueAsDouble();
  }

  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("RollerSubsystem");
    builder.addBooleanProperty("Has Game Piece", this::hasGamePiece, null);
    builder.addDoubleProperty("Velocity (RPS)", this::getVelocityRPS, null);
    builder.addDoubleProperty("Current (A)", this::getCurrent, null);
    builder.addDoubleProperty("Temperature (C)", this::getTemperature, null);
  }
}
