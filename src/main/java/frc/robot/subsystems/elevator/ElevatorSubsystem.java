// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;

import frc.robot.Constants.CANConstants;
import frc.robot.util.Utils;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX leftElevatorMotor;
  private TalonFX rightElevatorMotor;
  private TalonFXConfiguration eleMotorConfig;
  private double targetPosition;
  private double zeroPoint;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    leftElevatorMotor= new TalonFX(CANConstants.LeftElevatorID);
    rightElevatorMotor = new TalonFX(CANConstants.RightElevatorID);

    eleMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode( NeutralModeValue.Brake))
      .withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio(0.08910703) )
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(70)
        .withSupplyCurrentLowerLimit(40)
        .withSupplyCurrentLowerTime(1)
        .withSupplyCurrentLimitEnable(true))
      .withVoltage(new VoltageConfigs()
        .withPeakForwardVoltage(16)
        .withPeakReverseVoltage(-16))
      .withMotionMagic(new MotionMagicConfigs()
        .withMotionMagicAcceleration(ElevatorConstants.MMAcceleration)
        .withMotionMagicCruiseVelocity(ElevatorConstants.MMVelocity)
        .withMotionMagicJerk(ElevatorConstants.MMJerk));
      // .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
      //   .withForwardSoftLimitEnable(true)
      //   .withForwardSoftLimitThreshold(0)
      //   .withReverseSoftLimitEnable(true)
      //   .withReverseSoftLimitThreshold(ElevatorConstants.MaxHeightPosition));

    eleMotorConfig.Slot0 = new Slot0Configs()
      .withKP(1)
      .withKD(0)
      .withKG(0.0)
      .withKA(0.0)
      .withKV(0.0)
      .withKS(0.0);   
  
    // Apply the configuration settings
    leftElevatorMotor.getConfigurator().apply(eleMotorConfig);
    rightElevatorMotor.getConfigurator().apply(eleMotorConfig);
    
    // We want the right motor to follow the left motor
    rightElevatorMotor.setControl(new Follower(CANConstants.LeftElevatorID, false));
    
    // Initialize the target position
    targetPosition = 0;

    // We start the match with the elevator at the bottom, 
    // so set the current position as the zero point.
    setZeroPoint();

    // Initialize dashboard values
    SmartDashboard.putData("Elevator", this);

    // Output initialization progress
    Utils.logInfo("Elevator subsystem intialized");
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic() {

  }

  /**
   * Get the elevator's current position
   * @return
   */
  public double getPosition() {
    return leftElevatorMotor.getPosition().getValueAsDouble();
  }

  /**
   * Get the elevator's target position
   * @return
   */
  public double getTargetPosition() {
    return targetPosition;
  }

  /**
   * Sets the elevator to a position relative to the 0 set by setZeroPoint(). 
   * @param height double that controls how many millimeters from the distance sensor
   */
  public void setTargetPosition(double position) {
    targetPosition = position + zeroPoint;
    MotionMagicVoltage request = new MotionMagicVoltage(targetPosition);
    leftElevatorMotor.setControl(request);
  }

  /**
   * Determines if the elevator is at the target position
   * @return
   */
  public boolean isAtTargetPosition() {
    return Math.abs(getPosition() - targetPosition) < ElevatorConstants.ErrorThreshold;
  }

  /**
   * Determines if the elevator should stop moving
   * 1. If the target position is below the zero point and the elevator is at or below the zero point
   * 2. If the target position is above the max height and the elevator is at or above the max height
   * 3. If the elevator is at the target position
   * @return
   */
  public boolean shouldStop() {
    if (
      (targetPosition <= zeroPoint && getPosition() <= zeroPoint) ||
      (targetPosition >= ElevatorConstants.MaxHeightPosition + zeroPoint && getPosition() >= ElevatorConstants.MaxHeightPosition + zeroPoint) || 
      isAtTargetPosition()
    ) {
      return true;
    };
    return false;
  }

  /**
   * Get the current target position of the PID controller of the elevator
   * @return
   */
  public double getPIDTarget() {
    return leftElevatorMotor.getClosedLoopReference().getValueAsDouble();
  }

  /**
   * Stops the elevator motors
   */
  public void stopElevator(){
    leftElevatorMotor.set(0);
  }

  /**
   * Get the value of zeroPoint
   * @return
   */
  public double getZeroPoint() {
    return zeroPoint;
  }

  /**
   * Sets the current position of the elevator as the zero point
   */
  public void setZeroPoint( ){
    zeroPoint = leftElevatorMotor.getPosition().getValueAsDouble();
  }

  /**
   * Get the value of elevator's closed loop error
   * @return
   */
  public double getError() {
    return leftElevatorMotor.getClosedLoopError().getValueAsDouble();
  }

  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Error", this::getError, null);
    builder.addDoubleProperty("Current Position", this::getPosition, null);
    builder.addDoubleProperty("Target Position", this::getTargetPosition, null);
    builder.addBooleanProperty("At Target Position", this::isAtTargetPosition, null);
  }
}
