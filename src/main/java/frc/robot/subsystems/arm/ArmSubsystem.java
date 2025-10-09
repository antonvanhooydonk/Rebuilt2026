// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.Utils;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX armMotor;
  private TalonFXConfiguration armMotorConfig;
  private double targetPos;
  private double zeroPoint;
  private boolean algaeMode = false;

  public ArmSubsystem() {
    // Initialize motor
    armMotor = new TalonFX(CANConstants.ArmID);
    
    // Configure motor
    armMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode( NeutralModeValue.Brake))
      .withSlot0(new Slot0Configs()
        .withKP(1)
        .withKS(0)
        .withKA(0)
        .withKV(0))
        .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(30)
        .withSupplyCurrentLimitEnable(true));

    // Apply the configuration to the motor
    armMotor.getConfigurator().apply(armMotorConfig);  

    // Cache the starting position as the zero point
    zeroPoint = armMotor.getPosition().getValueAsDouble();
    
    // Initialize dashboard values
    SmartDashboard.putData("Arm", this);

    // Output initialization progress
    Utils.logInfo("Arm subsystem intialized");
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic() {
    isArmAtPose();
  }

  /**
   * Set the arm to a specific position
   * 
   * @param position
   */
  public void setArmPosition(double position) {
    // Calculate target position adjusting for the zero point offset
    targetPos = position + zeroPoint;

    // Command the motor to the desired position
    PositionVoltage voltReq = new PositionVoltage(0)
                                  .withPosition(targetPos)
                                  .withVelocity(0.1);

    // Send command to motor
    armMotor.setControl(voltReq);
  }

  /**
   * Check if the arm is at the target position
   * 
   * @return
   */
  public boolean isArmAtPose() {
    return Math.abs(armMotor.getPosition().getValueAsDouble() - targetPos) < ElevatorConstants.ErrorThreshold; 
  }

  /**
   * Get the current position of the arm
   * @return
   */
  public double getPosition() {
    return armMotor.getPosition().getValueAsDouble();
  }

  /**
   * Get the target position of the arm adjusted for zero point
   * @return
   */
  public double getTargetPosition() {
    return targetPos - zeroPoint;
  }

  /**
   * Get the current velocity of the arm
   * @return
   */
  public double getVelocity() {
    return armMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Get the current error of the closed loop PID control
   * @return
   */
  public double getError() {
    return armMotor.getClosedLoopError().getValueAsDouble();
  }

  /**
   * Check if the arm is in algae mode 
   * @return
   */
  public boolean isArmInAlgaeMode() {
    if (
      targetPos == ArmConstants.ProcessorAlgaePosition ||
      targetPos == ArmConstants.LowerAlgaePosition ||
      targetPos == ArmConstants.UpperAlgaePosition ||
      targetPos == ArmConstants.NetAlgaePosition
    ) {
      return true;
    }

    // default false
    return false;
  }

  /**
   * Check if the arm is in algae mode
   * @return
   */
  public boolean isAlgaeMode() {
    return algaeMode;
  }

  /**
   * Set algae mode
   * @param mode True if in algae mode, false otherwise
   */
  public void setAlgaeMode(boolean mode) {
    algaeMode = mode;
    if (algaeMode) {
    
    }
  }

  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Current Position", this::getPosition, null);
    builder.addDoubleProperty("Target Position", this::getTargetPosition, null);
    builder.addDoubleProperty("Velocity", this::getVelocity, null);
    builder.addBooleanProperty("Algae Mode", this::isArmInAlgaeMode, null);
    builder.addDoubleProperty("Error", this::getError, null);
    builder.addBooleanProperty("At Pose", this::isArmAtPose, null);
  }
}
