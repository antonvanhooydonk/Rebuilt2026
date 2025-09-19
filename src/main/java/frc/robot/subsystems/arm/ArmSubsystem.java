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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConstants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ArmSubsystem extends SubsystemBase {
  private TalonFX armMotor;
  private TalonFXConfiguration armMotorConfig;
  private double targetPos;
  private double zeroPoint;

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
    armMotor.getConfigurator().apply(armMotorConfig);  
    
    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);

    // Cache the starting position as the zero point
    zeroPoint = armMotor.getPosition().getValueAsDouble();
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Pos", armMotor.getPosition().getValueAsDouble() );
    SmartDashboard.putNumber("Arm Target",targetPos );
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

    // update dashboard
    SmartDashboard.putNumber("Target Position", position);    
  }

  /**
   * Check if the arm is at the target position
   * 
   * @return
   */
  public boolean isArmAtPose() {
    boolean atPose = Math.abs(armMotor.getPosition().getValueAsDouble() - targetPos) < ElevatorConstants.ErrorThreshold; 
    SmartDashboard.putNumber("Arm Error",  armMotor.getClosedLoopError().getValueAsDouble() );
    SmartDashboard.putBoolean("Arm At Pose", atPose );
    return atPose;
  }

  /**
   * Check if the arm is in algae mode
   * 
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
}
