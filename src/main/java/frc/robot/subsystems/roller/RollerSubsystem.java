// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.DIOConstants;

public class RollerSubsystem extends SubsystemBase {
  private TalonFX rollerMotor;
  private TalonFXConfiguration rollerMotorConfig;
  private DigitalInput intakeBeamBreakSwitch;
    
  /** Creates a new ElevatorSubsystem. */
  public RollerSubsystem() {
    intakeBeamBreakSwitch = new DigitalInput(DIOConstants.BeamBreakSensorPort);
    rollerMotor= new TalonFX(CANConstants.RollerID);
   
    // Configure the motor controller
    rollerMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode( NeutralModeValue.Brake))
      .withSlot0(new Slot0Configs()
        .withKP(1)
        .withKS(0)
        .withKA(0)
        .withKV(0))
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(70)
        .withSupplyCurrentLimitEnable(true));

    // Apply the configuration to the motor controller
    rollerMotor.getConfigurator().apply(rollerMotorConfig);

    // Initialize dashboard values
    SmartDashboard.putData("Roller", this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
 
  /**
   * Check if the beam break sensor is broken
   * @return
   */
  public boolean isBeamBreakBroke() {
    return !intakeBeamBreakSwitch.get();
  }

  /**
   * Intake coral at a fixed speed
   */
  public void intakeCoral() {
    rollerMotor.set(RollerConstants.IntakeCoralSpeed);
  }

  /**
   * Output coral at a fixed speed
   */
  public void outputCoral() {
    rollerMotor.set(RollerConstants.OutputCoralSpeed);
  } 

  /**
   * Intake algae at a fixed speed
   */
  public void intakeAlgae() {
    rollerMotor.set(RollerConstants.IntakeAlgaeSpeed);
  }

  /**
   * Output algae at a fixed speed
   */
  public void outputAlgae() {
    rollerMotor.set(RollerConstants.OutputAlgaeSpeed);
  } 

  /**
   * Hold algae
   */
  public void holdAlgae() {
    rollerMotor.set(RollerConstants.HoldAlgaeSpeed);
  } 
  
  /**
   * Stop the roller motor
   */
  public void stopRoller() {
    rollerMotor.set(0);
  }

  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Beam Broken", this::isBeamBreakBroke, null);
  }
}
