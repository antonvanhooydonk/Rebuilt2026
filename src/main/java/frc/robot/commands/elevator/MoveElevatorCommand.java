// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.elevator.ElevatorSubsystem;

/** An example command that uses an example subsystem. */
public class MoveElevatorCommand extends Command {
  private final ElevatorSubsystem elevatorSubsystem;
  private final double targetPosition;

  /**
   * Creates a new ExampleCommand.
   * @param subsystem The subsystem used by this command.
   * @param targetPosition The target position for the elevator.
   */
  public MoveElevatorCommand(ElevatorSubsystem elevatorSubsystem, double targetPosition) {
    // Set properties
    this.elevatorSubsystem = elevatorSubsystem;
    this.targetPosition = targetPosition;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setTargetPosition(targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {  
    return elevatorSubsystem.shouldStop();
  }
}
