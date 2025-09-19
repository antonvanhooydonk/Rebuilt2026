// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.RollerSubsystem;

/** An example command that uses an example subsystem. */
public class OutputCoralWithSensorCommand extends Command {
  private final RollerSubsystem rollerSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public OutputCoralWithSensorCommand(RollerSubsystem rollerSubsystem) {
    // Set properties
    this.rollerSubsystem = rollerSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rollerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if( rollerSubsystem.isBeamBreakBroke() )
      rollerSubsystem.outputCoral();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      
  }    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rollerSubsystem.stopRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {  
    return !rollerSubsystem.isBeamBreakBroke();
  }
}
