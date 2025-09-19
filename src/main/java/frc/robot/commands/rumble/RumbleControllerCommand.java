// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.rumble;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class RumbleControllerCommand extends Command {
  private final CommandXboxController controller;
  private final double duration;
  private final Timer timer = new Timer();

  /**
   * Creates a new RumbleControllerCommand.
   *
   * @param controller The subsystem used by this command.
   * @param duration The duratoin in seconds to rumble the controller for
   */
  public RumbleControllerCommand(CommandXboxController controller, double duration) {
    // Set properties
    this.controller = controller;
    this.duration = duration;

    // Use addRequirements() here to declare subsystem dependencies.
    // n/a
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setRumble(RumbleType.kLeftRumble, 1.0);
    controller.setRumble(RumbleType.kRightRumble, 1.0);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controller.setRumble(RumbleType.kLeftRumble, 0.0);
    controller.setRumble(RumbleType.kRightRumble, 0.0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {  
    return timer.hasElapsed(duration);
  }
}
