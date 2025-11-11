// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.arm.MoveArmCommand;
import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.commands.roller.OutputCoralCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.roller.RollerSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreLevel1CoralCommand extends SequentialCommandGroup {
  /** Creates a new ScoreLevel4Coral. */
  public ScoreLevel1CoralCommand(
    DriveSubsystem driveSubsystem,
    ElevatorSubsystem elevatorSubsystem, 
    ArmSubsystem armSubsystem,
    RollerSubsystem rollerSubsystem
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveArmCommand(armSubsystem, ArmConstants.CoralMovingPosition),
      new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.Lvl1Position),
      new MoveArmCommand(armSubsystem, ArmConstants.Lvl1Position),
      new OutputCoralCommand(rollerSubsystem).withTimeout(0.5),
      new MoveArmCommand(armSubsystem, ArmConstants.CoralMovingPosition),
      new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.HomePosition),
      new MoveArmCommand(armSubsystem, ArmConstants.HomePosition)
    );
  }
}
