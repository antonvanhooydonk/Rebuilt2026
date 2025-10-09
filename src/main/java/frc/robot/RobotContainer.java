// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.Controller1Constants;
import frc.robot.Constants.Controller2Constants;
import frc.robot.commands.arm.MoveArmCommand;
import frc.robot.commands.elevator.MoveElevatorCommand;
import frc.robot.commands.roller.HoldAlgaeCommand;
import frc.robot.commands.roller.IntakeAlgaeCommand;
import frc.robot.commands.roller.IntakeCoralCommand;
import frc.robot.commands.roller.OutputAlgaeCommand;
import frc.robot.commands.roller.OutputCoralCommand;
import frc.robot.commands.rumble.RumbleControllerCommand;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.Utils;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final RollerSubsystem rollerSubsystem = new RollerSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(visionSubsystem);

  // Initialize our robot controllers
  private final CommandXboxController xboxController = new CommandXboxController(0);
  // private final CommandGenericHID xboxController = new CommandGenericHID(0);
  private final CommandGenericHID opController1 = new CommandGenericHID(1);
  private final CommandGenericHID opController2 = new CommandGenericHID(2);

  private SendableChooser<Command> autoCommandChooser = new SendableChooser<>();
  private SendableChooser<Command> delayCommandChooser = new SendableChooser<>();

  /** 
   * The container for the robot. 
   * Contains subsystems, OI devices, and commands. 
   */
  public RobotContainer() {
    // Initialize the default driving command.
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    // Drive field relative by default.
    // See: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    driveSubsystem.setDefaultCommand(new RunCommand(() -> 
      driveSubsystem.drive(
        -xboxController.getLeftY(),
        -xboxController.getLeftX(),
        -xboxController.getRightX()
      ), 
      driveSubsystem
    ));
    // driveSubsystem.setDefaultCommand(new RunCommand(() -> 
    //   driveSubsystem.drive(
    //     -xboxController.getRawAxis(1),
    //     -xboxController.getRawAxis(0),
    //     -xboxController.getRawAxis(4)
    //   ), 
    //   driveSubsystem
    // ));

    // Configure the autonomous command chooser
    configureAutos();

    // Configure the trigger/button bindings
    configureButtonBindings();

    // Silence joystick warnings during testing
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Configure the autonomous command chooser.
   * This will build the auto chooser using the AutoBuilder class, 
   * which pulls in all autos defined in the PathPlanner deploy folder.
   */
  public void configureAutos() {
    // Build the auto chooser and add it to the dashboard
    // This will use Commands.none() as the default option.
    autoCommandChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> DriverStation.isTest()
        ? stream // in test, show all autos
        : stream.filter(auto -> !auto.getName().toLowerCase().startsWith("test")) // in comp, filter out test autos
    );
    
    // Add auto chooser to dashboard
    SmartDashboard.putData("Auto Command", autoCommandChooser);

    // Configure the available auto delay options
    delayCommandChooser.setDefaultOption("No delay", Commands.none());
    delayCommandChooser.addOption("1.0 second", new WaitCommand(1.0));
    delayCommandChooser.addOption("1.5 seconds", new WaitCommand(1.5));
    delayCommandChooser.addOption("2.0 seconds", new WaitCommand(2.0));
    delayCommandChooser.addOption("2.5 seconds", new WaitCommand(2.5));
    delayCommandChooser.addOption("3.0 seconds", new WaitCommand(3.0));
    delayCommandChooser.addOption("3.5 seconds", new WaitCommand(3.5));
    delayCommandChooser.addOption("4.0 seconds", new WaitCommand(4.0));
    delayCommandChooser.addOption("4.5 seconds", new WaitCommand(4.5));
    delayCommandChooser.addOption("5.0 seconds", new WaitCommand(5.0));
    
    // Add delay chooser to dashboard
    SmartDashboard.putData("Auto Delay", delayCommandChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s 
   * subclasses for {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} 
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureButtonBindings() {
    // Configure Xbox controller bindings
    xboxController.start().onTrue(new RunCommand(() -> driveSubsystem.zeroHeading(), driveSubsystem).ignoringDisable(true));
    xboxController.back().onTrue(Commands.none());
    xboxController.a().onTrue(new MoveArmCommand(armSubsystem, ArmConstants.HomePosition));
    xboxController.b().onTrue(Commands.none());
    xboxController.x().onTrue(new MoveArmCommand(armSubsystem, ArmConstants.CoralMovingPosition));
    xboxController.y().onTrue(Commands.none());
    
    xboxController.leftBumper()
      .whileTrue(driveSubsystem.driveToPoseCommand(Utils.getLeftScoringPose(visionSubsystem, "FRONT_CAMERA"))
      .andThen(new RumbleControllerCommand(xboxController, 2.0)));
    xboxController.rightBumper()
      .whileTrue(driveSubsystem.driveToPoseCommand(Utils.getRightScoringPose(visionSubsystem, "FRONT_CAMERA"))
      .andThen(new RumbleControllerCommand(xboxController, 2.0)));

    xboxController.rightTrigger().onTrue(new RunCommand(() -> driveSubsystem.setSlowMode(true), driveSubsystem));
    xboxController.rightTrigger().onFalse(new RunCommand(() -> driveSubsystem.setSlowMode(false), driveSubsystem));

    // Configure operator controller 1 - blue buttons
    opController1.button(Controller1Constants.ButtonBlue1)
      .onTrue(new MoveArmCommand(armSubsystem, ArmConstants.AlgaeMovingPosition)
      .andThen(new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.HomePosition))
      .andThen(new MoveArmCommand(armSubsystem, ArmConstants.ProcessorAlgaePosition))
      .andThen(new RunCommand(() -> driveSubsystem.setSlowMode(false), driveSubsystem)));
    opController2.button(Controller2Constants.ButtonBlue2)
      .onTrue(new MoveArmCommand(armSubsystem, ArmConstants.AlgaeMovingPosition)
      .andThen(new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.LowerAlgaePosition))
      .andThen(new MoveArmCommand(armSubsystem, ArmConstants.LowerAlgaePosition))
      .andThen(new RunCommand(() -> driveSubsystem.setSlowMode(false), driveSubsystem)));
    opController2.button(Controller2Constants.ButtonBlue3)
      .onTrue(new MoveArmCommand(armSubsystem, ArmConstants.AlgaeMovingPosition)
      .andThen(new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.UpperAlgaePosition))
      .andThen(new MoveArmCommand(armSubsystem, ArmConstants.UpperAlgaePosition))
      .andThen(new RunCommand(() -> driveSubsystem.setSlowMode(false), driveSubsystem)));
    opController2.button(Controller2Constants.ButtonBlue4)
      .onTrue(new MoveArmCommand(armSubsystem, ArmConstants.AlgaeMovingPosition)
      .andThen(new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.NetAlgaePosition))
      .andThen(new MoveArmCommand(armSubsystem, ArmConstants.NetAlgaePosition))
      .andThen(new RunCommand(() -> driveSubsystem.setSlowMode(true), driveSubsystem)));

    // Configure operator controller 1 - other buttons
    opController1.button(Controller1Constants.ButtonYellow)
      .whileTrue(new IntakeCoralCommand(rollerSubsystem));
    opController1.button(Controller1Constants.ButtonGreen)
      .whileTrue(new OutputCoralCommand(rollerSubsystem));
    opController1.button(Controller1Constants.ButtonPlayer1)
      .whileTrue(new IntakeAlgaeCommand(rollerSubsystem));
    opController1.button(Controller1Constants.ButtonPlayer2)
      .whileTrue(new OutputAlgaeCommand(rollerSubsystem));
    opController1.button(Controller1Constants.ButtonBlack2)
      .whileTrue(new HoldAlgaeCommand(rollerSubsystem));

    // Configure operator controller 2 - red buttons
    opController2.button(Controller2Constants.ButtonRed1)
      .onTrue(new MoveArmCommand(armSubsystem, ArmConstants.CoralMovingPosition)
      .andThen(new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.HomePosition))
      .andThen(new MoveArmCommand(armSubsystem, ArmConstants.HomePosition))
      .andThen(new RunCommand(() -> driveSubsystem.setSlowMode(false), driveSubsystem)));
    opController2.button(Controller2Constants.ButtonRed2)
      .onTrue(new MoveArmCommand(armSubsystem, ArmConstants.CoralMovingPosition)
      .andThen(new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.Lvl1Position))
      .andThen(new MoveArmCommand(armSubsystem, ArmConstants.Lvl1Position))
      .andThen(new RunCommand(() -> driveSubsystem.setSlowMode(true), driveSubsystem)));
    opController2.button(Controller2Constants.ButtonRed3)
      .onTrue(new MoveArmCommand(armSubsystem, ArmConstants.CoralMovingPosition)
      .andThen(new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.Lvl2Position))
      .andThen(new MoveArmCommand(armSubsystem, ArmConstants.Lvl2Position))
      .andThen(new RunCommand(() -> driveSubsystem.setSlowMode(true), driveSubsystem)));
    opController2.button(Controller2Constants.ButtonRed4)
      .onTrue(new MoveArmCommand(armSubsystem, ArmConstants.CoralMovingPosition)
      .andThen(new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.Lvl3Position))
      .andThen(new MoveArmCommand(armSubsystem, ArmConstants.Lvl3Position))
      .andThen(new RunCommand(() -> driveSubsystem.setSlowMode(true), driveSubsystem)));
    opController2.button(Controller2Constants.ButtonRed5)
      .onTrue(new MoveArmCommand(armSubsystem, ArmConstants.CoralMovingPosition)
      .andThen(new MoveElevatorCommand(elevatorSubsystem, ElevatorConstants.Lvl4Position))
      .andThen(new MoveArmCommand(armSubsystem, ArmConstants.Lvl4Position))
      .andThen(new RunCommand(() -> driveSubsystem.setSlowMode(true), driveSubsystem)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      delayCommandChooser.getSelected(),  // run the selected delay command
      autoCommandChooser.getSelected()    // then run the selected auto command
    );
  }

  /**
   * Set the motors to brake or coast mode.
   * @param brake True to enable motor brake, false for coast
   */
  public void setMotorBrake(boolean brake) {
    driveSubsystem.setMotorBrake(brake);
  }
}
