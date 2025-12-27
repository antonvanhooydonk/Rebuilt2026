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
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.feedback.FeedbackSubsystem;
import frc.robot.subsystems.roller.RollerSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.Utils;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot 
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Initialize our controllers
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  private final CommandGenericHID opController1 = new CommandGenericHID(1);
  private final CommandGenericHID opController2 = new CommandGenericHID(2);

  // The robot's subsystems and commands are defined here...
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final FeedbackSubsystem feedbackSubsystem = new FeedbackSubsystem(driverXbox);
  private final RollerSubsystem rollerSubsystem = new RollerSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem(visionSubsystem);

  // Auto choosers
  private SendableChooser<Command> autoCommandChooser = new SendableChooser<>();
  private SendableChooser<Command> delayCommandChooser = new SendableChooser<>();

  /** 
   * The container for the robot. 
   * Contains subsystems, IO devices, and commands. 
   */
  public RobotContainer() {
    // Initialize the default driving command.
    // The left stick controls translation of the robot.
    // Turning/rotatiton is controlled by the X axis of the right stick.
    // Robot drives field relative by default.
    // See: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    driveSubsystem.setDefaultCommand(Commands.run(() -> 
      driveSubsystem.drive(
        -driverXbox.getLeftY(),
        -driverXbox.getLeftX(),
        -driverXbox.getRightX()
      ), 
      driveSubsystem
    ));
  
    // Register named commands to be used in Pathplanner
    configureNamedCommands();

    // Configure the autonomous command chooser
    configureAutos();

    // Configure the event triggers & button bindings
    configureTriggers();

    // Silence joystick warnings during testing
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Register named commands to be used in PathPlanner autos.
   * Register named commands before the creation of any PathPlanner Autos or Paths. 
   * It is recommended to do this in RobotContainer, after subsystem 
   * initialization, but before the creation of any other commands.
   */
  public void configureNamedCommands() {
    // NamedCommands.registerCommand("ScoreLevel1Coral", new ScoreLevel1CoralCommand(
    //   driveSubsystem,
    //   elevatorSubsystem,
    //   armSubsystem,
    //   rollerSubsystem
    // ));
  }

  /**
   * Register named commands and configure the autonomous command chooser.
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
    delayCommandChooser.addOption("1.0 second", Commands.waitSeconds(1.0));
    delayCommandChooser.addOption("1.5 seconds", Commands.waitSeconds(1.5));
    delayCommandChooser.addOption("2.0 seconds", Commands.waitSeconds(2.0));
    delayCommandChooser.addOption("2.5 seconds", Commands.waitSeconds(2.5));
    delayCommandChooser.addOption("3.0 seconds", Commands.waitSeconds(3.0));
    delayCommandChooser.addOption("3.5 seconds", Commands.waitSeconds(3.5));
    delayCommandChooser.addOption("4.0 seconds", Commands.waitSeconds(4.0));
    delayCommandChooser.addOption("4.5 seconds", Commands.waitSeconds(4.5));
    delayCommandChooser.addOption("5.0 seconds", Commands.waitSeconds(5.0));
    
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
  private void configureTriggers() {
    // -------------------------------------------------------------------
    // Configure trigger -> command mappings
    // -------------------------------------------------------------------    
    // Trigger feedback when game piece acquired
    new Trigger(rollerSubsystem::hasGamePiece).onTrue(feedbackSubsystem.gamePieceAcquiredCommand());
  
    // -------------------------------------------------------------------
    // Configure button triggers
    // -------------------------------------------------------------------
    // Zero gyro yaw when start button is pushed
    driverXbox.start().onTrue(
      Commands.runOnce(() -> driveSubsystem.zeroHeading(), driveSubsystem).ignoringDisable(true)
    );

    // Toggle field-relative driving when back button is pressed
    driverXbox.back().onTrue(
      Commands.runOnce(() -> driveSubsystem.setFieldRelative(!driveSubsystem.isFieldRelative()), driveSubsystem)
    );

    // Score at coral level 1
    driverXbox.a().onTrue(armSubsystem.moveToHomeCommand());
    
    // Score at coral level 2
    driverXbox.x().onTrue(Commands.none());

    // Score at coral level 3
    driverXbox.b().onTrue(armSubsystem.moveToCoralMoveCommand());
    
    // Score at coral level 4
    driverXbox.y().onTrue(Commands.none());
    
    // Drive to left scoring pose of the current camera target
    driverXbox.leftBumper().whileTrue(
      driveSubsystem.driveToPoseCommand(Utils.getLeftScoringPose(visionSubsystem, "FRONT_CAMERA"))
      .andThen(feedbackSubsystem.doubleRumbleCommand())
    );
    
    // Drive to right scoring pose of the current camera target
    driverXbox.rightBumper().whileTrue(
      driveSubsystem.driveToPoseCommand(Utils.getRightScoringPose(visionSubsystem, "FRONT_CAMERA"))
      .andThen(feedbackSubsystem.doubleRumbleCommand())
    );

    // Manually toggle "slow" mode
    driverXbox.rightTrigger().onTrue(Commands.runOnce(() -> 
      driveSubsystem.setSlowMode(!driveSubsystem.isSlowMode()),
      driveSubsystem
    ));    
 
    // Configure operator controller 1 - blue buttons
    opController1.button(ControllerConstants.Operator1.ButtonBlue1).onTrue(
      armSubsystem.moveToAlgaeMoveCommand()
      .andThen(elevatorSubsystem.moveToGroundCommand())
      .andThen(armSubsystem.moveToAlgaeProcessorCommand())
      .andThen(Commands.run(() -> driveSubsystem.setSlowMode(false), driveSubsystem))
    );
    opController2.button(ControllerConstants.Operator2.ButtonBlue2).onTrue(
      armSubsystem.moveToAlgaeMoveCommand()
      .andThen(elevatorSubsystem.moveToAlgaeOneCommand())
      .andThen(armSubsystem.moveToAlgaeOneCommand())
      .andThen(Commands.run(() -> driveSubsystem.setSlowMode(false), driveSubsystem))
    );
    opController2.button(ControllerConstants.Operator2.ButtonBlue3).onTrue(
      armSubsystem.moveToAlgaeMoveCommand()
      .andThen(elevatorSubsystem.moveToAlgaeTwoCommand())
      .andThen(armSubsystem.moveToAlgaeTwoCommand())
      .andThen(Commands.run(() -> driveSubsystem.setSlowMode(false), driveSubsystem))
    );
    opController2.button(ControllerConstants.Operator2.ButtonBlue4).onTrue(
      armSubsystem.moveToAlgaeMoveCommand()
      .andThen(elevatorSubsystem.moveToAlgaeThreeCommand())
      .andThen(armSubsystem.moveToAlgaeNetCommand())
      .andThen(Commands.run(() -> driveSubsystem.setSlowMode(true), driveSubsystem))
    );

    // Configure operator controller 1 - other buttons
    opController1.button(ControllerConstants.Operator1.ButtonYellow)
      .whileTrue(rollerSubsystem.intakeCoralUntilDetectedCommand());
    opController1.button(ControllerConstants.Operator1.ButtonGreen)
      .whileTrue(rollerSubsystem.outputCoralCommand());
    opController1.button(ControllerConstants.Operator1.ButtonPlayer1)
      .whileTrue(rollerSubsystem.intakeAlgaeCommand());
    opController1.button(ControllerConstants.Operator1.ButtonPlayer2)
      .whileTrue(rollerSubsystem.outputAlgaeCommand());
    opController1.button(ControllerConstants.Operator1.ButtonBlack2)
      .whileTrue(rollerSubsystem.holdAlgaeCommand());

    // Configure operator controller 2 - red buttons
    opController2.button(ControllerConstants.Operator2.ButtonRed1).onTrue(
      armSubsystem.moveToCoralMoveCommand()
      .andThen(elevatorSubsystem.moveToGroundCommand())
      .andThen(armSubsystem.moveToHomeCommand())
      .andThen(Commands.run(() -> driveSubsystem.setSlowMode(false), driveSubsystem))
    );
    opController2.button(ControllerConstants.Operator2.ButtonRed2).onTrue(
      armSubsystem.moveToCoralMoveCommand()
      .andThen(elevatorSubsystem.moveToCoralOneCommand())
      .andThen(armSubsystem.moveToCoralOneCommand())
      .andThen(Commands.run(() -> driveSubsystem.setSlowMode(true), driveSubsystem))
    );
    opController2.button(ControllerConstants.Operator2.ButtonRed3).onTrue(
      armSubsystem.moveToCoralMoveCommand()
      .andThen(elevatorSubsystem.moveToCoralTwoCommand())
      .andThen(armSubsystem.moveToCoralTwoCommand())
      .andThen(Commands.run(() -> driveSubsystem.setSlowMode(true), driveSubsystem))
    );
    opController2.button(ControllerConstants.Operator2.ButtonRed4).onTrue(
      armSubsystem.moveToCoralMoveCommand()
      .andThen(elevatorSubsystem.moveToCoralThreeCommand())
      .andThen(armSubsystem.moveToCoralThreeCommand())
      .andThen(Commands.run(() -> driveSubsystem.setSlowMode(true), driveSubsystem))
    );
    opController2.button(ControllerConstants.Operator2.ButtonRed5).onTrue(
      armSubsystem.moveToCoralMoveCommand()
      .andThen(elevatorSubsystem.moveToCoralFourCommand())
      .andThen(armSubsystem.moveToCoralFourCommand())
      .andThen(Commands.run(() -> driveSubsystem.setSlowMode(true), driveSubsystem))
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.sequence(
      delayCommandChooser.getSelected(), // run the selected delay command
      autoCommandChooser.getSelected()   // then run the selected auto command
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
