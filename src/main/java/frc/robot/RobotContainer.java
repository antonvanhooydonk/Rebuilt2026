// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feedback.FeedbackSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

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

  // The robot's subsystems and commands are defined here...
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final FeedbackSubsystem feedbackSubsystem = new FeedbackSubsystem(driverXbox);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
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
    // Initialize the default driving command to use during teleop.
    // See: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
    driveSubsystem.setDefaultCommand(driveSubsystem.driveCommand(
      () -> -driverXbox.getLeftY(), // left joystick Y axis will drive the robot along the field's X axis
      () -> -driverXbox.getLeftX(), // left joystick X axis will drive the robot along the field's Y axis
      () -> -driverXbox.getRightX() // right joystick X axis will rotate the robot CCW & CW 
    ));
  
    // Register named commands to be used in Pathplanner
    configureNamedCommands();

    // Configure the autonomous command chooser
    configureAutos();

    // Configure the event triggers & button bindings
    configureBindings();

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
    //   intakeSubsystem
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
  private void configureBindings() {
    // -------------------------------------------------------------------
    // Configure event triggers
    // -------------------------------------------------------------------
    // Initialize drive subsystem at start of autonomous
    new Trigger(RobotState::isAutonomous).onTrue(driveSubsystem.initAutonomousCommand());

    // Initialize for teleop at start of teleop period
    new Trigger(RobotState::isTeleop).onTrue(driveSubsystem.initTeleopCommand());

    // -------------------------------------------------------------------
    // Configure button triggers
    // -------------------------------------------------------------------
    // Reset odometry when start button is pushed
    driverXbox.start().onTrue(driveSubsystem.resetOdometryCommand().ignoringDisable(true));

    // Toggle field-relative driving when back button is pressed
    driverXbox.back().onTrue(driveSubsystem.toggleFieldRelativeModeCommand().ignoringDisable(true));

    // Set the arm to the home position
    driverXbox.a().onTrue(shooterSubsystem.outputFuelCommand(() -> 1.0));
    
    // Stop and lock wheels
    driverXbox.x().onTrue(driveSubsystem.stopAndLockWheelsCommand());

    // Set team colors on feedback subsystem
    driverXbox.b().onTrue(feedbackSubsystem.teamColorsCommand());
    
    // Manually toggle "slow" mode
    driverXbox.y().onTrue(driveSubsystem.toggleSlowModeCommand().ignoringDisable(true));
 
    // Configure operator controller 1 - blue buttons
    opController1.button(ControllerConstants.Operator1.ButtonBlue1).onTrue(
      Commands.none()
      .andThen(Commands.parallel(
        Commands.none(),
        Commands.none()
      ))
    );

    // Configure operator controller 1 - other buttons
    opController1.button(ControllerConstants.Operator1.ButtonYellow)
      .whileTrue(intakeSubsystem.intakeFuelCommand());
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
   * Use this to schedule scoring shift feedback based
   * on the game data passed from the DriverStation.
   * @param gameData the game-specific message from the DriverStation
   */
  public void scheduleScoringShiftCommand(char inactiveAlliance) {
    CommandScheduler.getInstance().schedule(feedbackSubsystem.scoringShiftCommand(inactiveAlliance));
  }
}
