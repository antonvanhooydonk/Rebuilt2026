// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem.VisionMeasurement;
import frc.robot.util.Utils;

/**
 * Swerve drive subsystem using four SwerveModules.
 * 
 * Integrated functionality:
 * 1. PathPlanner 
 * 2. PhotonVision integration via VisionSubsystem
 * 3. Slew rate limiting for smoother joystick control
 */
public class DriveSubsystem extends SubsystemBase {  
  // Slew rate limiters to make joystick inputs smoother
  private final SlewRateLimiter xSpeedLimiter;
  private final SlewRateLimiter ySpeedLimiter;
  private final SlewRateLimiter rSpeedLimiter;

  // Gyroscope
  private final NavX2Gyro gyro;

  // Swerve modules - ensure indexed order is 0 = FL, 1 = FR, 2 = BL, 3 = BR
  private final SwerveModule[] modules;

  // Kinematics and odometry
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;

  // Setpoint generator
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint driveSetpoint;

  // Vision subsystem reference
  private final VisionSubsystem visionSubsystem;

  // Field visualization
  private final Field2d field2d = new Field2d();

  // Current robot state
  private boolean fieldRelative = true;
  private boolean slowMode = false;

  /**
   * Creates a new SwerveSubsystem
   * @param visionSubsystem The vision subsystem for pose estimation
   */
  public DriveSubsystem(VisionSubsystem visionSubsystem) {
    // Store vision subsystem reference
    this.visionSubsystem = visionSubsystem;

    // Initialize the slew rate limiters
    xSpeedLimiter = new SlewRateLimiter(DriveConstants.kTranslationalSlewRateLimit);
    ySpeedLimiter = new SlewRateLimiter(DriveConstants.kTranslationalSlewRateLimit);
    rSpeedLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRateLimit);

    // Initialize gyro
    gyro = new NavX2Gyro();

    // Initialize swerve modules
    modules = new SwerveModule[] {
      DriveConstants.kFrontLeftSwerveModule,
      DriveConstants.kFrontRightSwerveModule,
      DriveConstants.kBackLeftSwerveModule,
      DriveConstants.kBackRightSwerveModule
    };
    
    // Initialize swerve drive kinematics
    kinematics = new SwerveDriveKinematics(DriveConstants.kSwerveModuleTranslations);
    
    // Initialize pose estimator for vision integration
    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics,           // The kinematics object
      gyro.getAngle(),      // The current gyro angle
      getModulePositions(), // The current module positions
      new Pose2d()          // Will be updated by auto and vision
    );

    // Initialize the swerve setpoint generator
    setpointGenerator = new SwerveSetpointGenerator(
      DriveConstants.kRobotConfig, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
      DriveConstants.kMaxAngularSpeedRadsPerSecond // The max rotation velocity of a swerve module in radians per second.
    );
    
    // Initialize the "last" setpoint
    driveSetpoint = new SwerveSetpoint(
      new ChassisSpeeds(), 
      getModuleStates(), 
      DriveFeedforwards.zeros(4)
    );

    // Configure AutoBuilder for path following
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier
      this::driveWithChassisSpeeds, // Method that will drive the robot given ChassisSpeeds
      new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID constants
      ),
      DriveConstants.kRobotConfig, // Robot configuration
      Utils::isRedAlliance, // Method to flip path based on alliance color
      this // Reference to this subsystem to set requirements
    );

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD THEM HERE
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    // Reset odometry to origin on boot
    resetOdometry();

    // Add data to dashboard
    SmartDashboard.putData("Drive", this);
    SmartDashboard.putData("Drive/Field", field2d);
    SmartDashboard.putData("Drive/Swerve", builder -> {
      builder.setSmartDashboardType("SwerveDrive");
      builder.addDoubleProperty("Front Left Angle", () -> Utils.showDouble(modules[0].getState().angle.getRadians()), null);
      builder.addDoubleProperty("Front Left Velocity", () -> Utils.showDouble(modules[0].getState().speedMetersPerSecond), null);
      builder.addDoubleProperty("Front Right Angle", () -> Utils.showDouble(modules[1].getState().angle.getRadians()), null);
      builder.addDoubleProperty("Front Right Velocity", () -> Utils.showDouble(modules[1].getState().speedMetersPerSecond), null);
      builder.addDoubleProperty("Back Left Angle", () -> Utils.showDouble(modules[2].getState().angle.getRadians()), null);
      builder.addDoubleProperty("Back Left Velocity", () -> Utils.showDouble(modules[2].getState().speedMetersPerSecond), null);
      builder.addDoubleProperty("Back Right Angle", () -> Utils.showDouble(modules[3].getState().angle.getRadians()), null);
      builder.addDoubleProperty("Back Right Velocity", () -> Utils.showDouble(modules[3].getState().speedMetersPerSecond), null);
      builder.addDoubleProperty("Robot Angle", () -> Utils.showDouble(getHeading().getRadians()), null);
    });

    // Output initialization progress
    Utils.logInfo("Drive subsystem initialized");
  }

  @Override
  public void periodic() {
    // Run the gyro's periodic method
    gyro.periodic();

    // Call each swerve module's periodic before updating the pose estimate since 
    // they calculate and cache their state/positions in their periodics and the 
    // pose estimatore uses the cached postions in its update.
    for (var module : modules) {
      module.periodic();
    }

    // Update odometry
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), gyro.getAngle(), getModulePositions());
    
    // Add vision measurements to the pose estimator
    addVisionMeasurements();
    
    // Update field visualization
    field2d.setRobotPose(getPose());
  }

  /**
   * Initializes the drive subsystem at the start of the autonomous phase.
   * Should be called from Robot.autonomousInit() or a command scheduler binding.
   */
  private void initAutonomous() {
    // Reset odometry to origin as a safety default if no auto path is selected.
    // PathPlanner will immediately override this with the proper starting 
    // pose when auto starts.
    resetOdometry();

    // Reset last setpoint for setpoint generator
    driveSetpoint = new SwerveSetpoint(
      new ChassisSpeeds(), 
      getModuleStates(), 
      DriveFeedforwards.zeros(4)
    );

    // Reset slew rate limiters to prevent jumps
    xSpeedLimiter.reset(0);
    ySpeedLimiter.reset(0);
    rSpeedLimiter.reset(0);
    
    // Set motors to brake mode for match
    setMotorBrake(true);
    
    // Reset to default driving modes
    fieldRelative = true;
    slowMode = false;

    // Log initialization
    Utils.logInfo("Drive subsystem initialized for autonomous");
  }

  /**
   * Initializes the drive subsystem at the start of the teleop phase.
   * Should be called from Robot.teleopInit() or a command scheduler binding.
   */
  private void initTeleop() {
    // DO NOT reset gyro, encoders, or pose estimator.
    // We want to maintain position from autonomous.
    
    // DO NOT reset driveSetpoint - let it continue from autonomous.
    // The setpoint generator needs continuity of module states.

    // Reset slew rate limiters for smooth joystick control
    xSpeedLimiter.reset(0);
    ySpeedLimiter.reset(0);
    rSpeedLimiter.reset(0);
    
    // Ensure brake mode is enabled
    setMotorBrake(true);
    
    // Reset to default driving modes
    fieldRelative = true;
    slowMode = false;
  
    // Log initialization
    Utils.logInfo("Drive subsystem initialized for teleop");
  }

  /**
   * Checks if vision subsystem is available
   * @return Boolean indicating if vision is enabled
   */
  private boolean isVisionEnabled() {
    return visionSubsystem != null && visionSubsystem.isEnabled();
  }

  /**
   * Updates pose estimation using vision measurements from VisionSubsystem
   */
  private void addVisionMeasurements() {
    // Exit early if vision subsystem is not available or is disabled
    if (!isVisionEnabled()) return;

    // Get current time
    double now = Timer.getFPGATimestamp();
    
    // Get all latest vision measurements
    List<VisionMeasurement> measurements = visionSubsystem.getLatestMeasurements();

    // Process each vision measurement
    for (VisionMeasurement measurement : measurements) {
      try {
        Pose2d visionPose = measurement.getPose();
        double timestamp = measurement.getTimestampSeconds();
        double[] stdDevs = measurement.getStandardDeviations();

        // Reject timestamps older than 0.3 seconds
        if ((now - timestamp) > 0.3) {
          continue;
        }

        // Reject timestamps from the future
        if (timestamp > now) {
          continue;
        }

        // Get the robot's current pose
        Pose2d robotPose = getPose();

        // Reject large translation jumps. 
        // Typical thresholds: 
        //    Auto:   0.5m – 0.75m
        //    Teleop: 1.0m – 1.50m
        if (robotPose.getTranslation().getDistance(visionPose.getTranslation()) > 1.0) {
          continue;
        }

        // Reject rotations > 30 degrees
        if (Math.abs(robotPose.getRotation().minus(visionPose.getRotation()).getDegrees()) > 30.0) {
          continue;
        }

        // Reject if the robot is rotating very fast (> 3 rad/s)
        if (Math.abs(getChassisSpeeds().omegaRadiansPerSecond) > 3.0) {
          continue;
        }

        // Reject if the robot is moving very fast (> 3 m/s)
        ChassisSpeeds speeds = getChassisSpeeds();
        if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > 3.0) {
          continue;
        }

        // Reject poses outside the field boundaries
        if (!visionSubsystem.isPoseOnField(visionPose)) {
          continue;
        }

        // If we make it here => add vision measurement to pose estimator
        poseEstimator.addVisionMeasurement(
          visionPose,
          timestamp,
          VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2])
        );          
      } catch (Exception e) {
        Utils.logError("Error adding vision measurement: " + e.getMessage());
      }
    }
  }

  /**
   * Drive the robot using the given robot-relative chassis speeds.
   * @param speeds The desired robot-relative speeds
   */
  private void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    // Create a new drive setpoint based on the drive setpoint & the desired chassis speeds.
    // NOTE: Do NOT discretize speeds, call SwerveDriveKinematics.desaturateWheelSpeeds(), 
    // before or after using the setpoint generator, as it will discretize them for you.
    driveSetpoint = setpointGenerator.generateSetpoint(
      driveSetpoint,                        // The last calculated setpoint
      speeds,                               // The desired target speeds
      DriveConstants.kPeriodicTimeSeconds   // The loop time of the robot code, in seconds
    );

    // Set each swerve module state directly from the next calculated setpoint
    SwerveModuleState[] desiredStates = driveSetpoint.moduleStates();
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(desiredStates[i]);
    }
  }

  /**
   * Sets the desired state for each swerve module. Do not call this from driveWithChassisSpeeds.
   * @param desiredStates Array of desired SwerveModuleStates
   */
  private void setModuleStates(SwerveModuleState[] desiredStates) {
    // Normalize wheel speeds 
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    
    // Set each module state
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(desiredStates[i]);
    }
  }
  
  /**
   * Set each swerve module to brake/coast mode
   * @param brake True to enable motor brake, false for coast
   */
  private void setMotorBrake(boolean brake) {
    for (var module : modules) {
      module.setMotorBrake(brake);
    }
  }

  /**
   * Stop the robot
   */
  private void stop() {
    for (var module : modules) {
      module.stop();
    }
  }

  /**
   * Stop the robot and sets wheel positions to an X formation 
   * to resist being pushed by other robots while on defense.
   */
  private void stopAndLockWheels() {
    // Stop the robot
    stop();

    // Set modules to X formation
    SwerveModuleState[] states = {
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)), // Front Left
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),        // Front Right
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),        // Back Left
      new SwerveModuleState(0, Rotation2d.fromDegrees(45))  // Back Right
    };
    setModuleStates(states);
  }

  /**
   * Resets odometry by zeroing the gyroscope heading and resetting the pose estimator.
   * NOTE: Should never need to call this if vision is working properly.
   */
  private void resetOdometry() {
    resetOdometry(new Pose2d());
  }

  /**
   * Resets odometry to a specific pose and resets module encoders.
   * NOTE: Should never need to call this if vision is working properly.
   */
  private void resetOdometry(Pose2d pose) {
    // Handle null pose
    if (pose == null) {
      pose = new Pose2d();
    }

    // Reset gyro
    if (!isVisionEnabled()) {
      gyro.resetToAngle(pose.getRotation());
    }

    // Reset pose estimator to origin with 0° heading
    resetPose(pose);
  }

  /**
   * Gets the current heading of the robot from the pose estimator.
   * This is what should be fed into the drive (auto and teleop)
   * functions when calculating chassis speeds & module states.
   * @return Current heading as a Rotation2d
   */
  private Rotation2d getHeading() {
    return poseEstimator.getEstimatedPosition().getRotation(); 
  }

  /**
   * Gets the current pose of the robot
   * @return Current pose 
   */
  private Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the pose to a given pose
   * @param pose New pose
   */
  private void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(gyro.getAngle(), getModulePositions(), pose);
  }
  
  /**
   * Gets the current chassis speeds
   * @return Current ChassisSpeeds
   */
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Gets the current module states
   * @return Array of current SwerveModuleStates
   */
  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      modules[0].getState(),  // front left
      modules[1].getState(),  // front right
      modules[2].getState(),  // back left
      modules[3].getState()   // back right
    };
  }

  /**
   * Gets the current module positions
   * @return Array of current SwerveModulePositions
   */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      modules[0].getPosition(), // front left
      modules[1].getPosition(), // front right
      modules[2].getPosition(), // back left
      modules[3].getPosition()  // back right
    };
  }

  // ============================================================
  // State Query Methods
  // ============================================================

  /**
   * Gets whether field-relative driving is enabled
   * @return Field-relative status
   */
  public boolean isFieldRelative() {
    return fieldRelative;
  }

  /**
   * Gets whether slow mode is enabled
   * @return Slow mode status
   */
  public boolean isSlowMode() {
    return slowMode;
  }

  // ============================================================
  // Command Factory Methods
  // ============================================================

  /**
   * Command factory for binding to initAutonomous
   */
  public Command initAutonomousCommand() {
    return runOnce(this::initAutonomous);
  }

  /**
   * Command factory for binding to initTeleop
   */
  public Command initTeleopCommand() {
    return runOnce(this::initTeleop);
  }

  /**
   * Drive the robot using the raw joystick inputs.
   * This function applies deadband, squaring (default) / cubing (in slow mode) 
   * and slew rate limiting. Joystick inputs are scaled up to speeds in m/s and rad/s.
   * Field relative driving is automatically disabled if the gyro is disconnected. 
   * @param xSpeedSupplier Speed in x direction (-1 to 1)
   * @param ySpeedSupplier Speed in y direction (-1 to 1)
   * @param rSpeedSupplier Rotation speed (-1 to 1)
   */
  public Command driveCommand(
    DoubleSupplier xSpeedSupplier, 
    DoubleSupplier ySpeedSupplier, 
    DoubleSupplier rSpeedSupplier
  ) {
    // Return a command that runs the drive logic
    return run(() -> {
      // Get raw joystick inputs from the suppliers every cycle
      double xSpeed = xSpeedSupplier.getAsDouble();
      double ySpeed = ySpeedSupplier.getAsDouble();
      double rSpeed = rSpeedSupplier.getAsDouble();

      // 1. Apply deadband to the raw joystick inputs.
      // This ignores noise from the joystick when it's in the neutral position.
      xSpeed = MathUtil.applyDeadband(xSpeed, DriveConstants.kJoystickDeadband);
      ySpeed = MathUtil.applyDeadband(ySpeed, DriveConstants.kJoystickDeadband);
      rSpeed = MathUtil.applyDeadband(rSpeed, DriveConstants.kJoystickDeadband);

      // 2. Apply slew rate limiters for a smoother acceleration ramp 
      xSpeed = xSpeedLimiter.calculate(xSpeed);
      ySpeed = ySpeedLimiter.calculate(ySpeed);
      rSpeed = rSpeedLimiter.calculate(rSpeed);

      // 3. Square/cube the inputs (while preserving sign) for finer control at low speeds.
      //    Cubing is used in "slow" mode because it gives even finer control.
      xSpeed = Math.copySign(Math.pow(xSpeed, (this.slowMode ? 3 : 2)), xSpeed);
      ySpeed = Math.copySign(Math.pow(ySpeed, (this.slowMode ? 3 : 2)), ySpeed);
      rSpeed = Math.copySign(Math.pow(rSpeed, (this.slowMode ? 3 : 2)), rSpeed);

      // 4. Convert the joystick's -1..1 to m/s and rad/s
      double xSpeedMPS = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      double ySpeedMPS = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      double rSpeedRad = rSpeed * DriveConstants.kMaxAngularSpeedRadsPerSecond;

      // Force robot-relative if gyro disconnected
      // Not ideal, but better than unexpected robot behavior
      if (this.fieldRelative && !gyro.isConnected()) {
        this.fieldRelative = false;
      }

      // Create robot-relative chassis speeds
      ChassisSpeeds chassisSpeeds;
      if (this.fieldRelative) {
        // Invert translation inputs for red alliance so that pushing the joystick
        // forward always moves the robot away from the driver station
        int invert = Utils.isRedAlliance() ? -1 : 1;

        // Convert field-relative speeds to robot-relative speeds
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeedMPS * invert, 
          ySpeedMPS * invert, 
          rSpeedRad, 
          this.getHeading()
        );
      } else {
        // Use robot-relative speeds directly
        chassisSpeeds = new ChassisSpeeds(xSpeedMPS, ySpeedMPS, rSpeedRad);
      }

      // Drive the robot with robot-relative speeds
      this.driveWithChassisSpeeds(chassisSpeeds);
    });
  }

  /**
   * Creates a command to aim the robot at a specified target pose
   * @param targetPose The Pose2d to aim at
   * @return Command to aim at the target pose
   */  
  public Command aimAtTargetCommand(Pose2d targetPose) {
    // Deferred command to get latest pose when scheduled
    // Set.of(this) ensures driveSubsystem is required
    return Commands.defer(() -> {
      // Check for null target pose
      if (targetPose == null) {
        return Commands.none();
      }

      // Use PathPlanner to drive to current position with rotation toward target
      Pose2d currentPose = getPose();
      double dx = targetPose.getX() - currentPose.getX();
      double dy = targetPose.getY() - currentPose.getY();
      Rotation2d angleToTarget = new Rotation2d(dx, dy);
      
      // Create target pose at current location but rotated toward target
      Pose2d aimPose = new Pose2d(currentPose.getTranslation(), angleToTarget);
      
      // Pathfind to the aim pose (will only rotate since translation is same)
      PathConstraints constraints = new PathConstraints(
        0.5, // Slow translation speed
        DriveConstants.kMaxAccelMetersPerSecondSq,
        DriveConstants.kMaxAngularSpeedRadsPerSecond, 
        DriveConstants.kMaxAngularAccelRadsPerSecondSq
      );
      
      // Return the command to aim at the target
      return AutoBuilder.pathfindToPose(aimPose, constraints, 0.0);
    }, Set.of(this));
  }

  /**
   * Creates a command to drive to a specified pose using PathPlanner
   * @param targetPose The Pose2d to drive to
   * @return Command to drive via PathPlanner to the target pose
   */
  public Command driveToPoseCommand(Pose2d targetPose) {
    // Deferred command to get latest pose when scheduled
    // Set.of(this) ensures driveSubsystem is required
    return Commands.defer(() -> {
      // Check for null target pose
      if (targetPose == null) {
        return Commands.none();
      }

      // Create the constraints to use while pathfinding
      PathConstraints constraints = new PathConstraints(
        DriveConstants.kMaxSpeedMetersPerSecond, 
        DriveConstants.kMaxAccelMetersPerSecondSq,
        DriveConstants.kMaxAngularSpeedRadsPerSecond, 
        DriveConstants.kMaxAngularAccelRadsPerSecondSq
      );

      // Since AutoBuilder is configured, we can use it to build pathfinding commands
      return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
    }, Set.of(this));
  }

  /**
   * Reset the odometry
   */
  public Command resetOdometryCommand() {
    return runOnce(this::resetOdometry);
  }

  /**
   * Reset the odometry to a known pose
   */
  public Command resetOdometryToPoseCommand(Pose2d pose) {
    return runOnce(() -> resetOdometry(pose));
  }

  /**
   * Stop the robot and set the wheels to an X formation
   */
  public Command stopAndLockWheelsCommand() {
    return runOnce(this::stopAndLockWheels);
  }

  /**
   * Enables/disables field-relative driving mode
   */
  public Command toggleFieldRelativeModeCommand() {
    return runOnce(() -> this.fieldRelative = !this.fieldRelative);
  }

  /**
   * Enables/disables slow mode
   */
  public Command toggleSlowModeCommand() {
    return runOnce(() -> this.slowMode = !this.slowMode);
  }

  /**
   * Enable slow mode
   */
  public Command enableSlowModeCommand() {
    return runOnce(() -> this.slowMode = true);
  }

  /**
   * Disable slow mode
   */
  public Command disableSlowModeCommand() {
    return runOnce(() -> this.slowMode = false);
  }

  /**
   * Updates SmartDashboard with subsystem information
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    // Robot telemetry
    builder.addStringProperty("Robot Pose", () -> getPose().toString(), null);
    builder.addDoubleProperty("Robot Heading", () -> Utils.showDouble(getHeading().getDegrees()), null);
    builder.addDoubleProperty("Robot Pitch", () -> Utils.showDouble(gyro.getPitch()), null);
    builder.addDoubleProperty("Robot Roll", () -> Utils.showDouble(gyro.getRoll()), null);
    builder.addDoubleProperty("Chassis vX", () -> Utils.showDouble(getChassisSpeeds().vxMetersPerSecond), null);
    builder.addDoubleProperty("Chassis vY", () -> Utils.showDouble(getChassisSpeeds().vyMetersPerSecond), null);
    builder.addDoubleProperty("Chassis omega", () -> Utils.showDouble(Units.radiansToDegrees(getChassisSpeeds().omegaRadiansPerSecond)), null);
    builder.addBooleanProperty("Field Relative", this::isFieldRelative, null);
    builder.addBooleanProperty("Slow Mode", this::isSlowMode, null);
  }
}
