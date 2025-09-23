// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(DriveConstants.kSlewRateLimit);
  private final SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(DriveConstants.kSlewRateLimit);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kSlewRateLimit);
  
  // Swerve modules - ensure indexed order is 0 = FL, 1 = FR, 2 = BL, 3 = BR
  private final SwerveModule[] modules = new SwerveModule[4];
  
  // Gyroscope
  private final AHRS gyro;
  
  // Kinematics and odometry
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;

  // Setpoint generator
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint currentSetpoint;
  
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

    // Initialize gyroscope
    gyro = new AHRS(NavXComType.kMXP_SPI);

    // If we need a gyro offset (mounted wrong etc..), set it here
    // gyro.setAngleAdjustment(90);
    
    // Initialize front left swerve module
    modules[0] = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorID,                  // Drive motor CAN ID
      DriveConstants.kFrontLeftSteerMotorID,                  // Steer motor CAN ID  
      DriveConstants.kFrontLeftDriveMotorInverted,            // Drive motor inverted
      DriveConstants.kFrontLeftSteerMotorInverted,            // Steer motor inverted
      DriveConstants.kFrontLeftAbsoluteEncoderID,             // Absolute encoder analog port
      DriveConstants.kFrontLeftAbsoluteEncoderOffsetRadians,  // Absolute encoder offset in radians (calibrate this)
      DriveConstants.kFrontLeftAbsoluteEncoderInverted,       // Absolute encoder inverted
      "FL"
    );
    
    // Initialize front right swerve module
    modules[1] = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorID,                 // Drive motor CAN ID
      DriveConstants.kFrontRightSteerMotorID,                 // Steer motor CAN ID  
      DriveConstants.kFrontRightDriveMotorInverted,           // Drive motor inverted
      DriveConstants.kFrontRightSteerMotorInverted,           // Steer motor inverted
      DriveConstants.kFrontRightAbsoluteEncoderID,            // Absolute encoder analog port
      DriveConstants.kFrontRightAbsoluteEncoderOffsetRadians, // Absolute encoder offset in radians (calibrate this)
      DriveConstants.kFrontRightAbsoluteEncoderInverted,      // Absolute encoder inverted
      "FR"
    );
    
    // Initialize back left swerve module
    modules[2] = new SwerveModule(
      DriveConstants.kBackLeftDriveMotorID,                   // Drive motor CAN ID
      DriveConstants.kBackLeftSteerMotorID,                   // Steer motor CAN ID  
      DriveConstants.kBackLeftDriveMotorInverted,             // Drive motor inverted
      DriveConstants.kBackLeftSteerMotorInverted,             // Steer motor inverted
      DriveConstants.kBackLeftAbsoluteEncoderID,              // Absolute encoder analog port
      DriveConstants.kBackLeftAbsoluteEncoderOffsetRadians,   // Absolute encoder offset in radians (calibrate this)
      DriveConstants.kBackLeftAbsoluteEncoderInverted,        // Absolute encoder inverted
      "BL"
    );
    
    // Initialize back right swerve module    
    modules[3] = new SwerveModule(
      DriveConstants.kBackRightDriveMotorID,                  // Drive motor CAN ID
      DriveConstants.kBackRightSteerMotorID,                  // Steer motor CAN ID  
      DriveConstants.kBackRightDriveMotorInverted,            // Drive motor inverted
      DriveConstants.kBackRightSteerMotorInverted,            // Steer motor inverted
      DriveConstants.kBackRightAbsoluteEncoderID,             // Absolute encoder analog port
      DriveConstants.kBackRightAbsoluteEncoderOffsetRadians,  // Absolute encoder offset in radians (calibrate this)
      DriveConstants.kBackRightAbsoluteEncoderInverted,       // Absolute encoder inverted
      "BR"
    );
    
    // Initialize swerve drive kinematics
    kinematics = new SwerveDriveKinematics(
      DriveConstants.kFrontLeftLocation, 
      DriveConstants.kFrontRightLocation, 
      DriveConstants.kBackLeftLocation, 
      DriveConstants.kBackRightLocation
    );
    
    // Initialize pose estimator for vision integration
    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics,           // The kinematics object
      getGyroAngle(),       // The current gyro angle
      getModulePositions(), // The current module positions
      new Pose2d()          // Will be updated by auto and vision
    );

    // Initialize the robot config.
    // This could be populated from RobotConfig.fromGUISettings() 
    // but it is better to initialize this from constants.
    RobotConfig robotConfig = new RobotConfig(
      DriveConstants.kRobotMassKg,
      DriveConstants.kRobotMOI,
      new ModuleConfig(
        DriveConstants.kWheelRadiusMeters, 
        DriveConstants.kMaxSpeedMetersPerSecond, 
        DriveConstants.kWheelCOF, 
        DCMotor.getKrakenX60(1).withReduction(DriveConstants.kDriveGearRatio), 
        DriveConstants.kDriveMotorMaxCurrent,
        1
      ),
      DriveConstants.kFrontLeftLocation, 
      DriveConstants.kFrontRightLocation, 
      DriveConstants.kBackLeftLocation, 
      DriveConstants.kBackRightLocation
    );

    // Initialize the swerve setpoint generator
    setpointGenerator = new SwerveSetpointGenerator(
      robotConfig, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
      DriveConstants.kMaxAngularSpeedRadiansPerSecond // The max rotation velocity of a swerve module in radians per second.
    );
    
    // Initialize the current setpoint to the robot's speeds & module states
    currentSetpoint = new SwerveSetpoint(
      getChassisSpeeds(), 
      getModuleStates(), 
      DriveFeedforwards.zeros(robotConfig.numModules)
    );

    // Configure AutoBuilder for path following
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier
      this::driveWithChassisSpeeds, // Method that will drive the robot given ChassisSpeeds
      new PPHolonomicDriveController(
        new PIDConstants(DriveConstants.kDriveKP, DriveConstants.kDriveKI, DriveConstants.kDriveKD), // Translation PID constants
        new PIDConstants(DriveConstants.kSteerKP, DriveConstants.kSteerKI, DriveConstants.kSteerKD)  // Rotation PID constants
      ),
      robotConfig, // Robot configuration
      Utils::isRedAlliance, // Method to flip path based on alliance color
      this // Reference to this subsystem to set requirements
    );

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD THEM HERE
    PathfindingCommand.warmupCommand().schedule();

    // Add field to dashboard
    SmartDashboard.putData("Field", field2d);
  }
  
  @Override
  public void periodic() {
    // Update odometry
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroAngle(), getModulePositions());
    
    // Add vision measurements to the pose estimator
    addVisionMeasurements();
    
    // Update field visualization
    field2d.setRobotPose(getPose());
    
    // Update dashboard
    updateDashboard();

    // Call each swerve module's periodic
    for (var module : modules) {
      module.periodic();
    }
  }

  /**
   * Updates pose estimation using vision measurements from VisionSubsystem
   */
  private void addVisionMeasurements() {
    // Exit early if vision subsystem is not available or is disabled
    if (visionSubsystem == null || !visionSubsystem.isEnabled()) return;
    
    // Get all latest vision measurements
    List<VisionMeasurement> measurements = visionSubsystem.getLatestMeasurements();

    // Process each vision measurement
    for (VisionMeasurement measurement : measurements) {
      try {
        // Add vision measurement to pose estimator
        poseEstimator.addVisionMeasurement(
          measurement.pose,
          measurement.timestampSeconds,
          VecBuilder.fill(
            measurement.standardDeviations[0], 
            measurement.standardDeviations[1], 
            measurement.standardDeviations[2]
          )
        );          
      } catch (Exception e) {
        System.err.println("Error adding vision measurement: " + e.getMessage());
      }
    }
  }

  /**
   * Creates a command to aim the robot at a specified target pose
   * @param targetPose The Pose2d to aim at
   * @return Command to aim at the target pose
   */
  public Command aimAtTargetCommand(Pose2d targetPose) {
    return run(() -> driveWithChassisSpeeds(new ChassisSpeeds(
      0.0,  
      0.0,
      -targetPose.getRotation().getRadians()
    )));
  }

  /**
   * Creates a command to drive to a specified pose using PathPlanner
   * @param targetPose The Pose2d to drive to
   * @return Command to drive via PathPlanner to the target pose
   */
  public Command driveToPoseCommand(Pose2d targetPose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
      DriveConstants.kMaxSpeedMetersPerSecond, 
      DriveConstants.kMaxAccelMetersPerSecondSq,
      DriveConstants.kMaxAngularSpeedRadiansPerSecond, 
      DriveConstants.kMaxAngularAccelRadiansPerSecondSq
    );

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
  }

  /**
   * Drive the robot using joystick inputs
   * @param xSpeed Speed in x direction (-1 to 1)
   * @param ySpeed Speed in y direction (-1 to 1)
   * @param rot Rotation speed (-1 to 1)
   * @param fieldRelative True if the inputs are field relative, false if robot-relative
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Apply deadband to the raw joystick inputs.
    // This ignores noise from the joystick when it's in the neutral position.
    xSpeed = Math.abs(xSpeed) > DriveConstants.kJoystickDeadband ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > DriveConstants.kJoystickDeadband ? ySpeed : 0;
    rot = Math.abs(rot) > DriveConstants.kJoystickDeadband ? rot : 0;

    // Then cube or square the inputs (while preserving sign) for finer control at low speeds.
    // Cubing gives finer control than squaring, but squaring is still better than linear.
    xSpeed = Math.copySign(Math.pow(xSpeed, (slowMode ? 3 : 2)), xSpeed);
    ySpeed = Math.copySign(Math.pow(ySpeed, (slowMode ? 3 : 2)), ySpeed);
    rot = Math.copySign(Math.pow(rot, (slowMode ? 3 : 2)), rot);

    // Then apply slew rate limiters for a smoother acceleration ramp 
    xSpeed = xSpeedLimiter.calculate(xSpeed);
    ySpeed = ySpeedLimiter.calculate(ySpeed);
    rot = rotLimiter.calculate(rot);

    // Convert joystick's -1..1 to m/s and rad/s chassis speeds
    double xSpeedMS = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedMS = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotSpeedRad = rot * DriveConstants.kMaxAngularSpeedRadiansPerSecond;

    // Invert the translation speeds if we are on the red alliance
    // NOTE: may need to do this?, maybe only for fieldRelative?, left here for reference
    // if (Utils.isRedAlliance()) {
    //   xSpeedMS = -xSpeedMS;
    //   ySpeedMS = -ySpeedMS;
    // }

    // Create/convert inputs to robot-relative chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (fieldRelative) {
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeedMS, ySpeedMS, rotSpeedRad, getHeading()
      );
    } else {
      chassisSpeeds = new ChassisSpeeds(xSpeedMS, ySpeedMS, rotSpeedRad);
    }

    // Drive the robot with robot-relative speeds
    driveWithChassisSpeeds(chassisSpeeds);
  }

  /**
   * This method will take in desired/target robot-relative chassis speeds and
   * generate a swerve setpoint, then set the desired state for each module.
   * @param speeds The desired robot-relative speeds
   */
  public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
    // Note: It is important to not discretize speeds, that is,
    // call SwerveDriveKinematics.desaturateWheelSpeeds(), 
    // before or after using the setpoint generator, as 
    // it will discretize them for you.
    currentSetpoint = setpointGenerator.generateSetpoint(
      currentSetpoint,                      // The previous "current" setpoint
      speeds,                               // The desired target speeds
      DriveConstants.kPeriodicTimeSeconds   // The loop time of the robot code, in seconds
    );

    // Set each module state
    SwerveModuleState[] desiredStates = currentSetpoint.moduleStates();
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(desiredStates[i]);
    }
  }

  /**
   * Sets the desired state for each swerve module. Do not call this from driveWithChassisSpeeds.
   * @param desiredStates Array of desired SwerveModuleStates
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {    
    // Normalize wheel speeds 
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    
    // Set each module state
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(desiredStates[i]);
    }
  }

  /**
   * Set each swerver module to brake/coast mode
   * @param brake True to enable motor brake, false for coast
   */
  public void setMotorBrake(boolean brake) {
    for (var module : modules) {
      module.setMotorBrake(brake);
    }
  }

  /**
   * Stop the robot
   */
  public void stop() {
    for (var module : modules) {
      module.stop();
    }
  }

  /**
   * Sets modules to X formation for defense
   */
  public void setXFormation() {
    SwerveModuleState[] states = {
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)), // Front Left
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),        // Front Right
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),        // Back Left
      new SwerveModuleState(0, Rotation2d.fromDegrees(45))  // Back Right
    };
    setModuleStates(states);
  }

  /**
   * Resets all the swerve module encoders and resets the pose estimator.
   * NOTE: Should never need to call this if vision is working properly.
   */
  public void resetEncoders() {
    // reset each swerve module's encoders
    for (var module : modules) {
      module.resetEncoders();
    }

    // reset pose estimator
    resetPose(poseEstimator.getEstimatedPosition());
  }

  /**
   * Zeros the gyroscope heading and resets the pose estimator.
   * NOTE: Should never need to call this if vision is working properly.
   */
  public void zeroHeading() {
    // Reset gyro to zero
    gyro.reset();

    // reset pose estimator
    resetPose(poseEstimator.getEstimatedPosition());
  }

  /**
   * Gets the current raw gyro angle. This may not match the robot's heading
   * due to initial offset, or drift over time. Generally, this is only 
   * used as input into the swerve drive PoseEstimator, and then our 
   * robot can be driven using the PoseEstimator's heading.
   * @return The current gyro angle as a Rotation2d, CCW positive
   */
  public Rotation2d getGyroAngle() {
    // Negate the angle b/c our NavX is CW positive
    return Rotation2d.fromDegrees(-gyro.getAngle()); 
  }

  /**
   * Gets the current heading of the robot from the pose estimator.
   * This is what should be fed into the drive (auto and teleop)
   * functions when calculating chassis speeds & module states.
   * @return Current heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    return poseEstimator.getEstimatedPosition().getRotation(); 
  }

  /**
   * Gets the current pitch of the robot (for auto-balancing)
   * @return Current pitch in degrees
   */
  public double getPitch() {
    return gyro.getPitch();
  }

  /**
   * Gets the current roll of the robot
   * @return Current roll in degrees
   */
  public double getRoll() {
    return gyro.getRoll();
  }

  /**
   * Gets the current pose of the robot
   * @return Current pose 
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the pose to a given pose
   * @param pose New pose
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getGyroAngle(), getModulePositions(), pose);
  }

  /**
   * Gets the current chassis speeds
   * @return Current ChassisSpeeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Gets the current module states
   * @return Array of current SwerveModuleStates
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      modules[0].getState(),
      modules[1].getState(),
      modules[2].getState(),
      modules[3].getState()
    };
  }

  /**
   * Gets the current module positions
   * @return Array of current SwerveModulePositions
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      modules[0].getPosition(),
      modules[1].getPosition(),
      modules[2].getPosition(),
      modules[3].getPosition()
    };
  }

  /**
   * Enables/disables field-relative driving
   * @param enabled Whether field-relative is enabled
   */
  public void setFieldRelative(boolean enabled) {
    this.fieldRelative = enabled;
  }

  /**
   * Gets whether field-relative driving is enabled
   * @return Field-relative status
   */
  public boolean isFieldRelative() {
    return fieldRelative;
  }

  /**
   * Enables/disables slow mode
   * @param enabled Whether slow mode is enabled
   */
  public void setSlowMode(boolean enabled) {
    this.slowMode = enabled;
  }

  /**
   * Gets whether slow mode is enabled
   * @return Slow mode status
   */
  public boolean isSlowMode() {
    return slowMode;
  }

  /**
   * Gets the SwerveDriveKinematics object
   * @return Kinematics object
   */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Updates SmartDashboard with subsystem information
   */
  private void updateDashboard() {
    // Robot telemetry
    SmartDashboard.putString("Robot Pose", getPose().toString());
    SmartDashboard.putNumber("Robot Heading", getHeading().getDegrees());
    SmartDashboard.putNumber("Robot Pitch", getPitch());
    SmartDashboard.putNumber("Robot Roll", getRoll());
    SmartDashboard.putBoolean("Field Relative", fieldRelative);
    SmartDashboard.putBoolean("Slow Mode", slowMode);
    SmartDashboard.putBoolean("Gyro Connected", gyro.isConnected());
    
    // Chassis speeds
    ChassisSpeeds speeds = getChassisSpeeds();
    SmartDashboard.putNumber("Chassis vX", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Chassis vY", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Chassis omega", Units.radiansToDegrees(speeds.omegaRadiansPerSecond));
  }
}
