// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FieldConstants;
import frc.robot.util.Utils;

/**
 * Vision subsystem for AprilTag-based pose estimation using PhotonVision.
 * Uses getAllUnreadResults() called only once per periodic cycle and
 * caches the results for later use by the subsystem.
 */
public class VisionSubsystem extends SubsystemBase {
  // Camera configuration class
  public static class CameraConfig {
    private final String name;
    private final Transform3d robotToCamera;
    
    public CameraConfig(String name, Transform3d robotToCamera) {
      this.name = name;
      this.robotToCamera = robotToCamera;
    }

    /**
     * Gets the camera name
     * @return
     */
    public String getName() {
      return name;
    }

    /**
     * Gets the robot-to-camera transform
     * @return
     */
    public Transform3d getRobotToCamera() {
      return robotToCamera;
    }
  }

  // Vision measurement data class
  public static class VisionMeasurement {
    private final Pose2d pose;
    private final double timestampSeconds;
    private final double[] standardDeviations;

    public VisionMeasurement(Pose2d pose, double timestampSeconds, double[] standardDeviations) {
      this.pose = pose;
      this.timestampSeconds = timestampSeconds;
      this.standardDeviations = standardDeviations;
    }

    /**
     * Gets the estimated pose
     * @return
     */
    public Pose2d getPose() {
      return pose;
    }

    /**
     * Gets the timestamp of the measurement
     * @return
     */
    public double getTimestampSeconds() {
      return timestampSeconds;
    }

    /**
     * Gets the standard deviations for x, y, and theta
     * @return
     */
    public double[] getStandardDeviations() {
      return standardDeviations;
    }
  }

  // Camera data class with cached results
  private static class Camera {
    private final CameraConfig config;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    
    // Cached data from current periodic cycle
    private List<PhotonPipelineResult> cachedResults = new ArrayList<>();
    
    /**
     * Construct a new Camera instance
     * @param config The camera configuration
     */
    public Camera(CameraConfig config) {
      // Initialize PhotonCamera
      PhotonCamera camera = new PhotonCamera(config.name);

      // Initialize PhotonPoseEstimator
      PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
        FieldConstants.fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        config.robotToCamera
      );

      // Set fallback strategy for multi-tag ambiguity
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      // Assign to fields
      this.config = config;
      this.camera = camera;
      this.poseEstimator = poseEstimator;
    }
        
    /**
     * Updates cached results - CALL ONLY ONCE PER PERIODIC CYCLE
     */
    public void updateCache() {
      cachedResults = camera.getAllUnreadResults();
    }
        
    /**
     * Gets the most recent result from cache
     */
    public PhotonPipelineResult getLatestResult() {
      if (!cachedResults.isEmpty()) {
        return cachedResults.get(cachedResults.size() - 1);
      }
      return new PhotonPipelineResult();
    }
    
    /**
     * Checks if any cached results have targets
     */
    public boolean hasTargets() {
      if (!cachedResults.isEmpty()) {
        return cachedResults.stream().anyMatch(PhotonPipelineResult::hasTargets);
      }
      return false;
    }
        
    /**
     * Gets target count from most recent cached result
     */
    public int getTargetCount() {
      PhotonPipelineResult mostRecent = getLatestResult();
      return mostRecent.getTargets().size();
    }
        
    /**
     * Gets best target from most recent cached result
     */
    public Optional<PhotonTrackedTarget> getBestTarget() {
      PhotonPipelineResult mostRecent = getLatestResult();
      return mostRecent.hasTargets() ? Optional.of(mostRecent.getBestTarget()) : Optional.empty();
    }

    /**
     * Gets the camera name
     * @return The camera's name
     */
    public String getName() {
      return config.name;
    }

    /**
     * Gets the PhotonCamera instance
     * @return
     */
    public PhotonCamera getCamera() {
      return camera;
    }

    /**
     * Gets the PhotonPoseEstimator instance
     * @return
     */
    public PhotonPoseEstimator getPoseEstimator() {
      return poseEstimator;
    }

    /**
     * Gets the cached results
     * @return
     */
    public List<PhotonPipelineResult> getCachedResults() {
      return cachedResults;
    }
  }

  // Camera data storage
  private final List<Camera> cameras = new ArrayList<>();

  // Latest vision measurements (updated each periodic cycle)
  private final List<VisionMeasurement> latestMeasurements = new ArrayList<>();

  // Logging
  private double lastTargetLogTimestamp = 0;

  /**
   * Creates a new VisionSubsystem
   */
  public VisionSubsystem() {
    // Process each camera config
    for (CameraConfig config : VisionConstants.kCameraConfigs) {
      try {
        // Add each to camera the list
        cameras.add(new Camera(config));

        Utils.logInfo("Initialized camera: " + config.name);
      } catch (Exception e) {
        Utils.logError("Failed to initialize camera " + config.name + ": " + e.getMessage());
      }
    }
    
    // Warn if no cameras initialized
    if (cameras.isEmpty()) {
      Utils.logError("No cameras initialized!");
    }

    // Initialize dashboard values
    SmartDashboard.putData("Vision", this);

    // Output initialization progress
    Utils.logInfo("Vision subsystem intialized");
  }

  @Override
  public void periodic() {
    // CRITICAL: Update camera caches FIRST
    updateCameraCache();
    
    // Process vision measurements using cached data
    updateVisionMeasurements();
  }

  /**
   * Update each camera cache
   * The camera's updateCache() function should be the ONLY
   * place where getAllUnreadResults() gets called!
   */
  private void updateCameraCache() {
    for (Camera camera : cameras) {
      try {
        camera.updateCache();
      } catch (Exception e) {
        Utils.logError("Error updating cache for " + camera.getName() + ": " + e.getMessage());
      }
    }
  }

  /**
   * Update vision measurements from cached camera data
   */
  private void updateVisionMeasurements() {
    // Clear previous measurements
    latestMeasurements.clear();
    
    // Process each camera's cached unread results
    for (Camera camera : cameras) {
      try {
        // Process ALL cached unread results
        for (PhotonPipelineResult result : camera.getCachedResults()) {
          // Skip further processing if result has no targets
          if (!result.hasTargets()) continue;
          
          // Get pose estimate
          Optional<EstimatedRobotPose> poseResult = camera.getPoseEstimator().update(result);
          
          // Skip further processing if no pose result was returned
          if (poseResult.isEmpty()) continue;
          
          // Get estimated pose
          EstimatedRobotPose estimatedPose = poseResult.get();
          
          // Skip further processing if the pose estimate is not valid
          if (!isValidPose(result, estimatedPose)) continue;
          
          // Calculate standard deviations
          double[] stdDevs = calculateStandardDeviations(result);
          
          // Create measurement
          VisionMeasurement measurement = new VisionMeasurement(
            estimatedPose.estimatedPose.toPose2d(),
            estimatedPose.timestampSeconds,
            stdDevs
          );
          
          // Add to latest measurements
          // These measurements will be read by the drive subsystem and
          // added to the pose estimator in the next cycle.
          latestMeasurements.add(measurement);
          
          // Log targets periodically (only for the most recent result)
          if (result == camera.getCachedResults().get(camera.getCachedResults().size() - 1)) {
            logTargets(result, camera.getName());
          }
        }          
      } catch (Exception e) {
        Utils.logError("Error processing vision for " + camera.getName() + ": " + e.getMessage());
      }
    }
    
    // Sort measurements by timestamp to ensure chronological processing
    latestMeasurements.sort((a, b) -> Double.compare(a.getTimestampSeconds(), b.getTimestampSeconds()));
  }

  /**
   * Validates a pose estimate to reject obviously incorrect measurements
   * @param result The photon camera pipeline result validate
   * @param estimatedPose The estimated robot pose from the camera
   * @return True if the pose is valid
   */
  private boolean isValidPose(PhotonPipelineResult result, EstimatedRobotPose estimatedPose) {
    // Check pose ambiguity for single tag estimates
    if (result.getTargets().size() == 1) {
      PhotonTrackedTarget target = result.getBestTarget();
      if (target.getPoseAmbiguity() > VisionConstants.kPoseAmbiguityThreshold) {
        return false;
      }
    }
    
    // Get the 2D pose for boundary checks
    Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();
    
    // Check if pose is within field boundaries
    if (
      pose2d.getX() < -VisionConstants.kFieldBorderMargin || 
      pose2d.getX() > FieldConstants.fieldLengthMeters + VisionConstants.kFieldBorderMargin ||
      pose2d.getY() < -VisionConstants.kFieldBorderMargin || 
      pose2d.getY() > FieldConstants.fieldWidthMeters + VisionConstants.kFieldBorderMargin
    ) {
      return false;
    }
    
    // Check Z coordinate (robot should be on the ground)
    double z = estimatedPose.estimatedPose.getZ();
    if (Math.abs(z) > VisionConstants.kZMargin) {
      return false;
    }
    
    // If we reach here, the pose is considered valid
    return true;
  }

  /**
   * Calculate standard deviations based on target quality
   * @param result The photon camera pipeline result to calculate standard deviations for
   * @return Array of standard deviations for x, y, and theta
   */
  private double[] calculateStandardDeviations(PhotonPipelineResult result) {
    // Number of targets used in the estimate
    int numTargets = result.getTargets().size();
    
    // Calculate average distance to targets
    double avgDistance = result.getTargets().stream()
      .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
      .average()
      .orElse(VisionConstants.kMaxDistanceMeters);
        
    // Clamp distance to reasonable range
    avgDistance = Math.min(avgDistance, VisionConstants.kMaxDistanceMeters);
    
    // Declare standard deviations
    double xyStdDev, thetaStdDev;
    
    // Adjust standard deviations based on number of targets and distance
    if (numTargets == 1) {
      // Single tag - less reliable
      xyStdDev = VisionConstants.kSingleTagStdDevFactor * (1 + (avgDistance * avgDistance / 30));
      thetaStdDev = Units.degreesToRadians(12) * (1 + (avgDistance * avgDistance / 30));
    } else {
      // Multi tag - more reliable
      xyStdDev = VisionConstants.kMultiTagStdDevFactor * (1 + (avgDistance * avgDistance / 30));
      thetaStdDev = Units.degreesToRadians(6) * (1 + (avgDistance * avgDistance / 30));
    }
    
    // Return standard deviations for x, y, and theta
    return new double[] {xyStdDev, xyStdDev, thetaStdDev};
  }

  /**
   * Log target information periodically
   * @param result The photon camera pipeline result to log
   * @param cameraName The name of the camera
   */
  private void logTargets(PhotonPipelineResult result, String cameraName) {
    double currentTime = Timer.getFPGATimestamp();    
    if (currentTime - lastTargetLogTimestamp > VisionConstants.kTargetLogTimeSeconds) {
      Utils.logInfo(cameraName + " detected " + result.getTargets().size() + " targets:");      
      for (PhotonTrackedTarget target : result.getTargets()) {
        Utils.logInfo("ID=" + target.getFiducialId());
      }      
      lastTargetLogTimestamp = currentTime;
    }
  }

  /**
   * Gets the latest vision measurements from all cameras
   * @return List of vision measurements (from current periodic cycle)
   */
  public List<VisionMeasurement> getLatestMeasurements() {
    return new ArrayList<>(latestMeasurements);
  }

  /**
   * Gets the best target from the specified camera (from cached data)
   * @param cameraName Name of the camera
   * @return Optional containing the best target
   */
  public Optional<PhotonTrackedTarget> getBestTarget(String cameraName) {
    return cameras.stream()
      .filter(camera -> camera.getName().equals(cameraName))
      .findFirst()
      .flatMap(Camera::getBestTarget);
  }

  /**
   * Gets the best target from any camera (from cached data)
   * @return Optional containing the best target
   */
  public Optional<PhotonTrackedTarget> getBestTarget() {
    return cameras.stream()
      .map(Camera::getBestTarget)
      .filter(Optional::isPresent)
      .map(Optional::get)
      .findFirst();
  }

  /**
   * Checks if any camera has visible targets (from cached data)
   * @return True if any targets are visible
   */
  public boolean hasTargets() {
    return cameras.stream().anyMatch(Camera::hasTargets);
  }

  /**
   * Gets the number of visible targets across all cameras (from cached data)
   * @return Total number of visible targets
   */
  public int getTotalTargetCount() {
    return cameras.stream().mapToInt(Camera::getTargetCount).sum();
  }

  /**
   * Gets a list of all camera names
   * @return List of camera names
   */
  public List<String> getCameraNames() {
    return cameras.stream().map(camera -> camera.getName()).toList();
  }

  /**
   * Checks if vision subsystem is enabled (has at least one working camera)
   * @return True if vision is enabled
   */
  public boolean isEnabled() {
    return !cameras.isEmpty();
  }

  /**
   * Force an immediate vision measurement update (useful for testing)
   * This will update caches and process new measurements
   */
  public void forceUpdate() {
    updateCameraCache();
    updateVisionMeasurements();
  }

  /**
   * Gets the current buffer size for a specific camera (from cached data)
   * @param cameraName Name of the camera
   * @return Number of unread results in buffer (from last cache update)
   */
  public int getUnreadResultCount(String cameraName) {
    return cameras.stream()
      .filter(camera -> camera.getName().equals(cameraName))
      .findFirst()
      .map(camera -> camera.getCachedResults().size())
      .orElse(0);
  }

  /**
   * Gets the most recent result for a specific camera (from cached data)
   * @param cameraName Name of the camera
   * @return Most recent PhotonPipelineResult, or empty result if camera not found
   */
  public PhotonPipelineResult getLatestResult(String cameraName) {
    return cameras.stream()
      .filter(camera -> camera.getName().equals(cameraName))
      .findFirst()
      .map(Camera::getLatestResult)
      .orElse(new PhotonPipelineResult());
  }

  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Enabled", this::isEnabled, null);
    builder.addBooleanProperty("Has Targets", this::hasTargets, null);
    builder.addDoubleProperty("Total Targets", this::getTotalTargetCount, null);
    builder.addIntegerProperty("Measurements", () -> latestMeasurements.size(), null);
    
    // Individual camera status (using cached data)
    for (Camera camera : cameras) {
      String prefix = "Camera/" + camera.getName() + "/";
      PhotonPipelineResult result = camera.getLatestResult();
      
      builder.addBooleanProperty(prefix + "Connected", () -> camera.getCamera().isConnected(), null);
      builder.addBooleanProperty(prefix + "Has Targets", () -> result.hasTargets(), null);
      builder.addDoubleProperty(prefix + "Target Count", () -> result.getTargets().size(), null);
      builder.addDoubleProperty(prefix + "Unread Results", () -> camera.cachedResults.size(), null);
      
      if (result.hasTargets()) {
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        builder.addDoubleProperty(prefix + "Best Target ID", () -> bestTarget.getFiducialId(), null);
        builder.addDoubleProperty(prefix + "Best Target Distance", () -> bestTarget.getBestCameraToTarget().getTranslation().getNorm(), null);
        builder.addDoubleProperty(prefix + "Best Target Ambiguity", () -> bestTarget.getPoseAmbiguity(), null);
      }
    }
    
    // Latest measurements
    if (!latestMeasurements.isEmpty()) {
      VisionMeasurement latestMeasurement = latestMeasurements.get(latestMeasurements.size() - 1);
      builder.addStringProperty("Latest Pose", () -> latestMeasurement.pose.toString(), null);
      builder.addDoubleProperty("Latest Timestamp", () -> latestMeasurement.timestampSeconds, null);
    }
  }
}
