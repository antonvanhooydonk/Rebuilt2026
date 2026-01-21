// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
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
      return standardDeviations.clone();
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
    // Get the defined camera configurations
    HashMap<String, Transform3d> configs = VisionConstants.kCameraConfigs;
    
    // Add each to camera the list
    if (VisionConstants.kEnableVision) {
      for (String cameraName : configs.keySet()) {
        try {
          cameras.add(new Camera(cameraName, configs.get(cameraName)));
        } catch (Exception e) {
          Utils.logError("Failed to initialize camera " + cameraName + ": " + e.getMessage());
        }
      }
    }
    
    // Warn if no cameras initialized
    if (cameras.isEmpty() && VisionConstants.kEnableVision) {
      Utils.logError("Vision enabled but no cameras initialized!");
    }

    // Initialize dashboard values
    SmartDashboard.putData("Vision", this);

    // Output initialization progress
    Utils.logInfo("Vision subsystem initialized");
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
          
          // Get pose estimate from multi-tag estimator
          Optional<EstimatedRobotPose> poseResult = camera.getPoseEstimator().estimateCoprocMultiTagPose(result);
          
          // Fallback to lowest ambiguity estimate if multi-tag failed
          if (poseResult.isEmpty()) {            
            poseResult = camera.getPoseEstimator().estimateLowestAmbiguityPose(result);
          };

          // Skip further processing if no pose result was returned
          if (poseResult.isEmpty()) continue;
          
          // Get estimated pose
          EstimatedRobotPose estimate = poseResult.get();
          
          // Skip further processing if the pose estimate is not valid
          if (!isValidPose(result, estimate)) continue;
          
          // Calculate standard deviations
          double[] stdDevs = calculateStandardDeviations(result);
          
          // Create measurement
          VisionMeasurement measurement = new VisionMeasurement(
            estimate.estimatedPose.toPose2d(),
            estimate.timestampSeconds,
            stdDevs
          );
          
          // Ensure vision measurement is valid
          if (
            measurement.getPose() == null || 
            measurement.getStandardDeviations() == null ||
            measurement.getStandardDeviations().length != 3
          ) {
            continue;
          }

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
    // Check for single tag estimates
    if (result.getTargets().size() == 1) {
      // Get the target
      PhotonTrackedTarget target = result.getBestTarget();

      // Ambiguity check
      if (target == null || target.getPoseAmbiguity() > VisionConstants.kPoseAmbiguityThreshold) {
        return false;
      }

      // Area check (tags too small = far away = unreliable)
      if (target.getArea() < VisionConstants.kMinTagAreaPixels) {
        return false;
      }
    }
    
    // Get the 2D pose for boundary checks
    Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();
    
    // Check if pose is within field boundaries
    if (!isPoseOnField(pose2d)) {
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
      .mapToDouble(target -> {
        var transform = target.getBestCameraToTarget();
        return transform != null ? transform.getTranslation().getNorm() : VisionConstants.kMaxDistanceMeters;
      })
      .average()
      .orElse(VisionConstants.kMaxDistanceMeters);
        
    // Clamp distance to reasonable range
    avgDistance = Math.min(avgDistance, VisionConstants.kMaxDistanceMeters);

    /** 
      * Recommended Tuning Process:
      * 1. Start conservative (divide by 10-15)
      * 2. Test pose estimation accuracy in known locations
      * 3. Adjust based on results:      
      *    - If vision corrections are too weak => increase divisor (30 => 50)
      *    - If vision corrections fight odometry => decrease divisor (30 => 15)
      * 4. Validate with field testing
      *
      * Physical Meaning:
      *   The "/ 30" essentially means: "For every meter of distance, square it 
      *   and divide by 30 to get the uncertainty multiplier."
      *   XY uncertainty scaling:
      *     At 3 meters: (3²/20) = 0.45, so 1.45x multiplier
      *     At 5 meters: (5²/20) = 1.25, so 2.25x multiplier
      *   Theta uncertainty scaling (less aggressive):
      *     At 3 meters: (3²/40) = 0.225, so 1.225x multiplier
      *     At 5 meters: (5²/40) = 0.625, so 1.625x multiplier
      */

    // Base standard deviations (empirically determined)
    double baseXYStdDev, baseThetaStdDev;
    
    // Set base standard deviation based on number of targets
    if (numTargets == 1) {
      // Single tag - less reliable
      baseXYStdDev = VisionConstants.kSingleTagBaseXYstdDev;
      baseThetaStdDev = VisionConstants.kSingleTagBaseThetaStdDev;
    } else {
      // Multi tag - more reliable  
      baseXYStdDev = VisionConstants.kMultiTagBaseXYstdDev;
      baseThetaStdDev = VisionConstants.kMultiTagBaseThetaStdDev;
    }
    
    // Distance-based uncertainty scaling factors
    // These values are tuned based on testing with your specific setup
    double xyDistanceScale = 1.0 + (avgDistance * avgDistance / 20.0);    // More aggressive
    double thetaDistanceScale = 1.0 + (avgDistance * avgDistance / 40.0); // Less aggressive for rotation
    
    // Calculate final standard deviations
    double xyStdDev = baseXYStdDev * xyDistanceScale;
    double thetaStdDev = baseThetaStdDev * thetaDistanceScale;
    
    // Return as array [x, y, theta]
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
      List<Integer> ids = result.getTargets().stream().map(PhotonTrackedTarget::getFiducialId).toList();
      Utils.logInfo(cameraName + " detected " + result.getTargets().size() + " targets: " + ids);
      lastTargetLogTimestamp = currentTime;
    }
  }

  /**
   * Check if a given pose is within the field boundaries
   * @param pose The pose to check
   * @return True if the pose is on the field
   */
  public boolean isPoseOnField(Pose2d pose) {
    if (
      pose.getX() < -VisionConstants.kFieldBorderMargin || 
      pose.getX() > FieldConstants.kFieldLengthMeters + VisionConstants.kFieldBorderMargin ||
      pose.getY() < -VisionConstants.kFieldBorderMargin || 
      pose.getY() > FieldConstants.kFieldWidthMeters + VisionConstants.kFieldBorderMargin
    ) {
      return false;
    }
    return true;
  }

  /**
   * Get a vision system camera by name.
   * @param cameraName Name of the camera
   * @return Optional containing the camera
   */
  public Optional<Camera> getCamera(String cameraName) {
    return cameras.stream()
      .filter(camera -> camera.getName().equals(cameraName))
      .findFirst();
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
    
    // Individual camera status - call getLatestResult() in the lambda
    for (Camera camera : cameras) {
      String prefix = "Camera/" + camera.getName() + "/";
    
      builder.addBooleanProperty(prefix + "Connected", () -> camera.getCamera().isConnected(), null);
      builder.addBooleanProperty(prefix + "Has Targets", () -> camera.getLatestResult().hasTargets(), null);
      builder.addDoubleProperty(prefix + "Target Count", () -> camera.getLatestResult().getTargets().size(), null);
      builder.addDoubleProperty(prefix + "Unread Results", () -> camera.getCachedResults().size(), null);
      
      // For best target data, check if targets exist each time
      builder.addDoubleProperty(prefix + "Best Target ID", () -> {
        PhotonPipelineResult r = camera.getLatestResult();
        return r.hasTargets() ? r.getBestTarget().getFiducialId() : -1;
      }, null);
      builder.addDoubleProperty(prefix + "Best Target Distance", () -> {
        PhotonPipelineResult r = camera.getLatestResult();
        return r.hasTargets() ? r.getBestTarget().getBestCameraToTarget().getTranslation().getNorm() : 0;
      }, null);
      builder.addDoubleProperty(prefix + "Best Target Ambiguity", () -> {
        PhotonPipelineResult r = camera.getLatestResult();
        return r.hasTargets() ? r.getBestTarget().getPoseAmbiguity() : 0;
      }, null);
    }
    
    // Latest measurements
    builder.addStringProperty("Latest Pose", () -> {
      if (latestMeasurements.isEmpty()) return "None";
      return latestMeasurements.get(latestMeasurements.size() - 1).getPose().toString();
    }, null);
    builder.addDoubleProperty("Latest Timestamp", () -> {
      if (latestMeasurements.isEmpty()) return 0;
      return latestMeasurements.get(latestMeasurements.size() - 1).getTimestampSeconds();
    }, null);
  }
}
