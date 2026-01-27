// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class VisionConstants {
    // Camera configurations - modify for your robot
  public static final HashMap<String, Transform3d> kCameraConfigs = defineCameras();

  /**
   * Initialize the robot cameras
   * @return A hashmap containing camera names and their Transform3ds
   */
  private static HashMap<String, Transform3d> defineCameras() {
    HashMap<String, Transform3d> cameras = new HashMap<>();

    // define the robot's cameras here
    cameras.put("VISION_FRONT", new Transform3d(
      new Pose3d(
        Units.inchesToMeters(12), // Forward 12 inches
        Units.inchesToMeters(0),  // Centered left/right  
        Units.inchesToMeters(24), // 24 inches high
        Rotation3d.kZero
      ).getTranslation(), 
      new Rotation3d(
        0,     // Roll (tilt left/right)
        0,    // Pitch (tilt up/down) - negative = tilted down
        0       // Yaw (rotate left/right)
      )
    ));

    // return the camera map
    return cameras;
  }

  // Vision configuration constants
  public static final boolean kEnableVision = true;
  public static final double kPoseAmbiguityThreshold = 0.2;
  public static final double kTargetLogTimeSeconds = 0.1;
  public static final double kFieldBorderMargin = 0.5; // meters
  public static final double kZMargin = 0.75; // meters

  // Stand 5-6 meters from a tag
  // Check tag area in PhotonVision UI
  // Set threshold to 80% of that value
  // Example: If tag shows 1200 pixels² at 5m:
  public static final double kMinTagAreaPixels = 1000.0; // Adjust based on testing
  
  // Standard deviation calculation constants
  public static final double kSingleTagBaseXYstdDev = 0.7; // meters
  public static final double kSingleTagBaseThetaStdDev = Units.degreesToRadians(10.0); // radians
  public static final double kMultiTagBaseXYstdDev = 0.3; // meters
  public static final double kMultiTagBaseThetaStdDev = Units.degreesToRadians(5.0); // radians
  public static final double kMaxDistanceMeters = 6.0; // anything over this is max std dev
}
