// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.vision.VisionSubsystem;

public final class Utils {
  /**
   * Determine if the robot is currently on the red alliance.
   * @return True if the robot is on the red alliance, false otherwise.
   */
  public static boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  /**
   * Return the left side scoring pose for a specified camera.
   * @return Pose2d where the robot can score from.
   */
  public static Pose2d getLeftScoringPose(VisionSubsystem vs, String camera) {
    if (vs.getBestTarget(camera).isPresent()) {
      return FieldConstants.leftScoringPoses.get(vs.getBestTarget(camera).get().getFiducialId());
    }
    return null;
  }

  /**
   * Return the right side scoring pose for a specified camera.
   * @return Pose2d where the robot can score from.
   */
  public static Pose2d getRightScoringPose(VisionSubsystem vs, String camera) {
    if (vs.getBestTarget(camera).isPresent()) {
      return FieldConstants.rightScoringPoses.get(vs.getBestTarget(camera).get().getFiducialId());
    }
    return null;
  }
}
