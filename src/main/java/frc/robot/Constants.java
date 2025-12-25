// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class FieldConstants {
    public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    public static final double fieldLengthMeters = Units.inchesToMeters(651.25); // meters
    public static final double fieldWidthMeters = Units.inchesToMeters(315.5); // meters
    public static final HashMap<Integer, Pose2d> leftScoringPoses = getLeftScoringPoses();
    public static final HashMap<Integer, Pose2d> rightScoringPoses = getRightScoringPoses();

    /**
     * Define a hashmap of AprilTag poses that represent our 
     * left side scoring poses for each reef AprilTag
     * @return hashmap 
     */
    private static HashMap<Integer, Pose2d> getLeftScoringPoses() {      
      HashMap<Integer, Pose2d> map = new HashMap<>();
      map.put( 6, new Pose2d(13.533212698834394, 2.845061489129294, Rotation2d.fromDegrees(300)) );
      map.put( 7, new Pose2d(14.3193412054922, 3.8461651986517804, Rotation2d.fromDegrees(0)) );   
      map.put( 8, new Pose2d(13.844522506657809, 5.027003709522487, Rotation2d.fromDegrees(60)) );
      map.put( 9, new Pose2d(12.584591301165608, 5.206738510870706, Rotation2d.fromDegrees(120)) );
      map.put( 10, new Pose2d(11.798462794507797, 4.20563480134822, Rotation2d.fromDegrees(180)) );
      map.put( 11, new Pose2d(12.273281493342191, 3.0247962904775134, Rotation2d.fromDegrees(240)) );
      map.put( 17, new Pose2d(3.7038294933421905, 3.0247962904775134, Rotation2d.fromDegrees(240)) );
      map.put( 18, new Pose2d(3.2287567945077993, 4.20563480134822, Rotation2d.fromDegrees(180)) );
      map.put( 19, new Pose2d(4.015139301165607, 5.206738510870706, Rotation2d.fromDegrees(120)) );
      map.put( 20, new Pose2d(5.274816506657808, 5.027003709522487, Rotation2d.fromDegrees(60)) );
      map.put( 21, new Pose2d(5.749889205492201, 3.8461651986517804, Rotation2d.fromDegrees(0)) );
      map.put( 22, new Pose2d(4.963506698834392, 2.845061489129294, Rotation2d.fromDegrees(300)) );
      return map;
    }
      
    /**
     * Define a hashmap of AprilTag poses that represent our 
     * right side scoring poses for each reef AprilTag
     * @return hashmap 
     */
    private static HashMap<Integer, Pose2d> getRightScoringPoses() {
      HashMap<Integer, Pose2d> map = new HashMap<>();
      map.put( 6, new Pose2d(13.844522506657809, 3.0247962904775134, Rotation2d.fromDegrees(300)) );
      map.put( 7, new Pose2d(14.3193412054922, 4.20563480134822, Rotation2d.fromDegrees(0)) );
      map.put( 8, new Pose2d(13.533212698834394, 5.206738510870706, Rotation2d.fromDegrees(60)) );
      map.put( 9, new Pose2d(12.273281493342191, 5.027003709522487, Rotation2d.fromDegrees(120)) );
      map.put( 10, new Pose2d(11.798462794507797, 3.8461651986517804, Rotation2d.fromDegrees(180)) );
      map.put( 11, new Pose2d(12.584591301165606, 2.845061489129294, Rotation2d.fromDegrees(240)) );
      map.put( 17, new Pose2d(4.015139301165607, 2.845061489129294, Rotation2d.fromDegrees(240)) );
      map.put( 18, new Pose2d(3.2287567945077993, 3.8461651986517804, Rotation2d.fromDegrees(180)) );
      map.put( 19, new Pose2d(3.703829493342191, 5.027003709522487, Rotation2d.fromDegrees(120)) );
      map.put( 20, new Pose2d(4.963506698834392, 5.206738510870706, Rotation2d.fromDegrees(60)) );
      map.put( 21, new Pose2d(5.749889205492201, 4.20563480134822, Rotation2d.fromDegrees(0)) );
      map.put( 22, new Pose2d(5.274816506657808, 3.0247962904775134, Rotation2d.fromDegrees(300)) );
      return map;
    }
  }

  public static class Controller1Constants {  
    public static final int ButtonBlue1   =  3;
    public static final int ButtonBlack1  =  5;
    public static final int ButtonBlack2  =  6;
    public static final int ButtonGreen   =  1;  
    public static final int ButtonYellow  =  7;
    public static final int ButtonPlayer1 =  8;  
    public static final int ButtonPlayer2 =  2;
  }

  public static class Controller2Constants { 
    public static final int ButtonRed1    =  5;
    public static final int ButtonRed2    =  4;
    public static final int ButtonRed3    =  3;
    public static final int ButtonRed4    =  2;
    public static final int ButtonRed5    =  1;
    public static final int ButtonBlue2   =  8;
    public static final int ButtonBlue3   =  7;
    public static final int ButtonBlue4   =  6;
  }

  public static class CANConstants {
    public static final int RightElevatorID  = 1;
    public static final int LeftElevatorID   = 2;
    public static final int ArmID            = 3;
    public static final int RollerID         = 4;
  }
  
  public static final class DIOConstants {
    public static final int BeamBreakSensorPort = 0;
  }
}
