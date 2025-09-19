// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class ElevatorConstants {   
  public static final boolean ENABLE = true;
  public static double HomePosition = 0.0;
  public static double Lvl1Position = 50.0;
  public static double Lvl2Position = 235.0;//255.0;
  public static double Lvl3Position = 550.0;//605.0;
  public static double Lvl4Position = 1120.0;//1150.0;
  public static double MaxHeightPosition = 1150.0;  
  public static double LowerAlgaePosition = 330.0;
  public static double UpperAlgaePosition = 660.0;
  public static double NetAlgaePosition = 1080.0;
  public static double ErrorThreshold = 30.0;
  public static double MMVelocity = 1200;
  public static double MMAcceleration = 3000;
  public static double MMJerk = 0;    
}
