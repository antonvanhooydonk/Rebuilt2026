// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feedback;

import edu.wpi.first.wpilibj.util.Color;

/**
 * Constants for the Feedback subsystem
 */
public final class FeedbackConstants {
  
  // ==================== LED Hardware ====================
  
  /** PWM port for LED strip */
  public static final int LEDPort = 0;
  
  /** Number of LEDs in the strip */
  public static final int LEDLength = 60;
  
  // ==================== LED Colors ====================
  
  /** Color when robot is idle/ready (soft blue) */
  public static final Color IdleColor = new Color(0.0, 0.3, 1.0);
  
  /** Color when robot has game piece (green) */
  public static final Color HasGamePieceColor = Color.kGreen;
  
  /** Color when intaking (yellow) */
  public static final Color IntakingColor = Color.kYellow;
  
  /** Color when shooting (orange) */
  public static final Color ShootingColor = Color.kOrange;
  
  /** Color during auto-alignment (purple) */
  public static final Color AutoAlignColor = Color.kPurple;
  
  /** Color for warnings (orange) */
  public static final Color WarningColor = new Color(1.0, 0.5, 0.0);
  
  /** Color for errors (red) */
  public static final Color ErrorColor = Color.kRed;
  
  // ==================== Team Colors (Optional) ====================
  
  /** Team color - Green */
  public static final Color TeamGreen = new Color(0.0, 0.8, 0.2);
  
  /** Team color - Copper */
  public static final Color TeamCopper = new Color(0.72, 0.45, 0.20);
  
  // ==================== LED Display Modes ====================
  
  /**
   * Enum representing different LED display modes
   */
  public enum DisplayMode {
    /** All LEDs off */
    OFF,
    
    /** Robot idle/ready state */
    IDLE,
    
    /** Robot has game piece */
    HAS_GAME_PIECE,
    
    /** Intaking game piece */
    INTAKING,
    
    /** Shooting/scoring */
    SHOOTING,
    
    /** Auto-aligning to target */
    AUTO_ALIGN,
    
    /** Warning state */
    WARNING,
    
    /** Error state */
    ERROR,
    
    /** Rainbow animation */
    RAINBOW,
    
    /** Team colors gradient chase */
    TEAM_COLORS
  }
}
