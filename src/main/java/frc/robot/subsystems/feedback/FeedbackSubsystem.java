// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feedback;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.feedback.FeedbackConstants.DisplayMode;
import frc.robot.util.Utils;

public class FeedbackSubsystem extends SubsystemBase {
  // LED hardware
  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;
  
  // Controller reference for rumble
  private CommandXboxController controller;
  
  // Display state
  private DisplayMode currentMode = DisplayMode.OFF;
  private double animationTimer = 0;
  private int animationOffset = 0;
  
  /** Creates a new FeedbackSubsystem. */
  public FeedbackSubsystem(CommandXboxController controller) {
    // Set controller reference
    this.controller = controller;

    // Initialize LED strip
    ledStrip = new AddressableLED(FeedbackConstants.LEDPort);
    ledBuffer = new AddressableLEDBuffer(FeedbackConstants.LEDLength);
    
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();

    // Set default command to display
    setDefaultCommand(teamColorsCommand());
    
    // Output initialization progress
    Utils.logInfo("Feedback subsystem initialized");
  }

  @Override
  public void periodic() {
    updateLEDDisplay();
    ledStrip.setData(ledBuffer);
  }
  
  // ==================== LED Control Methods ====================
  
  /**
   * Set the current LED display mode
   * @param mode Display mode to show
   */
  public void setDisplayMode(DisplayMode mode) {
    currentMode = mode;
    animationTimer = 0;
    animationOffset = 0;
  }
  
  /**
   * Update the LED buffer based on current display mode
   */
  private void updateLEDDisplay() {
    animationTimer += 0.02; // Assuming 20ms periodic cycle
    
    switch (currentMode) {
      case OFF:
        setAllLEDs(Color.kBlack);
        break;
        
      case IDLE:
        setAllLEDs(FeedbackConstants.IdleColor);
        break;
        
      case HAS_GAME_PIECE:
        setAllLEDs(FeedbackConstants.HasGamePieceColor);
        break;
        
      case INTAKING:
        pulsePattern(FeedbackConstants.IntakingColor, 0.5);
        break;
        
      case SHOOTING:
        pulsePattern(FeedbackConstants.ShootingColor, 0.3);
        break;
        
      case AUTO_ALIGN:
        chasePattern(FeedbackConstants.AutoAlignColor, 0.1);
        break;
        
      case WARNING:
        blinkPattern(FeedbackConstants.WarningColor, 0.25);
        break;
        
      case ERROR:
        blinkPattern(FeedbackConstants.ErrorColor, 0.15);
        break;
        
      case RAINBOW:
        rainbowPattern();
        break;
        
      case TEAM_COLORS:
        teamColorsPattern();
        break;
    }
  }
  
  /**
   * Set all LEDs to a single color
   * @param color Color to set
   */
  private void setAllLEDs(Color color) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, color);
    }
  }
  
  /**
   * Pulse pattern that fades in and out
   * @param color Base color
   * @param period Time for one complete pulse cycle in seconds
   */
  private void pulsePattern(Color color, double period) {
    double brightness = (Math.sin(animationTimer * 2 * Math.PI / period) + 1) / 2;
    Color scaledColor = new Color(
      color.red * brightness,
      color.green * brightness,
      color.blue * brightness
    );
    setAllLEDs(scaledColor);
  }
  
  /**
   * Blink pattern that turns on and off
   * @param color Color to blink
   * @param period Time for one complete blink cycle in seconds
   */
  private void blinkPattern(Color color, double period) {
    boolean on = (animationTimer % period) < (period / 2);
    setAllLEDs(on ? color : Color.kBlack);
  }
  
  /**
   * Chase pattern that moves along the strip
   * @param color Color to chase
   * @param speed Speed of the chase (lower = faster)
   */
  private void chasePattern(Color color, double speed) {
    if (animationTimer % speed < 0.02) {
      animationOffset = (animationOffset + 1) % ledBuffer.getLength();
    }
    
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if ((i + animationOffset) % 5 == 0) {
        ledBuffer.setLED(i, color);
      } else {
        ledBuffer.setLED(i, Color.kBlack);
      }
    }
  }
  
  /**
   * Rainbow pattern across the strip
   */
  private void rainbowPattern() {
    int offset = (int)(animationTimer * 50) % 180;
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      int hue = (offset + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }
  }
  
  /**
   * Team colors gradient pattern that chases across the strip
   * Creates a smooth green-to-copper gradient that moves along the LEDs
   */
  private void teamColorsPattern() {
    // Update offset for chase effect (moves 10 pixels per second)
    if (animationTimer % 0.1 < 0.02) {
      animationOffset = (animationOffset + 1) % ledBuffer.getLength();
    }
    
    // Length of one complete gradient cycle (in LEDs)
    int gradientLength = ledBuffer.getLength() / 2;
    
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate position in gradient with offset for chase effect
      int position = (i + animationOffset) % gradientLength;
      double gradientPosition = (double) position / gradientLength;
      
      // Interpolate between green and copper
      Color interpolatedColor = interpolateColor(
          FeedbackConstants.TeamGreen,
          FeedbackConstants.TeamCopper,
          gradientPosition
      );
      
      ledBuffer.setLED(i, interpolatedColor);
    }
  }
  
  /**
   * Interpolate between two colors
   * @param color1 Starting color
   * @param color2 Ending color
   * @param t Position in gradient (0.0 to 1.0)
   * @return Interpolated color
   */
  private Color interpolateColor(Color color1, Color color2, double t) {
    double r = color1.red + (color2.red - color1.red) * t;
    double g = color1.green + (color2.green - color1.green) * t;
    double b = color1.blue + (color2.blue - color1.blue) * t;
    return new Color(r, g, b);
  }
  
  // ==================== Rumble Control Methods ====================
  
  /**
   * Set controller rumble
   * @param intensity Rumble intensity (0.0 to 1.0)
   */
  public void setRumble(double intensity) {
    if (controller != null) {
      double clampedIntensity = MathUtil.clamp(intensity, 0.0, 1.0);
      controller.getHID().setRumble(RumbleType.kBothRumble, clampedIntensity);
    }
  }
  
  /**
   * Stop controller rumble
   */
  public void stopRumble() {
    setRumble(0);
  }
  
  // ==================== Command Factories ====================
  
  /**
   * Command to set LED display mode
   * @param mode Display mode to show
   * @return Command that sets the LED display
   */
  public Command setDisplayCommand(DisplayMode mode) {
    return runOnce(() -> setDisplayMode(mode))
      .withName("SetLED_" + mode.name());
  }
  
  /**
   * Command to rumble controller for a duration
   * @param intensity Rumble intensity (0.0 to 1.0)
   * @param duration Duration in seconds
   * @return Command that rumbles then stops
   */
  public Command rumbleCommand(double intensity, double duration) {
    return run(() -> setRumble(intensity))
      .withTimeout(duration)
      .finallyDo(this::stopRumble)
      .withName("Rumble");
  }
  
  /**
   * Command for short rumble pulse
   * @return Command that does a quick rumble
   */
  public Command quickRumbleCommand() {
    return rumbleCommand(0.5, 0.2);
  }
  
  /**
   * Command for strong rumble pulse
   * @return Command that does a strong rumble
   */
  public Command strongRumbleCommand() {
    return rumbleCommand(1.0, 0.4);
  }
  
  /**
   * Command for double rumble pulse
   * @return Command that rumbles twice
   */
  public Command doubleRumbleCommand() {
    return rumbleCommand(0.7, 0.15)
      .andThen(Commands.waitSeconds(0.1))
      .andThen(rumbleCommand(0.7, 0.15));
  }
  
  /**
   * Command to indicate game piece acquired
   * @return Command with LED and rumble feedback
   */
  public Command gamePieceAcquiredCommand() {
    return Commands.parallel(
      setDisplayCommand(DisplayMode.HAS_GAME_PIECE),
      quickRumbleCommand()
    ).withName("GamePieceAcquired");
  }
  
  /**
   * Command to indicate successful shot
   * @return Command with LED and rumble feedback
   */
  public Command successfulShotCommand() {
    return Commands.parallel(
      setDisplayCommand(DisplayMode.SHOOTING)
        .andThen(Commands.waitSeconds(0.5))
        .andThen(setDisplayCommand(DisplayMode.IDLE)),
      doubleRumbleCommand()
    ).withName("SuccessfulShot");
  }
  
  /**
   * Command to indicate auto-alignment complete
   * @return Command with LED and rumble feedback
   */
  public Command autoAlignCompleteCommand() {
    return Commands.parallel(
      setDisplayCommand(DisplayMode.AUTO_ALIGN)
        .andThen(Commands.waitSeconds(0.3))
        .andThen(setDisplayCommand(DisplayMode.IDLE)),
      rumbleCommand(0.3, 0.2)
    ).withName("AutoAlignComplete");
  }
  
  /**
   * Command to indicate error state
   * @return Command with LED and rumble feedback
   */
  public Command errorCommand() {
    return Commands.parallel(
      setDisplayCommand(DisplayMode.ERROR),
      rumbleCommand(1.0, 0.5)
    ).withName("Error");
  }
  
  /**
   * Command to indicate warning
   * @return Command with LED and rumble feedback
   */
  public Command warningCommand() {
    return Commands.parallel(
      setDisplayCommand(DisplayMode.WARNING),
      rumbleCommand(0.4, 0.3)
    ).withName("Warning");
  }
  
  /**
   * Command to set idle state
   * @return Command that sets idle LED display
   */
  public Command idleCommand() {
    return setDisplayCommand(DisplayMode.IDLE);
  }
  
  /**
   * Command to display team colors gradient
   * @return Command that shows green-to-copper gradient chase
   */
  public Command teamColorsCommand() {
    return setDisplayCommand(DisplayMode.TEAM_COLORS);
  }
}
