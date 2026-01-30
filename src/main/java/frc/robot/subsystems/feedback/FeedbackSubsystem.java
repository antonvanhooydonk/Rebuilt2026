// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feedback;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.PWMConstants;
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
  private char inactiveAlliance = '?';
  
  /** Creates a new FeedbackSubsystem. */
  public FeedbackSubsystem(CommandXboxController controller) {
    // Set controller reference
    this.controller = controller;

    // Initialize LED strip
    ledStrip = new AddressableLED(PWMConstants.LEDStripID);
    ledBuffer = new AddressableLEDBuffer(FeedbackConstants.LEDLength);
    
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();

    // Set default command to display
    setDefaultCommand(scoringShiftCommand('?'));
    
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
  private void setDisplayMode(DisplayMode mode) {
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
        chasePattern(FeedbackConstants.IdleColor, 0.50);
        break;
        
      case WARNING:
        blinkPattern(FeedbackConstants.WarningColor, 0.25);
        break;
        
      case ERROR:
        blinkPattern(FeedbackConstants.ErrorColor, 0.15);
        break;
        
      case TEAM_COLORS:
        teamColorsPattern();
        break;

      case SCORING_SHIFT:
        scoringShiftPattern();
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
    // Calculate offset directly from time
    animationOffset = (int)(animationTimer / speed) % ledBuffer.getLength();
    
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if ((i + animationOffset) % 5 == 0) {
        ledBuffer.setLED(i, color);
      } else {
        ledBuffer.setLED(i, Color.kBlack);
      }
    }
  }
  
  /**
   * Team colors gradient pattern that chases across the strip
   * Creates a smooth green-to-copper gradient that moves along the LEDs
   */
  private void teamColorsPattern() {
    // Calculate offset directly from time
    animationOffset = (int)(animationTimer * 10) % ledBuffer.getLength(); // 10 pixels/second
      
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
  
  /**
   * Scoring shift pattern that indicates which alliance is allowed to score.
   * Sets all LEDs to our alliance colour when our scoring shift is activate.
   * Blinks our alliance colour for the last 5 seconds of active hub.
   * Turns off the LEDs when it is not our turn to score.
   */
  private void scoringShiftPattern() {
    // current remaining match time
    double time = Timer.getMatchTime();

    // set our alliance color
    Color allianceColor = Utils.isRedAlliance() ? Color.kRed : Color.kBlue;

    // autonomous period 20 seconds (both alliances can score)
    // set to 'A' in Robot.autonomousInit()
    if (inactiveAlliance == 'A') {
      if (time <= 20 && time > 5) {
        // autonomous (both alliances can score)
        setAllLEDs(allianceColor);
      }
      else if (time <= 5 && time > 0) {
        // autonomous - last 5 seconds (both alliances can score)
        pulsePattern(allianceColor, 0.35);
      }
      else {
        // default to off
        setAllLEDs(Color.kBlack);
      }
    }

    // Check that we have valid game data
    if (inactiveAlliance == 'R' || inactiveAlliance == 'B') {
      // check if our alliance is inactive for the 1st shift
      boolean isInactiveFirst = 
        (inactiveAlliance == 'R' && Utils.isRedAlliance()) ||
        (inactiveAlliance == 'B' && !Utils.isRedAlliance());

      // Shift data from 2026 FRC Game Manual
      // https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf
      if (time <= 140 && time > 135) {
        // transition shift (both alliances can score)
        setAllLEDs(allianceColor);
      }
      else if (time <= 135 && time > 130) {
        // transition shift - last 5 seconds (both alliances can score)
        pulsePattern(allianceColor, 0.35);
      }
      else if (time <= 130 && time > 110) {
        // shift 1
        setAllLEDs(isInactiveFirst ? Color.kBlack : allianceColor);
      }
      else if (time <= 110 && time > 105) {
        // shift 1 - last 5 seconds
        pulsePattern(isInactiveFirst ? Color.kBlack : allianceColor, 0.35);
      }
      else if (time <= 105 && time > 85) {
        // shift 2
        setAllLEDs(isInactiveFirst ? allianceColor : Color.kBlack);
      }
      else if (time <= 85 && time > 80) {
        // shift 2 - last 5 seconds
        pulsePattern(isInactiveFirst ? allianceColor : Color.kBlack, 0.35);
      }
      else if (time <= 80 && time > 60) {
        // shift 3
        setAllLEDs(isInactiveFirst ? Color.kBlack : allianceColor);
      }
      else if (time <= 60 && time > 55) {
        // shift 3 - last 5 seconds
        pulsePattern(isInactiveFirst ? Color.kBlack : allianceColor, 0.35);
      }
      else if (time <= 55 && time > 35) {
        // shift 4
        setAllLEDs(isInactiveFirst ? allianceColor : Color.kBlack);
      }
      else if (time <= 35 && time > 30) {
        // shift 4 - last 5 seconds
        pulsePattern(isInactiveFirst ? allianceColor : Color.kBlack, 0.35);
      }
      else if (time <= 30 && time > 5) {
        // end game (both alliances can score)
        setAllLEDs(allianceColor);
      }
      else if (time <= 5 && time > 0) {
        // end game - last 5 seconds (both alliances can score)
        pulsePattern(allianceColor, 0.35);
      }
      else {
        // default to off
        setAllLEDs(Color.kBlack);
      }
    }
  }

  // ==================== Rumble Control Methods ====================
  
  /**
   * Set controller rumble
   * @param intensity Rumble intensity (0.0 to 1.0)
   */
  private void setRumble(double intensity) {
    if (controller != null) {
      double clampedIntensity = MathUtil.clamp(intensity, 0.0, 1.0);
      controller.getHID().setRumble(RumbleType.kBothRumble, clampedIntensity);
    }
  }
  
  /**
   * Stop controller rumble
   */
  private void stopRumble() {
    setRumble(0);
  }
  
  // ==================== Command Factories ====================
  
  /**
   * Command to set LED display mode
   * @param mode Display mode to show
   * @return Command that sets the LED display
   */
  public Command setDisplayCommand(DisplayMode mode) {
    return runOnce(() -> setDisplayMode(mode)).withName("SetLED_" + mode.name());
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
   * Command to indicate error state
   * @return Command with LED and rumble feedback
   */
  public Command errorCommand() {
    return setDisplayCommand(DisplayMode.ERROR)
      .andThen(rumbleCommand(1.0, 0.5))
      .withName("Error");
  }
  
  /**
   * Command to indicate warning
   * @return Command with LED and rumble feedback
   */
  public Command warningCommand() {
    return setDisplayCommand(DisplayMode.WARNING)
      .andThen(rumbleCommand(0.4, 0.3))
      .withName("Warning");
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
  
  /**
   * Function to schedule scoring shift feedback
   */
  public Command scoringShiftCommand(char inactiveAlliance) {
    return runOnce(() -> {
      this.inactiveAlliance = inactiveAlliance;
      setDisplayMode(DisplayMode.SCORING_SHIFT);
    });
  }
}
