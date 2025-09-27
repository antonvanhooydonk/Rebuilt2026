// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;

import frc.robot.util.Utils;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED strip;
  private AddressableLEDBuffer buffer;
    
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    // Initialize the LED strip object and buffer.
    strip = new AddressableLED(LEDConstants.kPWMID);
    buffer = new AddressableLEDBuffer(LEDConstants.kPixelCount);
    strip.setLength(buffer.getLength());
    strip.start();

    // Set the default command to run on the LED strip 
    // when no other commands are using the LEDs.
    if (Utils.isRedAlliance()) {
      setDefaultCommand(showRedCommand().withName("DEFAULT"));
    } else {
      setDefaultCommand(showBlueCommand().withName("DEFAULT"));
    }

    // Output initialization progress
    Utils.logInfo("LED subsystem intialized");
  }

  /**
   * Periodically send the latest LED colour data 
   * buffer to the LED strip for it to display.
   */
  @Override
  public void periodic() {
    strip.setData(buffer);
  }

  /**
   * Show a solid black colour on the LED strip.
   */
  public Command showOffCommand() {
    return run(() -> LEDPattern.solid(Color.kBlack).applyTo(buffer)).ignoringDisable(true);
  }

  /**
   * Show a solid blue colour on the LED strip.
   */
  public Command showBlueCommand() {
    return run(() -> LEDPattern.solid(Color.kBlue).applyTo(buffer)).ignoringDisable(true);
  }

  /**
   * Show a solid red colour on the LED strip.
   */
  public Command showRedCommand() {
    return run(() -> LEDPattern.solid(Color.kRed).applyTo(buffer)).ignoringDisable(true);
  }

  /**
   * Show the Copper Hawk colours on the LED strip.
   */
  public Command showCopperHawksCommand() {
    return run(() -> 
      LEDPattern.gradient(
        GradientType.kContinuous, 
        Color.kDarkOrange, 
        Color.kForestGreen
      )
      .scrollAtAbsoluteSpeed(
        MetersPerSecond.of(0.5), 
        Meters.of(LEDConstants.kStripLengthMeters / buffer.getLength())
      )
      .applyTo(buffer)
    )
    .ignoringDisable(true);
  }

  /**
   * Show a blinking green colour on the LED strip.
   */
  public Command showBlinkingCommand() {
    return run(() -> 
      LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.25)).applyTo(buffer)
    )
    .repeatedly()
    .withTimeout(1.0);
  }
}
