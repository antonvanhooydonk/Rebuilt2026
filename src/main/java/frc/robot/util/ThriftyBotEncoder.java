package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Wrapper class for Thrifty Bot absolute encoder attached to swerve steer motor.
 * Provides both raw AnalogInput access and processed AnalogEncoder functionality.
 */
public class ThriftyBotEncoder implements Sendable {    
  // Hardware components
  private final AnalogInput analogInput;
  private final AnalogEncoder analogEncoder;
  
  // Configuration
  private final String name;
  private double offsetRadians;
  private boolean inverted = false;
  
  // Voltage thresholds for diagnostics
  private static final double MIN_VALID_VOLTAGE = 0.1;
  private static final double MAX_VALID_VOLTAGE = 4.9;
  private static final double VOLTAGE_NOISE_THRESHOLD = 0.05;
  
  // Previous reading for noise detection
  private double previousVoltage = 0.0;
  private int noiseCount = 0;
  
  /**
   * Creates a new ThriftyBotEncoder
   * @param analogPort The analog input port (0-3 on roboRIO)
   * @param offsetRadians Offset in radians to zero the encoder
   * @param name Name for debugging and dashboard
   */
  public ThriftyBotEncoder(int analogPort, double offsetRadians, boolean inverted, String name) {
    this.name = name;
    this.offsetRadians = offsetRadians;
    this.inverted = inverted;
    
    // Initialize both analog input and encoder
    this.analogInput = new AnalogInput(analogPort);
    this.analogEncoder = new AnalogEncoder(analogInput);

    // Initialize dashboard values
    SmartDashboard.putData("Encoder/" + name, this);
  }

  /**
   * Periodic method to be called from parent's periodic for diagnostics
   */
  public void periodic() {}
  
  /**
   * Gets the current angle in radians using AnalogEncoder
   * @return Angle in radians (-π to π)
   */
  public double getAngleRadians() {
    // Get position from AnalogEncoder (0.0 to 1.0 rotations)
    double position = analogEncoder.get();
    
    // Convert position to radians (0 to 2π)
    double angle = position * 2.0 * Math.PI;
    
    // Apply offset
    angle -= offsetRadians;
    
    // Apply inversion if needed
    if (inverted) {
      angle = -angle;
    }
    
    // Normalize to [-π, π]
    return normalizeAngle(angle);
  }
  
  /**
   * Gets the current angle as a Rotation2d
   * @return Rotation2d object
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRadians(getAngleRadians());
  }
  
  /**
   * Gets the current angle in degrees
   * @return Angle in degrees (-180 to 180)
   */
  public double getAngleDegrees() {
    return Units.radiansToDegrees(getAngleRadians());
  }
  
  /**
   * Gets the raw voltage from the analog input
   * @return Voltage (0-5V)
   */
  public double getRawVoltage() {
    return analogInput.getVoltage();
  }
  
  /**
   * Gets the current angle (0 to 2π) from the analog input.
   * This method does not apply the internal offset.
   * @return Raw angle in radians
   */
  public double getRawAngleRadians() {
    double voltage = getRawVoltage();
    double supplyVoltage = RobotController.getVoltage5V();
    double angle = (voltage / supplyVoltage) * 2.0 * Math.PI;    
    return inverted ? -angle : angle;
  }
  
  /**
   * Gets the absolute position (0 to 1.0 rotations).
   * This method does not apply the internal offset.
   * @return Position as fraction of full rotation
   */
  public double getAbsolutePosition() {
    return analogEncoder.get();
  }
  
  /**
   * Sets whether the encoder direction should be inverted
   * @param inverted True to invert the direction
   */
  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }
  
  /**
   * Gets whether the encoder is inverted
   * @return True if inverted
   */
  public boolean isInverted() {
    return inverted;
  }
  
  /**
   * Sets a new offset for the encoder (updates internal offset)
   * @param offsetRadians New offset in radians
   */
  public void setOffset(double offsetRadians) {
    this.offsetRadians = offsetRadians;
  }
  
  /**
   * Gets the current offset
   * @return Current offset in radians
   */
  public double getOffset() {
    return offsetRadians;
  }
  
  /**
   * Calibrates the encoder by calculating offset from current position.
   * Can be called manually after aligning the modules with a straignt
   * bar and pointing in the desired zero direction.
   */
  public void calibrate() {
    // Get current raw position
    double currentPosition = analogEncoder.get(); // 0.0 to 1.0
    double currentAngleRad = currentPosition * 2.0 * Math.PI; // 0 to 2π
    
    // Update the offset to make current position read as zero
    this.offsetRadians = currentAngleRad;
  }
  
  /**
   * Checks if the encoder readings are valid
   * @return True if readings appear valid
   */
  public boolean isValid() {
    double voltage = getRawVoltage();
    
    // Check voltage range
    if (voltage < MIN_VALID_VOLTAGE || voltage > MAX_VALID_VOLTAGE) {
      return false;
    }
    
    // Check for excessive noise
    double voltageDelta = Math.abs(voltage - previousVoltage);
    if (voltageDelta > VOLTAGE_NOISE_THRESHOLD) {
      noiseCount++;
    } else {
      noiseCount = Math.max(0, noiseCount - 1); // Decay noise count
    }
    
    // Cache previous voltage reading
    previousVoltage = voltage;
    
    // Consider invalid if consistently noisy
    return noiseCount < 10;
  }

  /**
   * Gets the current noise count for diagnostics
   * @return
   */
  public int getNoiseCount() {
    return noiseCount;
  }

  /**
   * Gets the analog input port number
   * @return Port number
   */
  public int getPort() {
    return analogInput.getChannel();
  }

  /**
   * Gets the encoder name
   * @return Name string
   */
  public String getName() {
    return name;
  }

  /**
   * Normalizes an angle to [-π, π] range
   * @param angle Angle in radians
   * @return Normalized angle
   */
  private double normalizeAngle(double angle) {
    while (angle > Math.PI) {
      angle -= 2.0 * Math.PI;
    }
    while (angle < -Math.PI) {
      angle += 2.0 * Math.PI;
    }
    return angle;
  }

  /**
   * Closes the encoder and frees resources
   */
  public void close() {
    analogInput.close();
    analogEncoder.close();
  }
  
  /**
   * Initialize the data sent to SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Angle (deg)", this::getAngleDegrees, null);
    builder.addDoubleProperty("Angle (rad)", this::getAngleRadians, null);
    builder.addDoubleProperty("Raw Voltage", this::getRawVoltage, null);
    builder.addDoubleProperty("Supply Voltage", RobotController::getVoltage5V, null);
    builder.addDoubleProperty("Absolute Position", this::getAbsolutePosition, null);
    builder.addBooleanProperty("Valid", this::isValid, null);
    builder.addBooleanProperty("Inverted", this::isInverted, null);
    builder.addDoubleProperty("Noise Count", this::getNoiseCount, null);
  }
}
