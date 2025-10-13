package frc.robot.sensors;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.Utils;

/**
 * Wrapper class for NavX 2 gyro that is used by the robot's swerve drive.
 */
public class NavX2Gyro implements Sendable {    
  // Hardware components
  private final AHRS gyro;
  
  // Instance fields
  private boolean connected = true;
  private int disconnectCount = 0;
  
  /**
   * Creates a new NavX2 gyro object.
   */
  public NavX2Gyro() {
    // Create the gyro
    gyro = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k50Hz);

    // Wait for gyro to calibrate itself on startup
    waitForGyroCalibration();

    // Set an offset if the gyro wasn't mounted with the X axis pointing forward.
    // NOTE: if the value is 0, you can safely comment this line out.
    gyro.setAngleAdjustment(DriveConstants.kGyroXAngleOffsetDegrees);

    // Initialize dashboard values
    SmartDashboard.putData("Drive/Gyro", gyro);
  }

  /**
   * Periodic method to be called by subsystem
   */
  public void periodic() {
    checkConnection();
  }

  /**
   * Waits for gyro calibration to complete (with timeout)
   */
  private void waitForGyroCalibration() {
    boolean timeout = false;
    int waitCount = 0;

    while (gyro.isCalibrating() && timeout == false) {
      // Wait for 1 second
      Timer.delay(1.0);
      waitCount++;
      
      // Print message every second
      Utils.logInfo("Calibrating gyro... (" + waitCount + "s). Do not move the robot!");
      
      // 20 seconds timeout
      if (waitCount > 20) { 
        timeout = true;
      }
    }

    // Print calibration complete message
    if (timeout) {
      Utils.logError("Gyro calibration completed with timeout.");
    }
    else if (!gyro.isMagnetometerCalibrated()) {
      Utils.logError("Gyro calibration complete. Magnetometer not calibrated!");
    }
    else {
      Utils.logInfo("Gyro calibration completed successfully.");
    }
  }

  /**
   * Simple debounced gryo disconnect detection.
   * The gyro must be disconnected for 10 consecutive 
   * checks to be considered disconnected.
   */
  private void checkConnection() {
    if (gyro.isConnected()) {
      disconnectCount = 0;
      connected = true;
    } else {
      disconnectCount++;
      if (disconnectCount > 10) {
        if (connected) {
          Utils.logError("Gyro disconnected! (checkConnection)"); // first time
        }
        connected = false;
      }
    }
  }

  /**
   * Returns the gyro's connection state.
   * @return True if the gyro is connected, false if disconnected
   */
  public boolean isConnected() {
    return connected;
  }

  /**
   * Gets the current gyro angle. This may not match the robot's heading
   * due to initial offset, or drift over time. Generally, this is only 
   * used as input into the swerve drive PoseEstimator, and then our 
   * robot can be driven based on the PoseEstimator's heading.
   * @return The current gyro angle as a Rotation2d, CCW positive
   */
  public Rotation2d getAngle() {
    // Return zero if gyro is disconnected - effectively forces robot-relative driving
    if (!connected) {
      return new Rotation2d();
    }

    // The getAngle() method should drive the same as the getYaw() because
    // we're turning returing it as Rototation2d and it is normalized by
    // the SwerveDrivePoseEstimator.
    // Return the gyro angle with the offset applied (CCW positive)
    return Rotation2d.fromDegrees(-gyro.getAngle());

    // Return the gyro yaw angle (CCW positive)
    // return Rotation2d.fromDegrees(-gyro.getYaw()); 
  }

  /**
   * Gets the current roll of the robot
   * @return Current roll in degrees
   */
  public double getRoll() {
    return gyro.getRoll();
  }
  
  /**
   * Gets the current pitch of the robot (for auto-balancing)
   * @return Current pitch in degrees
   */
  public double getPitch() {
    return gyro.getPitch();
  }
  
  /**
   * Gets the current yaw of the robot
   * @return Current yaw in degrees -180 to 180
   */
  public double getYaw() {
    return gyro.getYaw();
  }

  /**
   * Resets the gyro to zero.
   */
  public void reset() {
    gyro.reset(); 
    gyro.zeroYaw();
  }

  /**
   * Initialize the data sent to SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {}
}
