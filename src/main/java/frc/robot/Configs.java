package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
    public static final SparkMaxConfig armConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor =
          ModuleConstants.kWheelDiameterMeters * Math.PI
              / ModuleConstants.kDrivingMotorReduction;

      // SDS MK4i turning ratio (150/7) with radians conversion (2pi)
      double turningFactor = (2 * Math.PI * 7.0) / 150.0;

      double drivingVelocityFeedForward =
          1 / ModuleConstants.kDriveWheelFreeSpeedRps;

      /* ===================== DRIVING ===================== */
      drivingConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(50);

      drivingConfig.encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters/sec

      drivingConfig.closedLoop
          // Encoder is automatically used
          .pid(0.04, 0, 0)
          .velocityFF(drivingVelocityFeedForward)
          .outputRange(-1, 1);

      /* ===================== TURNING ===================== */
      turningConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(20)
          .inverted (true); //(false);

      turningConfig.encoder
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0); // rad/sec

      turningConfig.closedLoop
          // Encoder is automatically used
          .pid(0.2, 0, 0)
          .outputRange(-1, 1);

      // ⚠ DO NOT enable position wrapping (as you already noted)

      /* ===================== ARM ===================== */
      armConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .inverted(false);

      armConfig.softLimit
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimitEnabled(false);

      armConfig.encoder
          .positionConversionFactor(2 * Math.PI) // radians
          .velocityConversionFactor((Math.PI / 3) / 60.0); // rad/sec

      armConfig.closedLoop
          // Encoder is automatically used
          .pid(2, 0, 0)
          .outputRange(-1, 1);
    }
  }
}
