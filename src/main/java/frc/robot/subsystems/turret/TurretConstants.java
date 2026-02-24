package frc.robot.subsystems.turret;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class TurretConstants {

  public static final double kSensorMechanismRatio = 3.4; // sensor to mechanism
  public static final double kRotorSensorGearRatio = 15; // rotor to sensor
  public static final double kCancoderOffset = -0.4721679687;

  public static final double kRotorRotationsToDegrees = 360.0 / kSensorMechanismRatio;

  public static final double kHomeRadians = 0;
  public static final double kToleranceRotations = 0.5;

  public static final double kMinRadiansLimit = -1.1;
  public static final double kMaxRadiansLimit = 1.3;

  public static final double kTurretOffsetX = -0.132;
  public static final double kTurretOffsetY = 0.158;

  public static final double kStatusSignalUpdateFrequencyHz = 100.0;

  public record TurretHardware(
      int motorIDLeft, int cancoderID, double sensorMechanismGearRatio, double rotorSensorGearRatio, double rotorRotationsToDegrees, double cancoderOffset) {}

  public record TurretGains(
      double p,
      double i,
      double d,
      double s,
      double g,
      double v,
      double a,
      double maxVelocityDegPerSec,
      double maxAccelerationDegPerSec2,
      double maxJerkDegPerSec3) {}

  public record TurretMotorConfiguration(
      boolean invert,
      boolean invertFeedback,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public static final TurretHardware kTurretHardware =
      new TurretHardware(
          21, // motor CAN ID
          17, // cancoder CAN ID
          kSensorMechanismRatio,
          kRotorSensorGearRatio,
          kRotorRotationsToDegrees,
          kCancoderOffset);

  public static final TurretGains kTurretGains =
      switch (Constants.currentMode) {
        case REAL -> new TurretGains(155, 0.0, 0.0, 0.0, 0.0, 0.0, 0.00, 120.0, 240.0, 0);

        case SIM -> new TurretGains(8.0, 0.0, 0.2, 0.1, 0.3, 1.0, 0.03, 180.0, 360.0, 0);

        default -> new TurretGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
      };

  public static final TurretMotorConfiguration kMotorConfiguration =
      new TurretMotorConfiguration(
          false, false, true, true, 80.0, 50.0, 12.0, -12.0, NeutralModeValue.Brake);
}
