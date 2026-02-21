package frc.robot.subsystems.shooterhood;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class HoodConstants {

  public static final double kGearRatio = 31.6; // rotations motor per rotations pivot
  public static final double kRotorOffset = -0.23388671875; // rotations, find in TunerX

  public static final double kMinRadians = 0.00;
  public static final double kMaxRadians = 1.0;

  public static final double kPositionToleranceRad = 0.005;

  public static final double kStatusSignalUpdateFrequencyHz = 100.0;

  public record HoodHardware(
      int motorID, double gearRatio, double rotorOffset) {}

  public record HoodGains(
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

  public record HoodMotorConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}
      // TODO: add min/max soft limits

  public static final HoodHardware kHoodHardware =
      new HoodHardware(
          52, // left motor CAN ID
          kGearRatio,
          kRotorOffset
          );

  public static final HoodGains kHoodGains =
      switch (Constants.currentMode) {
        case REAL -> new HoodGains(300, 0.0, 0.0, 0.15, 0.0, 0.0, 0.05, 120.0, 240.0, 0);

        case SIM -> new HoodGains(8.0, 0.0, 0.2, 0.1, 0.3, 1.0, 0.03, 180.0, 360.0, 0);

        default -> new HoodGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
      };

  public static final HoodMotorConfiguration kMotorConfiguration =
      new HoodMotorConfiguration(
          true, true, true, 80.0, 50.0, 1.5, -1.5, NeutralModeValue.Brake);
}
