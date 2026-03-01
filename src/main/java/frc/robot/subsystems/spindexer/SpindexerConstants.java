package frc.robot.subsystems.spindexer;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class SpindexerConstants {

  public static final double kSensorToMechanismGearing = 3.0;

  public static final double kStatusSignalUpdateFrequencyHz = 100.0;
  public static final int kLinearFilterSampleCount = 5;

  public static final double kDefaultSpeedRPS = 27.0;


  public record SpindexerHardware(
      int motorIDIndex, double motorRotationsToIndexRotations) {}

  public record SpindexerGains(
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

  public record SpindexerMotorConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      NeutralModeValue neutralMode) {}

  public static final SpindexerHardware kIndexHardware =
      new SpindexerHardware(
          25,
          kSensorToMechanismGearing);

  public static final SpindexerGains kSpindexIndexGains =
      switch (Constants.currentMode) {
        case REAL -> new SpindexerGains(1, 0.0, 0.0, 0.0, 0.0, 0.355, 0.0, 120.0, 240.0, 0);

        case SIM -> new SpindexerGains(8.0, 0.0, 0.2, 0.1, 0.3, 1.0, 0.03, 180.0, 360.0, 0);

        default -> new SpindexerGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
      };

  public static final SpindexerMotorConfiguration kMotorConfiguration =
      new SpindexerMotorConfiguration(
          false, true, true, 80.0, 20.0, 12.0, -12.0, NeutralModeValue.Coast);
}
