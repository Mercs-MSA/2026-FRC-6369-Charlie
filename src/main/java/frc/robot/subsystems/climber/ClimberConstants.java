package frc.robot.subsystems.climber;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ClimberConstants {
  public static final double kRotorRotationsToMeters = 5;
  public static final double kMaxPositionMeters = Units.inchesToMeters(62.0);
  public static final double kMinPositionMeters = Units.inchesToMeters(-0.1);
  public static final double kPositionToleranceRadians = 0.08;
  public static final double kGearingRatioSensorToMechanism = 125;

  public static final double kFlywheelSpeedToleranceRps = 2.0;

  /** The frequency that telemetry form the motor is pushed to the CANBus */
  public static final double kStatusSignalUpdateFrequencyHz = 100.0;

  public static final int kLinearFilterSampleCount = 5;
  public static final int kAmpFilterThreshold = 40;
  public static final boolean kHomeWithCurrent = false;

  public static final GravityTypeValue gravityType = GravityTypeValue.Arm_Cosine;

  public static final double kSimGearing = 9.0 / 1.0; // gearing used in simulator ONLY

  public record ClimberHardware(
       int motorID, double rotorRotationsToMeters) {}

  public record ClimberGains(
      double p,
      double i,
      double d,
      double v,
      double s,
      double g,
      double a,
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSquared,
      double jerkMetersPerSecondCubed) {}

  public record ClimberMotorConfiguration(
      boolean invert,
      boolean enableStatorCurrentLimit,
      boolean enableSupplyCurrentLimit,
      double statorCurrentLimitAmps,
      double supplyCurrentLimitAmps,
      double peakForwardVoltage,
      double peakReverseVoltage,
      // double maxPositionRotations,
      NeutralModeValue neutralMode) {}
      // TODO: add min/max soft limits

  public record SimulationConfiguration(
      DCMotor motorType,
      double carriageMassKg,
      double drumRadiusMeters,
      boolean simulateGravity,
      double startingHeightMeters,
      double measurementStdDevs) {}

  public static final ClimberHardware kClimberHardware =
      new ClimberHardware(
          34,
          kRotorRotationsToMeters); // Drum (sprocket) circumference

  public static final ClimberGains kClimberGains =
      new ClimberGains(
          55, 0.0, 0, 0, // 2.947
          0, // 22
          0.4, 0, 3.7, 1.2, 0); // 0.11

  public static final ClimberMotorConfiguration kMotorConfiguration =
      new ClimberMotorConfiguration(
          false, true, true, 80.0, 50.0, 12.0, -12.0, NeutralModeValue.Brake);

  public static final SimulationConfiguration kSimulationConfiguration =
      new SimulationConfiguration(
          DCMotor.getKrakenX60(1),
          // empty carriage load = .8kg
          // prototype carriage load = 13.61 kg
          5.0,
          kRotorRotationsToMeters,
          true,
          0.0,
          0.0002);
}
