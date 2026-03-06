package frc.robot.subsystems.shooterhood;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface HoodIO {

  public class HoodIOInputs {
    public boolean isMotorConnected = false;

    public double positionDegrees = 0.0;
    public double velocityDegreesPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  public void updateInputs(HoodIOInputs inputs);

  public void setPosition(double rotations);

  public void setPositionMM(double positionDegrees);
  
  public StatusSignal<Angle> getPosition();

  public StatusSignal<AngularVelocity> getVelocity();

  public void stop();

  /**
   * @param p Proportional
   * @param i Integral
   * @param d Derivative gain
   * @param v Velocity rot/s
   * @param s Static volts to hold up
   * @param g Gravity term
   * @param a Acceleration
   */
  public void setGains(double p, double i, double d, double v, double s, double g, double a);

  public void setMotionMagicConstraints(double maxVelocity, double maxAcceleration);

  public void setBrakeMode(boolean brake);
}
