package frc.robot.subsystems.shooterhood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.math.ShooterMathProvider;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

  public enum HoodGoal {
    STOW(() -> 0.00),
    PROVIDED(() -> 0.0);

    private final DoubleSupplier goalDegrees;

    HoodGoal(DoubleSupplier goalDegrees) {
      this.goalDegrees = goalDegrees;
    }

    public double getGoalRadians() {
      return goalDegrees.getAsDouble();
    }
  }

  public enum HoodState {
    STOW,
    PROVIDED
  }

  public HoodState currentState = HoodState.STOW;

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private HoodGoal currentGoal = null;
  private double goalRotations = 0.0;

  private Translation2d targetPoint = new Translation2d();

  private final ShooterMathProvider math;

  public Hood(HoodIO io, ShooterMathProvider math) {
    this.io = io;
    this.math = math;

    //Take  in math, assume the values in there are updated

    io.setGains(
        HoodConstants.kHoodGains.p(),
        HoodConstants.kHoodGains.i(),
        HoodConstants.kHoodGains.d(),
        HoodConstants.kHoodGains.s(),
        HoodConstants.kHoodGains.g(),
        HoodConstants.kHoodGains.v(),
        HoodConstants.kHoodGains.a());

    io.setMotionMagicConstraints(
        HoodConstants.kHoodGains.maxVelocityDegPerSec(),
        HoodConstants.kHoodGains.maxAccelerationDegPerSec2());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood/Inputs", inputs);

    if (DriverStation.isDisabled()) {
      stop();
      return;
    }

    if (currentGoal != null) {
      if (currentState != HoodState.PROVIDED) {
        goalRotations = currentGoal.getGoalRadians();
      } else {
        goalRotations = math.shooterHoodAngle;
      }
    }
    // io.setPosition(math.hoodStow ? HoodGoal.STOW.getGoalRadians() : goalRotations);
    io.setPosition(goalRotations);
}

  public void setGoal(HoodGoal goal) {
    this.currentGoal = goal;
  }

  @AutoLogOutput(key = "Hood/targetPoint")
  public Translation2d getTargetPoint() {
    return targetPoint;
  }

  public void setAngle(double angleDeg) {
    goalRotations = angleDeg;
    currentState = HoodState.PROVIDED;
    setPositionRad(angleDeg);
  }

  public void stop() {
    currentGoal = null;
    io.stop();
  }

  public void setPositionRad(double angle) {
    io.setPosition(angle);
  }

  public void setHoodGoalWithState() {
    switch (currentState) {
      case STOW -> setGoal(HoodGoal.STOW);
      case PROVIDED -> setGoal(HoodGoal.PROVIDED);
    }
  }

  @AutoLogOutput(key = "Hood/State")
  public HoodState getHoodState() {
    return currentState;
  }

  @AutoLogOutput(key = "Hood/GoalDegrees")
  public double getSimGoalDeg() {
    return goalRotations;
  }

  public void setHoodState(HoodState state) {
    this.currentState = state;
  }

  // @AutoLogOutput(key = "Pivot/AtGoal")
  // public boolean atGoal() {
  //   return Math.abs(goalAngleRad - getAngleDeg()) < PivotConstants.kPositionToleranceRad;
  // }

  @AutoLogOutput(key = "Hood/AngleDeg")
  public double getAngleDeg() {
    return inputs.positionDegrees;
  }

  @AutoLogOutput(key = "Hood/GoalDeg")
  public double getGoalDeg() {
    return goalRotations;
  }
}
