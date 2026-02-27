// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.flywheel.Flywheel.FlywheelState;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** List of position setpoints for the Intake in meters */
  public enum IntakeGoal {
    kOut(() -> 2.43, 2.3, 2.43),
    kHalf(() -> 1.5, 1.2, 1.8),
    kAutoTravel(() -> 1.8, 1.8, 1.8),
    kStow(() -> 0.005, 0.005, 0.005),
    kPushback(() -> 0.5, 0.5, 0.5);
    /** Custom setpoint that can be modified over network tables; Usefu for debugging */
    private DoubleSupplier goal;
    private double goalAgitateMax;
    private double goalAgitateMin;

    IntakeGoal(DoubleSupplier goalMeters, double goalAgitateMax, double goalAgitateMin) {
      this.goal = goalMeters;
      this.goalAgitateMax = goalAgitateMax;
      this.goalAgitateMin = goalAgitateMin;
    }

    public double getGoalRadians() {
      return this.goal.getAsDouble();
    }

    public double getGoalAgitateMax() {
      return this.goalAgitateMax;
    }

    public double getGoalAgitateMin() {
      return this.goalAgitateMin;
    }
  }

  public enum IntakeFlywheelGoal {
    kStop(() -> 0),
    kSlow(() -> 2.5),
    kRunning(() -> 8);

    private DoubleSupplier goalRps;

    IntakeFlywheelGoal(DoubleSupplier goalRps) {
      this.goalRps = goalRps;
    }

    public double getGoalRps() {
      return this.goalRps.getAsDouble();
    }
  }

  private final IntakeIO kIntake;
  private final IntakeIOInputsAutoLogged kInputsIntake = new IntakeIOInputsAutoLogged();

  private final IntakeFlywheelIO kIntakeFlywheel;
  private final IntakeFlywheelIOInputsAutoLogged kInputsIntakeFlywheel = new IntakeFlywheelIOInputsAutoLogged();

  private IntakeGoal currentIntakeGoal = IntakeGoal.kStow;
  private IntakeFlywheelGoal currentFlywheelGoal = IntakeFlywheelGoal.kStop;

  private boolean isAgitating = false;
  private boolean isPushback = false;

  public Intake(IntakeIO io, IntakeFlywheelIO flywheelIO) {
    kIntake = io;
    kIntakeFlywheel = flywheelIO;
  }

  @Override
  public void periodic() {
    kIntake.updateInputs(kInputsIntake);
    kIntakeFlywheel.updateInputs(kInputsIntakeFlywheel);
    Logger.recordOutput("Intake/PositionGoal", currentIntakeGoal);
    Logger.recordOutput("Intake/FlywheelGoal", currentFlywheelGoal);
    Logger.recordOutput("Intake/Actual", getPosition());
    Logger.recordOutput("Intake/AtGoal", positionAtGoal());
    Logger.processInputs("Intake/Inputs", kInputsIntake);


    if (currentIntakeGoal != null) {
      if (isPushback) {
        kIntake.setPosition(IntakeGoal.kPushback.goal.getAsDouble(), true);
      } else if (isAgitating) {
        double agitateRange = currentIntakeGoal.getGoalAgitateMax() - currentIntakeGoal.getGoalAgitateMin();
        double agitateOffset = agitateRange / 2 * Math.sin(2 * Math.PI * 4 * System.currentTimeMillis() / 1000);
        kIntake.setPosition(currentIntakeGoal.getGoalRadians() + agitateOffset, false);
      } else {
        kIntake.setPosition(currentIntakeGoal.getGoalRadians(), false);
      }

      Logger.processInputs("Intake/Inputs", kInputsIntake);
    } else {
      Logger.recordOutput("Intake/PositionGoal", "NONE");
    }

    if (currentFlywheelGoal != null) {
      if (currentFlywheelGoal == IntakeFlywheelGoal.kStop) {
        kIntakeFlywheel.stop();
      } else {
        kIntakeFlywheel.setVelocity(currentFlywheelGoal.getGoalRps());
      }

      Logger.processInputs("Intake/FlywheelInputs", kInputsIntake);
    } else {
      Logger.recordOutput("Intake/FlywheelGoal", "NONE");
    }
  }

  /**
   * Sets the position goal of the mechanism, logic runs in subsystem periodic method
   *
   * @param desiredGoal The desired position goal
   */
  public void setIntakeGoal(IntakeGoal desiredGoal) {
    currentIntakeGoal = desiredGoal;
  }

  public void setFlywheelGoal(IntakeFlywheelGoal desiredGoal) {
    currentFlywheelGoal = desiredGoal;
  }

  /** Stops the mechanism */
  public void stop() {
    currentIntakeGoal = null;
    kIntake.stop();
  }

  public void setAgitating(boolean isAgitating) {
    this.isAgitating = isAgitating;
  }

  public void setPushback(boolean isPushback) {
    this.isPushback = isPushback;
  }

  /** Reset the mechanism's encoder to 0 */
  public void resetPosition() {
    kIntake.resetPosition();
  }

  public IntakeGoal getGoal() {
    return currentIntakeGoal;
  }


  /**
   * Compute the error based off of our current position and current goal
   *
   * @return The computed error in radians
   */
  @AutoLogOutput(key = "Intake/Feedback/PositionError")
  public double getPositionError() {
    return currentIntakeGoal.getGoalRadians() - Units.rotationsToRadians(getPosition());
  }

  /**
   * Compute the error based off of our current speed and current goal
   *
   * @return The computed error in rps
   */
  @AutoLogOutput(key = "Intake/Feedback/FlywheelError")
  public double getFlywheelError() {
    return currentIntakeGoal.getGoalRadians() - getPosition();
  }

  /**
   * @return If the Intake is at its desired goal yet
   */
  @AutoLogOutput(key = "Intake/Feedback/PositionAtGoal")
  public boolean positionAtGoal() {
    return Math.abs(getPositionError()) < IntakeConstants.kPositionToleranceRadians;
  }

  /**
   * @return If the Intake flywheels are at its desired goal yet
   */
  @AutoLogOutput(key = "Intake/Feedback/FlywheelsAtGoal")
  public boolean flywheelAtGoal() {
    return Math.abs(getFlywheelError()) < IntakeConstants.kFlywheelSpeedToleranceRps;
  }

  /**
   * @return The position of the linear mechanism in radians
   */
  public double getPosition() {
    return kIntake.getPosition();
  }
}
