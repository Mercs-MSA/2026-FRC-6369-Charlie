// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  /** List of position setpoints for the Climber in meters */
  public enum ClimberGoal {
  kDown(() -> 0.0),
  
  kClimbed(() -> 0.05);
    /** Custom setpoint that can be modified over network tables; Usefu for debugging */
    private DoubleSupplier goal;

    ClimberGoal(DoubleSupplier goalMeters) {
      this.goal = goalMeters;
    }

    public double getGoalRadians() {
      return this.goal.getAsDouble();
    }
  }


  private final ClimberIO kClimber;
  private final ClimberIOInputsAutoLogged kInputsClimber = new ClimberIOInputsAutoLogged();


  private ClimberGoal currentClimberGoal = ClimberGoal.kDown;

  public Climber(ClimberIO io) {
    kClimber = io;
  }

  @Override
  public void periodic() {
    kClimber.updateInputs(kInputsClimber);
    Logger.recordOutput("Climber/PositionGoal", currentClimberGoal);
    Logger.recordOutput("Climber/Actual", getPosition());
    Logger.recordOutput("Climber/AtGoal", positionAtGoal());
    Logger.processInputs("Climber/Inputs", kInputsClimber);


    if (currentClimberGoal != null) {
      kClimber.setPosition(currentClimberGoal.getGoalRadians());

      Logger.processInputs("Climber/Inputs", kInputsClimber);
    } else {
      Logger.recordOutput("Climber/PositionGoal", "NONE");
    }

  }

  /**
   * Sets the position goal of the mechanism, logic runs in subsystem periodic method
   *
   * @param desiredGoal The desired position goal
   */
  public void setClimberGoal(ClimberGoal desiredGoal) {
    currentClimberGoal = desiredGoal;
  }

  /** Stops the mechanism */
  public void stop() {
    currentClimberGoal = null;
    kClimber.stop();
  }

  /** Reset the mechanism's encoder to 0 */
  public void resetPosition() {
    kClimber.resetPosition();
  }

  public ClimberGoal getClimberGoal() {
    return currentClimberGoal;
  }


  /**
   * Compute the error based off of our current position and current goal
   *
   * @return The computed error in radians
   */
  @AutoLogOutput(key = "Climber/Feedback/PositionError")
  public double getPositionError() {
    return currentClimberGoal.getGoalRadians() - Units.rotationsToRadians(getPosition());
  }

  /**
   * Compute the error based off of our current speed and current goal
   *
   * @return The computed error in rps
   */
  @AutoLogOutput(key = "Climber/Feedback/FlywheelError")
  public double getFlywheelError() {
    return currentClimberGoal.getGoalRadians() - getPosition();
  }

  /**
   * @return If the Climber is at its desired goal yet
   */
  @AutoLogOutput(key = "Climber/Feedback/PositionAtGoal")
  public boolean positionAtGoal() {
    return Math.abs(getPositionError()) < ClimberConstants.kPositionToleranceRadians;
  }

  /**
   * @return If the Intake flywheels are at its desired goal yet
   */


  /**
   * @return The position of the linear mechanism in radians
   */
  public double getPosition() {
    return kClimber.getPosition();
  }
}

