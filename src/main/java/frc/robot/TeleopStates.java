package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import frc.robot.math.ShooterMathProvider;
import frc.robot.subsystems.drive.Drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.Flywheel.FlywheelState;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.Index.IndexState;
import frc.robot.subsystems.intake.Intake.IntakeFlywheelGoal;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import frc.robot.subsystems.shooterhood.Hood;
import frc.robot.subsystems.shooterhood.Hood.HoodGoal;
import frc.robot.subsystems.shooterhood.Hood.HoodState;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.Spindexer.SpindexerState;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretGoalState;
import frc.robot.subsystems.intake.Intake;

public class TeleopStates {
    public final Drive drive;
    public final Intake intake;
    public final Flywheel shooterFlywheels;
    public final Hood shooterHood;
    public final Turret shooterTurret;
    public final Spindexer spindexer;
    public final Index index;

    public TeleopStates(Drive drive, Intake intake, Flywheel shooterFlywheels, Hood shooterHood, Turret shooterTurret, Spindexer spindexer, Index index) {
        this.drive = drive;
        this.intake = intake;
        this.shooterFlywheels = shooterFlywheels;
        this.shooterHood = shooterHood;
        this.shooterTurret = shooterTurret;
        this.spindexer = spindexer;
        this.index = index;
    }

    public void warmupShootMode() {
      if (intake.currentIntakeGoal != IntakeGoal.kHalf && intake.currentIntakeGoal != IntakeGoal.kOut) {
        intake.setIntakeGoal(IntakeGoal.kOut);
      }
      shooterFlywheels.setFlywheelState(FlywheelState.PROVIDED);
      shooterTurret.setTurretState(TurretGoalState.PROVIDED);
      shooterHood.setGoal(HoodGoal.PROVIDED);
      shooterHood.setHoodState(HoodState.PROVIDED);

      if (shooterFlywheels.atSpeed()) {
        shootActive();
      }
    }

    public void intakeHalfMode() {
      intake.setIntakeGoal(IntakeGoal.kHalf);
      intake.setFlywheelGoal(IntakeFlywheelGoal.kSlow);      
    }

    public void intakeMode() {
      intake.setIntakeGoal(IntakeGoal.kOut);
      intake.setFlywheelGoal(IntakeFlywheelGoal.kRunning);      
    }

    public void shootActive() {
      index.setIndexState(IndexState.PROVIDED);
      spindexer.setIndexState(SpindexerState.RUNNING);
    }
    
    public void intakeStop() {
      intake.setFlywheelGoal(IntakeFlywheelGoal.kStop);
    }

    public void shootStop() {
      shooterFlywheels.setFlywheelState(FlywheelState.STOP);
      shooterTurret.setTurretState(TurretGoalState.PROVIDED);
      shooterHood.setGoal(HoodGoal.PROVIDED);
      shooterHood.setHoodState(HoodState.PROVIDED);
      index.setIndexState(IndexState.STOP);
      spindexer.setIndexState(SpindexerState.STOP);
    }

    public void halfMode() {
      intake.setIntakeGoal(IntakeGoal.kHalf);
      intake.setFlywheelGoal(IntakeFlywheelGoal.kSlow);
    }

    public void homeMode() {
      intake.setIntakeGoal(IntakeGoal.kStow);
      intake.setFlywheelGoal(IntakeFlywheelGoal.kStop);
      shooterFlywheels.setFlywheelState(FlywheelState.STOP);
      shooterTurret.setTurretState(TurretGoalState.HOME);
      index.setIndexState(IndexState.STOP);
      spindexer.setIndexState(SpindexerState.STOP);
      shooterHood.setGoal(HoodGoal.STOW);
      shooterHood.setHoodState(HoodState.STOW);
    }
}
