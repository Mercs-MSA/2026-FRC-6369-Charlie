// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.TeleopStates.TeleopMode;
import frc.robot.generated.TunerConstants;
import frc.robot.math.ShooterMathProvider;
import frc.robot.math.ShooterMathProvider.CalculationState;
import frc.robot.subsystems.drive.Drive.Controllers.HolonomicController;
import frc.robot.subsystems.drive.Drive.Drive;
import frc.robot.subsystems.drive.Drive.Drive.DriveState;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel.FlywheelState;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.IndexConstants;
import frc.robot.subsystems.index.IndexIOTalonFX;
import frc.robot.subsystems.index.Index.IndexState;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.turret.Turret.TurretGoalState;
import frc.robot.subsystems.drive.Drive.GyroIO;
import frc.robot.subsystems.drive.Drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.Drive.ModuleIO;
import frc.robot.subsystems.drive.Drive.ModuleIOSim;
import frc.robot.subsystems.drive.Drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.Intake.IntakeFlywheelGoal;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import frc.robot.subsystems.shooterhood.Hood;
import frc.robot.subsystems.shooterhood.HoodConstants;
import frc.robot.subsystems.shooterhood.HoodIOTalonFX;
import frc.robot.subsystems.shooterhood.Hood.HoodGoal;
import frc.robot.subsystems.shooterhood.Hood.HoodState;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeFlywheelIOTalonFX;
import frc.robot.subsystems.intake.IntakeFlywheelConstants;
import frc.robot.subsystems.spindexer.Spindexer;
import frc.robot.subsystems.spindexer.SpindexerConstants;
import frc.robot.subsystems.spindexer.SpindexerIOTalonFX;
import frc.robot.subsystems.spindexer.Spindexer.SpindexerState;

import java.util.HashMap;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // controllers
  public final HolonomicController driveToPoseController = new HolonomicController();
  
  // Math
  public final ShooterMathProvider shooterMath = new ShooterMathProvider();

  // Subsystems

  public final Drive drive;
  public final Vision vision;

  public final Flywheel shooterFlywheels;
  public final Hood shooterHood;
  public final Turret shooterTurret;
  public final Intake intake;
  public final Spindexer spindexer;

  public final Index index;
  public final Trigger flywheelsAtGoalTrigger;
  public final Trigger intakeTrigger;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController testController = new CommandXboxController(2);
  
  // Commands
  public final TeleopStates teleopState;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // Vision subsystem
        vision =
            new Vision(
                new CameraIO[] {
                  new VisionIOLimelight(VisionConstants.camera0Name, () -> drive.getRotation()),
                  new VisionIOLimelight(VisionConstants.camera1Name, () -> drive.getRotation()),
                  new VisionIOLimelight(VisionConstants.camera2Name, () -> drive.getRotation()),
                  new VisionIOLimelight(VisionConstants.camera3Name, () -> drive.getRotation())
                });
 
        // Shooter
        shooterFlywheels =
                new Flywheel(
                  new FlywheelIOTalonFX(FlywheelConstants.kFlywheelHardware, FlywheelConstants.kMotorConfiguration, FlywheelConstants.kFlywheelGains), shooterMath
                );

        shooterHood = 
                new Hood(
                  new HoodIOTalonFX(HoodConstants.kHoodHardware, HoodConstants.kMotorConfiguration, HoodConstants.kHoodGains, HoodConstants.kMinRadians, HoodConstants.kMaxRadians), shooterMath);

        shooterTurret =
                new Turret(
                  new TurretIOTalonFX(TurretConstants.kTurretHardware, TurretConstants.kMotorConfiguration, TurretConstants.kTurretGains, TurretConstants.kMinRadiansLimit, TurretConstants.kMaxRadiansLimit), drive, shooterMath);

        intake =
                new Intake(
                  new IntakeIOTalonFX(IntakeConstants.kIntakeHardware, IntakeConstants.kMotorConfiguration, IntakeConstants.kIntakeGains),
                  new IntakeFlywheelIOTalonFX(IntakeFlywheelConstants.kFlywheelHardware, IntakeFlywheelConstants.kMotorConfiguration, IntakeFlywheelConstants.kFlywheelGains));

        index = 
                new Index(
                  new IndexIOTalonFX(IndexConstants.kIndexHardware, IndexConstants.kMotorConfiguration, IndexConstants.kIndexGains), shooterMath
                );

        spindexer = new Spindexer(
          new SpindexerIOTalonFX(SpindexerConstants.kIndexHardware, SpindexerConstants.kMotorConfiguration, SpindexerConstants.kSpindexIndexGains)
        );
        
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        // Vision subsystem
        vision =
            new Vision(
                new CameraIO[] {
                  new VisionIOLimelight(VisionConstants.camera0Name, () -> drive.getRotation()),
                  new VisionIOLimelight(VisionConstants.camera1Name, () -> drive.getRotation()),
                  new VisionIOLimelight(VisionConstants.camera2Name, () -> drive.getRotation()),
                  new VisionIOLimelight(VisionConstants.camera3Name, () -> drive.getRotation())
                });

        // Shooter
        shooterFlywheels =
                new Flywheel(
                  new FlywheelIOTalonFX(FlywheelConstants.kFlywheelHardware, FlywheelConstants.kMotorConfiguration, FlywheelConstants.kFlywheelGains), shooterMath
                );

        shooterHood = 
                new Hood(
                  new HoodIOTalonFX(HoodConstants.kHoodHardware, HoodConstants.kMotorConfiguration, HoodConstants.kHoodGains, HoodConstants.kMinRadians, HoodConstants.kMaxRadians), shooterMath);

        shooterTurret =
                new Turret(
                  new TurretIOTalonFX(TurretConstants.kTurretHardware, TurretConstants.kMotorConfiguration, TurretConstants.kTurretGains, TurretConstants.kMinRadiansLimit, TurretConstants.kMaxRadiansLimit), drive, shooterMath);

        intake =
                new Intake(
                  new IntakeIOTalonFX(IntakeConstants.kIntakeHardware, IntakeConstants.kMotorConfiguration, IntakeConstants.kIntakeGains),
                  new IntakeFlywheelIOTalonFX(IntakeFlywheelConstants.kFlywheelHardware, IntakeFlywheelConstants.kMotorConfiguration, IntakeFlywheelConstants.kFlywheelGains));
        
        index = 
                new Index(
                  new IndexIOTalonFX(IndexConstants.kIndexHardware, IndexConstants.kMotorConfiguration, IndexConstants.kIndexGains), shooterMath
                );

        spindexer = new Spindexer(
          new SpindexerIOTalonFX(SpindexerConstants.kIndexHardware, SpindexerConstants.kMotorConfiguration, SpindexerConstants.kSpindexIndexGains)
        );

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        // Vision subsystem
        vision =
            new Vision(
                new CameraIO[] {
                  new VisionIOLimelight(VisionConstants.camera0Name, () -> drive.getRotation()),
                  new VisionIOLimelight(VisionConstants.camera1Name, () -> drive.getRotation()),
                  new VisionIOLimelight(VisionConstants.camera2Name, () -> drive.getRotation()),
                  new VisionIOLimelight(VisionConstants.camera3Name, () -> drive.getRotation())
                });

     
        // Shooter
        shooterFlywheels =
                new Flywheel(
                  new FlywheelIOTalonFX(FlywheelConstants.kFlywheelHardware, FlywheelConstants.kMotorConfiguration, FlywheelConstants.kFlywheelGains), shooterMath
                );

        shooterHood = 
                new Hood(
                  new HoodIOTalonFX(HoodConstants.kHoodHardware, HoodConstants.kMotorConfiguration, HoodConstants.kHoodGains, HoodConstants.kMinRadians, HoodConstants.kMaxRadians), shooterMath);

        shooterTurret =
                new Turret(
                  new TurretIOTalonFX(TurretConstants.kTurretHardware, TurretConstants.kMotorConfiguration, TurretConstants.kTurretGains, TurretConstants.kMinRadiansLimit, TurretConstants.kMaxRadiansLimit), drive, shooterMath);

        intake =
                new Intake(
                  new IntakeIOTalonFX(IntakeConstants.kIntakeHardware, IntakeConstants.kMotorConfiguration, IntakeConstants.kIntakeGains),
                  new IntakeFlywheelIOTalonFX(IntakeFlywheelConstants.kFlywheelHardware, IntakeFlywheelConstants.kMotorConfiguration, IntakeFlywheelConstants.kFlywheelGains));

        index = 
                new Index(
                  new IndexIOTalonFX(IndexConstants.kIndexHardware, IndexConstants.kMotorConfiguration, IndexConstants.kIndexGains), shooterMath
                );
                
        spindexer = new Spindexer(
          new SpindexerIOTalonFX(SpindexerConstants.kIndexHardware, SpindexerConstants.kMotorConfiguration, SpindexerConstants.kSpindexIndexGains)
        );

        break;
    }

    flywheelsAtGoalTrigger = new Trigger(() -> shooterFlywheels.atSpeed());
    intakeTrigger = new Trigger(() -> intake.positionAtGoal());
    teleopState = new TeleopStates(drive, intake, shooterFlywheels, shooterHood, shooterTurret, spindexer, index);

    // Create auto routines
    NamedCommands.registerCommands(new HashMap<String, Command>(){
      {
        put("Start", Commands.runOnce(() -> {
          System.out.println("start");
          shooterFlywheels.setFlywheelState(FlywheelState.STOP);
          shooterHood.setHoodState(HoodState.PROVIDED);
          shooterHood.setGoal(HoodGoal.PROVIDED);
          shooterTurret.setTurretState(TurretGoalState.PROVIDED);
          index.setIndexState(IndexState.STOP);
          spindexer.setIndexState(SpindexerState.STOP);
          intake.setIntakeGoal(IntakeGoal.kStow);
          intake.setFlywheelGoal(IntakeFlywheelGoal.kStop);
        }, shooterFlywheels, shooterHood, shooterTurret, index, spindexer, intake));
        put("StartFlywheels", Commands.runOnce(() -> {
          shooterFlywheels.setFlywheelState(FlywheelState.PROVIDED);
        }, shooterFlywheels, shooterHood, shooterTurret, index, spindexer));
        put("BeginIndex", Commands.runOnce(() -> {
          index.setIndexState(IndexState.PROVIDED);
          spindexer.setIndexState(SpindexerState.RUNNING);
        }, index, spindexer));
        put("IntakeOut", Commands.runOnce(() -> {
          intake.setIntakeGoal(IntakeGoal.kOut);
        }, intake));
        put("IntakeHalf", Commands.runOnce(() -> {
          intake.setIntakeGoal(IntakeGoal.kAutoTravel);
        }, intake));
        put("IntakeUp", Commands.runOnce(() -> {
          intake.setIntakeGoal(IntakeGoal.kStow);
        }, intake));
        put("IntakeRun", Commands.runOnce(() -> {
          intake.setFlywheelGoal(IntakeFlywheelGoal.kRunning);
        }, intake));
        put("IntakeSlow", Commands.runOnce(() -> {
          intake.setFlywheelGoal(IntakeFlywheelGoal.kSlow);
        }, intake));
        put("IntakeStop", Commands.runOnce(() -> {
          intake.setFlywheelGoal(IntakeFlywheelGoal.kStop);
        }, intake));
        put("StartAgitate", Commands.runOnce(() -> {
          intake.setAgitating(true);
        }, intake));
      }
    });

    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

    // // Set up SysId routines
    // pathplannerAutoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // pathplannerAutoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // pathplannerAutoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // pathplannerAutoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // pathplannerAutoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // pathplannerAutoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    drive.acceptJoystickInputs(
        () ->
            -Math.copySign(
                driverController.getLeftY() * driverController.getLeftY(),
                driverController.getLeftY()),
        () ->
            -Math.copySign(
                driverController.getLeftX() * driverController.getLeftX(),
                driverController.getLeftX()),
        () -> -driverController.getRightX());

    // Configure the button bindings
    configureButtonBindings();
  }

  public Command getTeleopCommand() {
    return new SequentialCommandGroup(drive.setDriveStateCommand(Drive.DriveState.TELEOP));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


    drive.setDefaultCommand(
        Commands.run(
            () ->
                drive.acceptJoystickInputs(
                    () ->
                        -Math.copySign(
                            driverController.getLeftY() * driverController.getLeftY(),
                            driverController.getLeftY()),
                    () ->
                        -Math.copySign(
                            driverController.getLeftX() * driverController.getLeftX(),
                            driverController.getLeftX()),
                    () -> -driverController.getRightX()),
            drive));

    driverController.pov(0).whileTrue(
      Commands.runEnd(() -> {drive.acceptJoystickInputs(()->0.5, ()->0.0, ()->0.0);}, () -> {drive.acceptJoystickInputs(()->0.0, ()->0.0, ()->0.0);})
    );

    driverController.pov(90).whileTrue(
      Commands.runEnd(() -> {drive.acceptJoystickInputs(()->0.0, ()->-0.5, ()->0.0);}, () -> {drive.acceptJoystickInputs(()->0.0, ()->0.0, ()->0.0);})
    );

    driverController.pov(270).whileTrue(
      Commands.runEnd(() -> {drive.acceptJoystickInputs(()->0.0, ()->0.5, ()->0.0);}, () -> {drive.acceptJoystickInputs(()->0.0, ()->0.0, ()->0.0);})
    );
    
    driverController.pov(180).whileTrue(
      Commands.runEnd(() -> {drive.acceptJoystickInputs(()->-0.5, ()->0.5, ()->0.0);}, () -> {drive.acceptJoystickInputs(()->0.0, ()->0.0, ()->0.0);})
    );

    // prepare for climb
    driverController
        .back()
        .onTrue(drive.setDriveStateCommandContinued(DriveState.DRIVETOCLIMB))
        .onFalse(drive.setDriveStateCommand(DriveState.TELEOP));

    // driverController
    //     .x()
    //     .onTrue(drive.setDriveStateCommandContinued(DriveState.DRIVETOPOSEPROFILED))
    //     .onFalse(drive.setDriveStateCommandContinued(DriveState.TELEOP));

    testController.leftBumper().onTrue(Commands.runOnce(() -> {SignalLogger.start();}));
    testController.rightBumper().onTrue(Commands.runOnce(() -> {SignalLogger.stop();}));
    testController.a().onTrue(Commands.runOnce(() -> {drive.setDriveState(DriveState.AUTO);})).whileTrue(
        drive.sysIdQuasistatic(Direction.kForward)).onFalse(Commands.runOnce(() -> {drive.setDriveState(DriveState.TELEOP);}));
    testController.b().onTrue(Commands.runOnce(() -> {drive.setDriveState(DriveState.AUTO);})).whileTrue(
        drive.sysIdQuasistatic(Direction.kReverse)).onFalse(Commands.runOnce(() -> {drive.setDriveState(DriveState.TELEOP);}));
    testController.x().onTrue(Commands.runOnce(() -> {drive.setDriveState(DriveState.AUTO);})).whileTrue(
      drive.sysIdDynamic(Direction.kForward)).onFalse(Commands.runOnce(() -> {drive.setDriveState(DriveState.TELEOP);}));
    testController.y().onTrue(Commands.runOnce(() -> {drive.setDriveState(DriveState.AUTO);})).whileTrue(
      drive.sysIdDynamic(Direction.kReverse)).onFalse(Commands.runOnce(() -> {drive.setDriveState(DriveState.TELEOP);}));

    // Reset gyro to 0° when B button is pressed
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // driverController
    //     .leftBumper()
    //     .onTrue(
    //         Commands.runOnce(
    //             () -> {
    //               if (drive.getDriveState() == DriveState.POINTANDDRIVE) {
    //                 drive.setDriveStateCommandContinued(DriveState.TELEOP).schedule();
    //               } else {
    //                 drive.setDriveStateCommandContinued(DriveState.POINTANDDRIVE).schedule();
    //               }
    //             }));
    
    // Reset everything to stowed position
    operatorController.a().onTrue(Commands.runOnce(() -> {
      teleopState.homeMode();
    }, intake));

    // intake mode
    operatorController.leftBumper().onTrue(Commands.runOnce(() -> {
      if (teleopState.currentTeleopMode == TeleopMode.INTAKE_WARMUP || teleopState.currentTeleopMode == TeleopMode.INTAKE_ACTIVE
      ) {
        teleopState.idleMode();
      } else {
        teleopState.warmupIntakeMode();
      }
    }, intake));
    
    // driverController.y().onTrue(Commands.runOnce(()-> {
    //   teleopState.currentTeleopMode=TeleopMode.
    // }
    
    // Shooting Mode
    driverController.rightBumper().onTrue(Commands.runOnce(() -> {
      shooterMath.setState(CalculationState.HUB);
      if (teleopState.currentTeleopMode == TeleopMode.SHOOT_ACTIVE || teleopState.currentTeleopMode == TeleopMode.SHOOT_ACTIVE) {
        teleopState.idleMode();
      } else {
        teleopState.warmupShootMode();
      }
    }, intake));
    driverController.leftBumper().onTrue(Commands.runOnce(() -> {
      shooterMath.setState(CalculationState.SHUNT);
      if (teleopState.currentTeleopMode == TeleopMode.SHOOT_ACTIVE || teleopState.currentTeleopMode == TeleopMode.SHOOT_ACTIVE) {
        teleopState.idleMode();
      } else {
        teleopState.warmupShootMode();
      }
    }, intake));

    // idle mode
    operatorController.x().onTrue(Commands.runOnce(() -> {
      teleopState.idleMode();
    }, intake));

    // half mode
    operatorController.y().onTrue(Commands.runOnce(() -> {
      if (teleopState.currentTeleopMode == TeleopMode.HALF) {
        teleopState.idleMode();
      } else { 
      teleopState.halfMode();
      }
    }, intake));

    operatorController.b().onTrue(Commands.runOnce(() -> {
      intake.setAgitating(true);
    })).onFalse(Commands.runOnce(() -> {
      intake.setAgitating(false);
    }));

    operatorController.rightTrigger().onTrue(Commands.runOnce(() -> {
      intake.setPushback(true);
    })).onFalse(Commands.runOnce(() -> {
      intake.setPushback(false);
    }));


    intakeTrigger.onTrue(Commands.runOnce(()->
      {
        if(teleopState.currentTeleopMode == TeleopMode.INTAKE_WARMUP)
        {
          teleopState.intakeActive();
        }
      }));

    flywheelsAtGoalTrigger.onTrue(Commands.runOnce(() -> {
    if (teleopState.currentTeleopMode == TeleopMode.SHOOT_WARMUP) {
      teleopState.shootActive();
    }
  }));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
