// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.InhaleCommand;
import frc.robot.commands.ShootFuelCommandAuto;
import frc.robot.commands.SuperShooterCommand;
import frc.robot.commands.UnjamCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.climber.ClimberConstants.ClimberModes;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.Carpet;
import frc.robot.subsystems.hopper.CarpetIO;
import frc.robot.subsystems.hopper.CarpetIOSim;
import frc.robot.subsystems.hopper.CarpetIOTalonFX;
import frc.robot.subsystems.hopper.CarpetConstants.CarpetModes;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;
import frc.robot.subsystems.intake.pivot.PivotIO;
import frc.robot.subsystems.intake.pivot.PivotIOSim;
import frc.robot.subsystems.intake.pivot.PivotIOTalonFX;
import frc.robot.subsystems.intake.roller.RollerIO;
import frc.robot.subsystems.intake.roller.RollerIOSim;
import frc.robot.subsystems.intake.roller.RollerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterModes;
import frc.robot.subsystems.shooter.feeder.FeederIO;
import frc.robot.subsystems.shooter.feeder.FeederIOSim;
import frc.robot.subsystems.shooter.feeder.FeederIOTalonFX;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.AllianceFlipUtil;
import frc.team5431.titan.core.joysticks.CommandXboxController;

import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  // private final Vision vision;
  private final Intake intake;
  private final Shooter shooter;
  private final Carpet carpet;
  private final Climber climber;
  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive = new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));

        // Real robot, instantiate hardware IO implementations
        // vision =
        // new Vision(
        // drive::addVisionMeasurement,
        // new VisionIOLimelight(camera0Name, drive::getRotation),
        // new VisionIOLimelight(camera1Name, drive::getRotation));

        intake = new Intake(new RollerIOTalonFX(), new PivotIOTalonFX());
        shooter = new Shooter(new FeederIOTalonFX(), new FlywheelIOTalonFX());
        carpet = new Carpet(new CarpetIOTalonFX());
        climber = new Climber(new ClimberIOSim());
        vision = new Vision(drive::addVisionMeasurement,
            // new VisionIOLimelight(VisionConstants.camera2Name, drive::getRotation),
            new VisionIOLimelight(VisionConstants.camera2Name, drive::getRotation),
            new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
        // vision =
        // new Vision(
        // demoDrive::addVisionMeasurement,
        // new VisionIOPhotonVision(camera0Name, robotToCamera0),
        // new VisionIOPhotonVision(camera1Name, robotToCamera1));
        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(TunerConstants.FrontLeft),
            new ModuleIOSim(TunerConstants.FrontRight),
            new ModuleIOSim(TunerConstants.BackLeft),
            new ModuleIOSim(TunerConstants.BackRight));

        // vision =
        // new Vision(
        // drive::addVisionMeasurement,
        // new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
        // new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        intake = new Intake(new RollerIOSim(), new PivotIOSim());
        shooter = new Shooter(new FeederIOSim(), new FlywheelIOSim());
        carpet = new Carpet(new CarpetIOSim());
        climber = new Climber(new ClimberIOSim());
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
        });
        break;
      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });

        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new
        // VisionIO() {});
        intake = new Intake(new RollerIO() {
        }, new PivotIO() {
        });
        shooter = new Shooter(new FeederIO() {
        }, new FlywheelIO() {
        });
        carpet = new Carpet(new CarpetIO() {
        });
        climber = new Climber(new ClimberIO() {
        });
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
        });
        break;

    }

    registerCommands();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureDriverBindings();

    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    RobotController.setBrownoutVoltage(6);

    SmartDashboard.putNumber("Match/MatchTime", Timer.getMatchTime());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureDriverBindings() {
    // Default Commands
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * 0.7,
            () -> -controller.getLeftX() * 0.7,
            () -> -controller.getRightX()));

    
    intake.setDefaultCommand(intake.stop());
    carpet.setDefaultCommand(carpet.runCarpetCommand(CarpetModes.IDLE));
    shooter.setDefaultCommand(shooter.stop());
    climber.setDefaultCommand(climber.runClimberCommand(ClimberModes.STOW));
    // shooter.setDefaultCommand();
    // // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> Rotation2d.kZero));

    // // Switch to X pattern when X button is pressed
    // driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

     //TODO: ready to test
      controller.y().whileTrue(
      new ParallelCommandGroup(
        DriveCommands.joystickDriveAtAngle(drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> getTranslationToGameElement().getAngle(), () -> getTranslationToGameElement())
        // shooter.runShooterCustom(2250, 2750)
        // shooter.tune()
        // new ShootFuelCommandAuto(shooter, vision, () -> getTranslationToGameElement().getNorm())
      )
    );
    // controller.y().whileTrue(shooter.runCustomVoltageCommand(5, 5));

    controller.a().whileTrue(new SuperShooterCommand(shooter, intake, carpet, ShooterModes.SHOOT_CLOSE));
    controller.x().whileTrue(new SuperShooterCommand(shooter, intake, carpet, ShooterModes.SHOOT_FAR));

    // controller.x().whileTrue(new ShootFuelCommandAuto(shooter, vision, () -> getTranslationToGameElement().getNorm()));

    // controller.a().whileTrue(Commands.defer(() -> {
    //   return new AutoShootCommand(intake, carpet, feeder, shooter, drive);
    // }, Set.of(intake, carpet, feeder, shooter)));x
    
    // controller.rightTrigger().whileTrue(new InhaleCommand(intake, carpet, true)); // TODO: run magic carpet, also when pivot is out, doesn't run if pivot is in
    
    controller.rightTrigger().whileTrue(intake.runIntakeCommand(IntakeMode.INTAKE_MORE));
    controller.rightBumper().whileTrue(intake.runIntakeCommand(IntakeMode.INTAKE));
    
    controller.leftBumper().whileTrue(intake.runIntakeCommand(IntakeMode.OUTTAKE));

    controller.povUp().whileTrue(intake.runPivotVoltageCommand(-6)); //TODO: change back to 5
    controller.povDown().whileTrue(intake.runPivotVoltageCommand(3)); //positive means down
    // controller.povUp().or(controller.povDown()).whileFalse(intake.runPivotVoltageCommand(0));
    controller.povRight().whileTrue(climber.runClimberCommand(ClimberModes.CLIMB));
    controller.povLeft().whileTrue(climber.runClimberCommand(ClimberModes.CLIMB_MINS));
    
    controller.rightTrigger().whileTrue(intake.runIntakeTuneCommand());
    // controller.povRight().onTrue(shooter.runAngler(ShooterModes.SHOOT_FAR));

    // controller.b().whileTrue(new IntakeJiggleCommand(intake));
    // controller.b().whileTrue(new ParallelCommandGroup(shooter.tune()));

    controller.start().whileTrue(new UnjamCommand(shooter));

  }

  private void registerCommands() {
    NamedCommands.registerCommand("ShootClose", Commands.sequence(
        new SuperShooterCommand(shooter, intake, carpet, ShooterModes.SHOOT_CLOSE)).withTimeout(10));
    // NamedCommands.registerCommand("DeployIntake");
    // NamedCommands.registerCommand("ShootCloseJer", );
    NamedCommands.registerCommand("ShootFar", Commands.sequence(
        new SuperShooterCommand(shooter, intake, carpet, ShooterModes.SHOOT_FAR)).withTimeout(10));

    



    // NamedCommands.registerCommand("AutoShoot", );
    NamedCommands.registerCommand("AutoShoot",
        new ShootFuelCommandAuto(shooter, vision, () -> getTranslationToGameElement().getNorm()));

    NamedCommands.registerCommand("AutoAlign",
        DriveCommands.joystickDriveAtAngle(drive, () -> 0, () -> 0, () -> getTranslationToGameElement().getAngle(), () -> getTranslationToGameElement()));

    NamedCommands.registerCommand("Intake", intake.runIntakeCommand(IntakeMode.INTAKE));
    NamedCommands.registerCommand("StopIntake", intake.runIntakeCommand(IntakeMode.OUT_IDLE));
    NamedCommands.registerCommand("deployIntake", intake.runPivotVoltageCommand(3));
    NamedCommands.registerCommand("RevShooterClose",
        shooter.runShooterCommand(ShooterModes.SHOOT_CLOSE).withTimeout(0.5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void teleopInit() {
    // if (!shooter.isZeroed()) {
    // CommandScheduler.getInstance().schedule(shooter.homing());
    // }
  }

  public Translation2d getTranslationToGameElement() {
    Translation2d hubPose = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
    Translation2d hubPoseAdj = AllianceFlipUtil.apply(hubPose);

    Pose2d robotPose = drive.getPose();
    Transform2d shooterTransform = new Transform2d(Inches.of(-10.824), Inches.of(0), Rotation2d.kZero);
    Pose2d shooterPose = robotPose.transformBy(shooterTransform);

    Translation2d diff = hubPoseAdj.minus(shooterPose.getTranslation());
    Logger.recordOutput("/Measurements/DistToHUb", diff.getNorm());
    Logger.recordOutput("/Measurements/AngleToHub", diff.getAngle());
    return diff;
  }

  // public Command getHomingCommand() {
  // return shooter.homing();
  // }
}
