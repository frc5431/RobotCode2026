// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
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
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PassCommand;
import frc.robot.commands.SuperShooterCommand;

import java.util.Map;
import frc.robot.commands.InhaleCommand;

import frc.robot.commands.UnjamCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants.ClimberModes;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.Carpet;
import frc.robot.subsystems.hopper.CarpetConstants.CarpetModes;
import frc.robot.subsystems.hopper.CarpetIO;
import frc.robot.subsystems.hopper.CarpetIOSim;
import frc.robot.subsystems.hopper.CarpetIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;
import frc.robot.subsystems.intake.IntakeConstants.IntakePivotConstants;
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
import frc.robot.util.LimelightCameraStream;
import frc.team5431.titan.core.joysticks.CommandXboxController;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  
  private final Intake intake;
  private final Shooter shooter;
  private final Carpet carpet;
  private final Climber climber;
  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // ===========================================================================
  // Setup
  // ===========================================================================

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
      
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

     

        intake = new Intake(new RollerIOTalonFX(), new PivotIOTalonFX());
        shooter = new Shooter(new FeederIOTalonFX(), new FlywheelIOTalonFX());
        carpet = new Carpet(new CarpetIOTalonFX());
        climber = new Climber(new ClimberIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
              
                new VisionIOLimelight(VisionConstants.camera2Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

       
        LimelightCameraStream.publish(VisionConstants.camera1Name);
        LimelightCameraStream.publish(VisionConstants.camera2Name);
      
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

        intake = new Intake(new RollerIOSim(), new PivotIOSim());
        shooter = new Shooter(new FeederIOSim(), new FlywheelIOSim());
        carpet = new Carpet(new CarpetIOSim());
        climber = new Climber(new ClimberIOSim());
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
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

      
        intake = new Intake(new RollerIO() {}, new PivotIO() {});
        shooter = new Shooter(new FeederIO() {}, new FlywheelIO() {});
        carpet = new Carpet(new CarpetIO() {});
        climber = new Climber(new ClimberIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        break;
    }

    // Log the live shoot-map RPM estimate from the hub distance every loop.
    // shooter.setHubDistanceSupplier(drive::distFromHub);

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

  /** Registers the named commands used by PathPlanner autos. */
  private void registerCommands() {
    NamedCommands.registerCommand(
        "ShootClose",
        Commands.sequence(new SuperShooterCommand(shooter, carpet, intake, 2000)));

    NamedCommands.registerCommand(
        "UnjamShooter", Commands.sequence(new UnjamCommand(shooter)));

    NamedCommands.registerCommand(
        "ShootIdle",
        Commands.sequence(shooter.stop()));

    NamedCommands.registerCommand(
        "ShootFar",
        Commands.sequence(new SuperShooterCommand(shooter, carpet, intake, 4000))
            .withTimeout(10));

    NamedCommands.registerCommand(
        "AutoAlign",
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> 0,
            () -> 0,
            () -> getTranslationToGameElement().getAngle(),
            () -> getTranslationToGameElement()));

    NamedCommands.registerCommand("Intake", intake.runIntakeCommand(IntakeMode.INTAKE));
    NamedCommands.registerCommand("CarpetRun", carpet.runCarpetRPM(() -> 6500));
    NamedCommands.registerCommand("StopIntake", intake.runIntakeCommand(IntakeMode.OUT_IDLE));
    NamedCommands.registerCommand(
        "deployIntake",
        intake.runPivotPositionCommand(() -> IntakePivotConstants.downSetpoint.get())
            .withTimeout(1));

    NamedCommands.registerCommand("RunIntake", intake.runIntakeCommand(IntakeMode.INTAKE));
    NamedCommands.registerCommand(
        "RevShooterClose", shooter.runShooterCommand(ShooterModes.SHOOT_CLOSE).withTimeout(0.5));

    NamedCommands.registerCommand(
        "AutoShoot",
        new AutoShootCommand(drive, shooter, carpet, intake, () -> 0, () -> 0));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by instantiating
   * a {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link
   * XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureDriverBindings() {
    // ---- Default commands ----
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> Constants.DriveSpeedMultipler.get()));
    intake.setDefaultCommand(intake.stop());
    carpet.setDefaultCommand(carpet.runCarpetCommand(CarpetModes.IDLE));
    shooter.setDefaultCommand(shooter.stop());
    climber.setDefaultCommand(climber.runClimberCommand(ClimberModes.STOW));


    controller
        .y()
        .whileTrue(
            Commands.select(
                Map.of(
                    ShootMode.SHOOT,
                        new AutoShootCommand(
                            drive, shooter, carpet, intake,
                            () -> -controller.getLeftY(),
                            () -> -controller.getLeftX()),
                    ShootMode.PASS,
                        new PassCommand(
                            drive, shooter, carpet,
                            () -> -controller.getLeftY(),
                            () -> -controller.getLeftX()),
                    ShootMode.NONE, Commands.none()),
                () -> {
                  ShootMode m = pickMode();
                  Logger.recordOutput("SelectedMode", m);
                  return m;
                }));
    controller.a().whileTrue(new SuperShooterCommand(shooter, carpet, intake, 2000));
    controller.x().whileTrue(new SuperShooterCommand(shooter, carpet, intake, 4000));
  

    // Bumpers & triggers 
    controller.rightTrigger().whileTrue(intake.runIntakeCommand(IntakeMode.INTAKE));
    controller.rightBumper().whileTrue(carpet.runCarpetRPM(() -> 6500));
    controller.leftBumper().whileTrue(intake.runIntakeCommand(IntakeMode.OUTTAKE));
    controller.leftTrigger().whileTrue(new InhaleCommand(intake, carpet, true));

    
    controller.povUp().whileTrue(intake.runPivotPositionCommand(
        () -> IntakePivotConstants.upSetpoint.get())); // up (retract)
    controller.povDown().whileTrue(intake.runPivotPositionCommand(
        () -> IntakePivotConstants.downSetpoint.get())); // down (deploy)
    controller.povRight().whileTrue(new UnjamCommand(shooter));

   
   
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void teleopInit() {
    
  }



  private enum ShootMode { SHOOT, PASS, NONE }


  private static final Rectangle2d DEAD_ZONE = new Rectangle2d(
      new Translation2d(4.0, 8.0), new Translation2d(5.0, 0.0));
  public static final Rectangle2d shootZone = new Rectangle2d(
      new Translation2d(3.95, 7.95), new Translation2d(0.0, 0.0));
  public static final Rectangle2d passZoneFar = new Rectangle2d(
      new Translation2d(8.05, 12.8), new Translation2d(16.5, 0));
  public static final Rectangle2d passZone = new Rectangle2d(
      new Translation2d(5.05, 8.05), new Translation2d(11.4, 0.0));

  private ShootMode pickMode() {

    Translation2d pos = AllianceFlipUtil.apply(drive.getPose().getTranslation());
    if (DEAD_ZONE.contains(pos)) return ShootMode.NONE;
    if (shootZone.contains(pos)) return ShootMode.SHOOT;
    if (passZone.contains(pos) || passZoneFar.contains(pos)) return ShootMode.PASS;
    return ShootMode.NONE;
  }


  public Translation2d getTranslationToPassSpot() {
    Translation2d robot = drive.getPose().getTranslation();
    Translation2d left = AllianceFlipUtil.apply(FieldConstants.passSpotLeft);
    Translation2d right = AllianceFlipUtil.apply(FieldConstants.passSpotRight);
    Translation2d target = robot.getDistance(left) <= robot.getDistance(right) ? left : right;
    return target.minus(robot);
  }


 


  public Translation2d getTranslationToGameElement() {
    Translation2d hubPose = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
    Translation2d hubPoseAdj = AllianceFlipUtil.apply(hubPose);

    Pose2d robotPose = drive.getPose();
    Transform2d shooterTransform = new Transform2d(Inches.of(-10.824), Inches.of(0), Rotation2d.kZero);
    Pose2d shooterPose = robotPose.transformBy(shooterTransform);

    
    ChassisSpeeds speeds = drive.getChassisSpeeds();
    Translation2d fieldVel =
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            .rotateBy(drive.getRotation());

    Translation2d diff =
        hubPoseAdj.minus(shooterPose.getTranslation()).minus(fieldVel.times(AutoShootCommand.offset.getAsDouble()));
    Logger.recordOutput("/Measurements/DistToHub", diff.getNorm());
    Logger.recordOutput("/Measurements/AngleToHub", diff.getAngle());
    return diff;
  }

 
}
