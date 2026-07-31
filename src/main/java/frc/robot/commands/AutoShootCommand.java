package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.hopper.Carpet;
import frc.robot.subsystems.hopper.CarpetConstants.CarpetModes;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakePivotConstants;
import frc.robot.util.AllianceFlipUtil;


public final class AutoShootCommand extends ParallelCommandGroup {

    
    
    
    
    public static final LoggedNetworkNumber offset =
        new LoggedNetworkNumber("/Tuning/Commands/AutoShoot/sotmOffset", 0.12);
    private final Drive drive;
    private final Shooter shooter;
    private Translation2d hub;
    private Carpet carpet;
    private Intake intake;
    private ChassisSpeeds ChassisSpeeds;


    public AutoShootCommand(
            Drive drive,
            Shooter shooter,
            Carpet carpet,
            Intake intake,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {
        this.drive = drive;
        this.shooter = shooter;
        this.carpet = carpet;
        this.intake = intake;

        addCommands(

                DriveCommands.joystickDriveAtAngle(
                        drive,
                        xSupplier,
                        ySupplier,
                        () -> getHubDifference().getAngle(),
                        this::getHubDifference),


                Commands.sequence(
                        shooter.runShootMap(this::getDistanceToHub, () -> 0)
                               .until(this::isReadyToShoot),
                        Commands.parallel(
                                carpet.runCarpetCommand(CarpetModes.INTAKE),
                                shooter.runShootMap(this::getDistanceToHub, Constants.Feeder_RPM::getAsDouble),
                                Constants.PIVOT_PULSE.getAsBoolean() ? pivotPulse(intake) : Commands.none())),

                Commands.run(this::recordMeasurements)
        );
    }

    private static Command pivotPulse(Intake intake) {
        return Commands.sequence(
                new ParallelDeadlineGroup(
                        new WaitCommand(1.5),
                        intake.runPivotPositionCommand(() -> IntakePivotConstants.upSetpoint.get())),
                new ParallelDeadlineGroup(
                        new WaitCommand(0.5),
                        intake.runPivotPositionCommand(() -> IntakePivotConstants.downSetpoint.get())))
                .repeatedly();
    }


    private double getDistanceToHub() {
        return getHubDifference().getNorm();
    }


    private boolean isAligned() {

        double angleErrorDegrees =
                getHubDifference()
                        .getAngle()
                        .rotateBy(Rotation2d.fromDegrees(180))
                        .minus(drive.getPose().getRotation())
                        .getDegrees();

        return Math.abs(angleErrorDegrees) <= Constants.Align_Tolerance_Deg.getAsDouble();
    }

    private boolean isReadyToShoot() {
        return isAligned()
                && shooter.atSetpoint().getAsBoolean();
    }


    private Translation2d getHubDifference() {
        hub = AllianceFlipUtil.apply(
                FieldConstants.Hub.innerCenterPoint.toTranslation2d());

        Pose2d robotPose = drive.getPose();
        ChassisSpeeds = drive.getChassisSpeeds();

        Translation2d fieldVel = new Translation2d(
                ChassisSpeeds.vxMetersPerSecond,
                ChassisSpeeds.vyMetersPerSecond)
                .rotateBy(drive.getRotation());

        Translation2d offsetTranslation = fieldVel.times(offset.getAsDouble());

        return hub.minus(robotPose.getTranslation().plus(offsetTranslation));
    }

    private void recordMeasurements() {
        Translation2d diff = getHubDifference();

        Logger.recordOutput(
                "Measurements/DistToHub",
                diff.getNorm());

        Logger.recordOutput(
                "Measurements/AngleToHubDegrees",
                diff.getAngle().getDegrees());
    }
}
