package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.hopper.Carpet;
import frc.robot.subsystems.hopper.CarpetConstants.CarpetModes;
import frc.robot.util.AllianceFlipUtil;


public final class PassCommand extends ParallelCommandGroup {

    
     
        private final LoggedNetworkNumber PASS_RPM =
        new LoggedNetworkNumber("/Tuning/Commands/Passing/Pass_RPM", 4000);
        
    
    
    private final Drive drive;
    private final Shooter shooter;
    private Carpet carpet;

    public PassCommand(
            Drive drive,
            Shooter shooter,
            Carpet carpet,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {
        this.drive = drive;
        this.shooter = shooter;
        this.carpet = carpet;

        addCommands(

                DriveCommands.joystickDriveAtAngle(
                        drive,
                        xSupplier,
                        ySupplier,
                        () -> {
                          Logger.recordOutput("PassTarget", getTarget());
                          Logger.recordOutput(
                              "PassCmdActive",
                              getPassDifference().getAngle().getDegrees());
                          return getPassDifference().getAngle();
                        },
                        this::getPassDifference),

                Commands.sequence(
                        shooter.runShooterCustom(PASS_RPM::getAsDouble, () -> 0)
                               .until(this::isReadyToShoot),
                        Commands.parallel(
                                carpet.runCarpetCommand(CarpetModes.INTAKE),
                                shooter.runShooterCustom(PASS_RPM::getAsDouble, Constants.Feeder_RPM::getAsDouble)))

        );
    }


    private Translation2d getTarget() {
        Translation2d robot = drive.getPose().getTranslation();
        Translation2d left  = AllianceFlipUtil.apply(FieldConstants.passSpotLeft);
        Translation2d right = AllianceFlipUtil.apply(FieldConstants.passSpotRight);
        return robot.getDistance(left) <= robot.getDistance(right) ? left : right;
    }

    private Translation2d getPassDifference() {
        return getTarget().minus(drive.getPose().getTranslation());
    }


    private boolean isAligned() {
        
        double angleErrorDegrees =
                getPassDifference()
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
}
