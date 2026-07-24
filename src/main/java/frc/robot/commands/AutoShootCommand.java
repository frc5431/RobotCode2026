package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Carpet;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;


public class AutoShootCommand extends ParallelCommandGroup {

  private static final double FEEDER_RPM = 2850;
  private static final double CARPET_RPM = 6500;
  private static final double RPM_TOLERANCE = 0.99;
  private static final double SPINUP_TIMEOUT = 1.5;
  private static final double PIVOT_PERIOD_SECONDS = 1.5;
  private static final double PIVOT_UP_VOLTAGE = -6; // negative = up
  private static final double PIVOT_DOWN_VOLTAGE = 3; // positive = down

  public AutoShootCommand(
      Drive drive,
      Shooter shooter,
      Intake intake,
      Carpet carpet,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Translation2d> hubDiffSupplier) {

    DoubleSupplier dist = drive::distFromHub;

    addCommands(
      
        DriveCommands.joystickDriveAtAngle(
            drive, xSupplier, ySupplier, () -> hubDiffSupplier.get().getAngle(), hubDiffSupplier),

 
        Commands.sequence(
            shooter
                .runShootMap(dist, 0)
                .until(
                    () ->
                        shooter.getFlywheelSpeed()
                            >= shooter.getMapSpeed(dist.getAsDouble()) * RPM_TOLERANCE)
                .withTimeout(SPINUP_TIMEOUT),
            new ParallelCommandGroup(
                shooter.runShootMap(dist, FEEDER_RPM),
                carpet.runCarpetRPM(RPM.of(CARPET_RPM)),
              
                intake.runIntakeCommand(IntakeMode.INTAKE)
             
                // intake.runIntakePivotPulseCommand(
                //     IntakeMode.INTAKE,
                //     PIVOT_UP_VOLTAGE,
                //     PIVOT_DOWN_VOLTAGE,
                //     PIVOT_PERIOD_SECONDS)
                )));
  }
}
