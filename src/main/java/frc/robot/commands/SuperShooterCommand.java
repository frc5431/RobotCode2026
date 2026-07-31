package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.hopper.Carpet;
import frc.robot.subsystems.hopper.CarpetConstants.CarpetModes;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakePivotConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.Constants;

public class SuperShooterCommand extends ParallelCommandGroup {



    public SuperShooterCommand(
            Shooter shooter,
            Carpet carpet,
            Intake intake,
            int shootFar) {

        addCommands(
               
                shooter.runShooterCustom(() -> shootFar, Constants.Feeder_RPM::getAsDouble),
                
                Commands.sequence(
                        Commands.waitUntil(shooter.atSetpoint()),
                        Commands.parallel(
                                carpet.runCarpetCommand(CarpetModes.INTAKE),
                                Constants.PIVOT_PULSE.getAsBoolean() ? pivotPulse(intake) : Commands.none()
)));

        addRequirements(shooter, carpet, intake);
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
}
