package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.hopper.Carpet;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterModes;

public class SuperShooterCommand extends SequentialCommandGroup {
    public SuperShooterCommand(Shooter shooter, Intake intake, Carpet carpet, ShooterModes shooterModes){
        addCommands(
            shooter.runShooterCustom(shooterModes.flywheelSpeed.magnitude(), 0).until(
                () -> shooter.getFlywheelSpeed() > shooterModes.flywheelSpeed.magnitude() * 0.99
            ).withTimeout(0.5),
            new ParallelDeadlineGroup(
                new WaitCommand(1.25),
                shooter.runShooterCustom(shooterModes.flywheelSpeed.magnitude(), 2850),
                carpet.runCarpetRPM(Units.RPM.of(6500))
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.5), 
                intake.runPivotVoltageCommand(-5),
                shooter.runShooterCustom(shooterModes.flywheelSpeed.magnitude(), 2850),
                carpet.runCarpetRPM(Units.RPM.of(6500))
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.5), 
                intake.runPivotVoltageCommand(3),
                shooter.runShooterCustom(shooterModes.flywheelSpeed.magnitude(), 2850),
                carpet.runCarpetRPM(Units.RPM.of(6500))
            )
        );
        addRequirements(shooter, carpet);
    }
}
