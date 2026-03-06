package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants.FeederModes;
import frc.robot.subsystems.hopper.Carpet;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterModes;

public class ShootFuelCommand extends SequentialCommandGroup {
  public ShootFuelCommand(Intake intake, Carpet carpet, Feeder feeder, Shooter shooter) {
    addCommands(
      new ParallelRaceGroup(
        feeder.runFeederCommand(FeederModes.REVERSE),
        new WaitCommand(0.5)),
      new ParallelCommandGroup(
        new InhaleCommand(intake, carpet, feeder,true, true).withName("ShootFuelCommand.Inhale")),
        shooter.runShooterCommand(ShooterModes.SHOOT_CLOSE).withName("ShootFuelCommand.Shoot")
      );
      
    addRequirements(intake, carpet, feeder, shooter);
  }
}
