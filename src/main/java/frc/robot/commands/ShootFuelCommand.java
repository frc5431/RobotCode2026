package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants.FeederModes;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterModes;

public class ShootFuelCommand extends SequentialCommandGroup {
  public ShootFuelCommand(Feeder feeder, Shooter shooter, ShooterModes shooterModes) {
    addCommands(
      // shooter.runShooterCommand(shooterModes).withName("ShootFuelCommand.Shoot")).until(() -> MathUtil.isNear(shooterModes.speed.magnitude(), shooter.getSpeed(), shooterModes.speed.magnitude() * 0.05),
      new ParallelCommandGroup(
        feeder.runFeederCommand(FeederModes.FEEDER).withName("FeederCommand.Feed"),
        shooter.runShooterCommand(shooterModes).withName("ShootFuelCommand.Shoot")
    ));
      
    addRequirements(feeder, shooter);
  }

  //TODO: once shooter hits rpm run feeder
}
