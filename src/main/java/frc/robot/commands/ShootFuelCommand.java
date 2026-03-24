package frc.robot.commands;



import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterModes;


public class ShootFuelCommand extends SequentialCommandGroup {
  public ShootFuelCommand(Shooter shooter, ShooterModes shooterModes) {
    addCommands(
      shooter.runShooterCommand(shooterModes).withName("ShootFuelCommand.Shoot").
        until(() -> MathUtil.isNear(shooterModes.flywheelSpeed.magnitude(), shooter.getFlywheelSpeed(), shooterModes.flywheelSpeed.magnitude() * 0.10)).withTimeout(2),
      new ParallelCommandGroup(
        shooter.runShooterCommand(shooterModes).withName("ShootFuelCommand.Shoot")
    ));  
      
    addRequirements(shooter);
  }

  //TODO: once shooter hits rpm run feeder
}
